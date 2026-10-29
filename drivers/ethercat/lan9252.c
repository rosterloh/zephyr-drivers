/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_lan9252

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/ethercat/lan9252.h>

LOG_MODULE_REGISTER(lan9252, CONFIG_LAN9252_LOG_LEVEL);

#define LAN9252_SPI_READ          0x03
#define LAN9252_SPI_WRITE         0x02
#define LAN9252_SPI_ADDR_AUTO_INC BIT(14)
#define LAN9252_SPI_ADDR_MASK     GENMASK(13, 0)
#define LAN9252_MAX_BURST_DWORDS  8

#define LAN9252_CSR_BUSY       BIT(31)
#define LAN9252_CSR_READ       BIT(30)
#define LAN9252_CSR_SIZE(size) FIELD_PREP(GENMASK(18, 16), (size))
#define LAN9252_CSR_ADDR(addr) FIELD_PREP(GENMASK(15, 0), (addr))

struct lan9252_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec int_gpio;
	uint32_t reset_assert_us;
	uint32_t reset_deassert_us;
	uint32_t init_timeout_ms;
};

struct lan9252_data {
	/* ESC CSR access is a multi-transfer command/poll/data sequence. */
	struct k_mutex esc_lock;
};

static bool lan9252_esc_size_valid(size_t len)
{
	return len == 1 || len == 2 || len == 4;
}

static uint16_t lan9252_spi_addr(uint16_t addr, size_t count)
{
	uint16_t spi_addr = addr & LAN9252_SPI_ADDR_MASK;

	if (count > 1) {
		spi_addr |= LAN9252_SPI_ADDR_AUTO_INC;
	}

	return spi_addr;
}

static uint32_t lan9252_load_le(const void *buf, size_t len)
{
	const uint8_t *bytes = buf;
	uint32_t value = 0;

	for (size_t i = 0; i < len; i++) {
		value |= (uint32_t)bytes[i] << (i * 8);
	}

	return value;
}

static void lan9252_store_le(uint32_t value, void *buf, size_t len)
{
	uint8_t *bytes = buf;

	for (size_t i = 0; i < len; i++) {
		bytes[i] = (uint8_t)(value >> (i * 8));
	}
}

static int lan9252_transfer(const struct device *dev, uint8_t command, uint16_t addr,
			    const uint32_t *tx_values, uint32_t *rx_values, size_t count)
{
	const struct lan9252_config *config = dev->config;
	uint8_t tx[3 + LAN9252_MAX_BURST_DWORDS * sizeof(uint32_t)] = {0};
	uint8_t rx[sizeof(tx)] = {0};

	if (count == 0 || count > LAN9252_MAX_BURST_DWORDS) {
		return -EINVAL;
	}

	if ((addr % sizeof(uint32_t)) != 0) {
		return -EINVAL;
	}

	uint16_t spi_addr = lan9252_spi_addr(addr, count);

	tx[0] = command;
	sys_put_be16(spi_addr, &tx[1]);

	if (command == LAN9252_SPI_WRITE) {
		if (tx_values == NULL) {
			return -EINVAL;
		}

		for (size_t i = 0; i < count; i++) {
			sys_put_le32(tx_values[i], &tx[3 + i * sizeof(uint32_t)]);
		}
	}

	struct spi_buf tx_buf = {
		.buf = tx,
		.len = 3 + count * sizeof(uint32_t),
	};
	struct spi_buf rx_buf = {
		.buf = rx,
		.len = tx_buf.len,
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};

	int ret = spi_transceive_dt(&config->spi, &tx_set, &rx_set);

	if (ret != 0) {
		return ret;
	}

	if (command == LAN9252_SPI_READ) {
		if (rx_values == NULL) {
			return -EINVAL;
		}

		for (size_t i = 0; i < count; i++) {
			rx_values[i] = sys_get_le32(&rx[3 + i * sizeof(uint32_t)]);
		}
	}

	return 0;
}

int lan9252_read_regs(const struct device *dev, uint16_t addr, uint32_t *values, size_t count)
{
	if (dev == NULL || values == NULL) {
		return -EINVAL;
	}

	return lan9252_transfer(dev, LAN9252_SPI_READ, addr, NULL, values, count);
}

int lan9252_write_regs(const struct device *dev, uint16_t addr, const uint32_t *values,
		       size_t count)
{
	if (dev == NULL || values == NULL) {
		return -EINVAL;
	}

	return lan9252_transfer(dev, LAN9252_SPI_WRITE, addr, values, NULL, count);
}

int lan9252_read_reg(const struct device *dev, uint16_t addr, uint32_t *value)
{
	return lan9252_read_regs(dev, addr, value, 1);
}

int lan9252_write_reg(const struct device *dev, uint16_t addr, uint32_t value)
{
	return lan9252_write_regs(dev, addr, &value, 1);
}

int lan9252_get_id(const struct device *dev, struct lan9252_id *id)
{
	uint32_t id_rev;
	int ret;

	if (id == NULL) {
		return -EINVAL;
	}

	ret = lan9252_read_reg(dev, LAN9252_REG_ID_REV, &id_rev);
	if (ret != 0) {
		return ret;
	}

	id->chip_id = FIELD_GET(GENMASK(31, 16), id_rev);
	id->revision = FIELD_GET(GENMASK(15, 0), id_rev);

	return 0;
}

static int lan9252_wait_csr_ready(const struct device *dev)
{
	const struct lan9252_config *config = dev->config;
	int64_t timeout = k_uptime_get() + config->init_timeout_ms;
	uint32_t command;
	int ret;

	do {
		ret = lan9252_read_reg(dev, LAN9252_REG_ECAT_CSR_CMD, &command);
		if (ret != 0) {
			return ret;
		}

		if ((command & LAN9252_CSR_BUSY) == 0) {
			return 0;
		}

		k_sleep(K_USEC(50));
	} while (k_uptime_get() <= timeout);

	return -ETIMEDOUT;
}

int lan9252_read_esc_reg(const struct device *dev, uint16_t addr, void *buf, size_t len)
{
	struct lan9252_data *data;
	uint32_t command;
	uint32_t value;
	int ret;

	if (dev == NULL || buf == NULL) {
		return -EINVAL;
	}

	data = dev->data;

	if (!lan9252_esc_size_valid(len)) {
		return -EINVAL;
	}

	if ((len == 2 && (addr & 0x1) != 0) || (len == 4 && (addr & 0x3) != 0)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->esc_lock, K_FOREVER);

	ret = lan9252_wait_csr_ready(dev);
	if (ret != 0) {
		goto out;
	}

	command = LAN9252_CSR_BUSY | LAN9252_CSR_READ | LAN9252_CSR_SIZE(len) |
		  LAN9252_CSR_ADDR(addr);
	ret = lan9252_write_reg(dev, LAN9252_REG_ECAT_CSR_CMD, command);
	if (ret != 0) {
		goto out;
	}

	ret = lan9252_wait_csr_ready(dev);
	if (ret != 0) {
		goto out;
	}

	ret = lan9252_read_reg(dev, LAN9252_REG_ECAT_CSR_DATA, &value);
	if (ret != 0) {
		goto out;
	}

	lan9252_store_le(value, buf, len);

out:
	k_mutex_unlock(&data->esc_lock);

	return ret;
}

int lan9252_write_esc_reg(const struct device *dev, uint16_t addr, const void *buf, size_t len)
{
	struct lan9252_data *data;
	uint32_t command;
	uint32_t value = 0;
	int ret;

	if (dev == NULL || buf == NULL) {
		return -EINVAL;
	}

	data = dev->data;

	if (!lan9252_esc_size_valid(len)) {
		return -EINVAL;
	}

	if ((len == 2 && (addr & 0x1) != 0) || (len == 4 && (addr & 0x3) != 0)) {
		return -EINVAL;
	}

	value = lan9252_load_le(buf, len);

	k_mutex_lock(&data->esc_lock, K_FOREVER);

	ret = lan9252_wait_csr_ready(dev);
	if (ret != 0) {
		goto out;
	}

	ret = lan9252_write_reg(dev, LAN9252_REG_ECAT_CSR_DATA, value);
	if (ret != 0) {
		goto out;
	}

	command = LAN9252_CSR_BUSY | LAN9252_CSR_SIZE(len) | LAN9252_CSR_ADDR(addr);
	ret = lan9252_write_reg(dev, LAN9252_REG_ECAT_CSR_CMD, command);
	if (ret != 0) {
		goto out;
	}

	ret = lan9252_wait_csr_ready(dev);

out:
	k_mutex_unlock(&data->esc_lock);

	return ret;
}

static int lan9252_reset(const struct device *dev)
{
	const struct lan9252_config *config = dev->config;
	int ret;

	if (config->reset_gpio.port == NULL) {
		return 0;
	}

	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_ERR("reset GPIO port is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (ret != 0) {
		return ret;
	}

	k_sleep(K_USEC(config->reset_assert_us));

	ret = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (ret != 0) {
		return ret;
	}

	k_sleep(K_USEC(config->reset_deassert_us));

	return 0;
}

static int lan9252_wait_for_value(const struct device *dev, uint16_t addr, uint32_t mask,
				  uint32_t expected)
{
	const struct lan9252_config *config = dev->config;
	int64_t timeout = k_uptime_get() + config->init_timeout_ms;
	uint32_t value;
	int ret;

	do {
		ret = lan9252_read_reg(dev, addr, &value);
		if (ret != 0) {
			return ret;
		}

		if ((value & mask) == expected) {
			return 0;
		}

		k_sleep(K_USEC(100));
	} while (k_uptime_get() <= timeout);

	return -ETIMEDOUT;
}

static int lan9252_init(const struct device *dev)
{
	const struct lan9252_config *config = dev->config;
	struct lan9252_data *data = dev->data;
	struct lan9252_id id;
	int ret;

	k_mutex_init(&data->esc_lock);

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

	ret = lan9252_reset(dev);
	if (ret != 0) {
		return ret;
	}

	ret = lan9252_wait_for_value(dev, LAN9252_REG_BYTE_TEST, UINT32_MAX,
				     LAN9252_BYTE_TEST_VALUE);
	if (ret != 0) {
		LOG_ERR("BYTE_TEST did not become valid: %d", ret);
		return ret;
	}

	ret = lan9252_wait_for_value(dev, LAN9252_REG_HW_CFG, LAN9252_HW_CFG_READY,
				     LAN9252_HW_CFG_READY);
	if (ret != 0) {
		LOG_ERR("device did not become ready: %d", ret);
		return ret;
	}

	ret = lan9252_get_id(dev, &id);
	if (ret != 0) {
		return ret;
	}

	if (id.chip_id != LAN9252_CHIP_ID) {
		LOG_ERR("unexpected LAN9252 chip id 0x%04x", id.chip_id);
		return -ENODEV;
	}

	LOG_INF("LAN9252 rev 0x%04x ready", id.revision);

	return 0;
}

#define LAN9252_DEFINE(inst)                                                                       \
	static const struct lan9252_config lan9252_config_##inst = {                               \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8) |       \
							  SPI_TRANSFER_MSB),                       \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                        \
		.reset_assert_us = DT_INST_PROP(inst, reset_assert_us),                            \
		.reset_deassert_us = DT_INST_PROP(inst, reset_deassert_us),                        \
		.init_timeout_ms = DT_INST_PROP(inst, init_timeout_ms),                            \
	};                                                                                         \
	static struct lan9252_data lan9252_data_##inst;                                            \
	DEVICE_DT_INST_DEFINE(inst, lan9252_init, NULL, &lan9252_data_##inst,                      \
			      &lan9252_config_##inst, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,       \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(LAN9252_DEFINE)

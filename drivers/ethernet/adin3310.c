/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adin3310

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/ethernet/adin3310.h>

LOG_MODULE_REGISTER(adin3310, CONFIG_ADIN3310_LOG_LEVEL);

struct adin3310_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec int_gpio;
	uint32_t reset_assert_us;
	uint32_t reset_deassert_us;
	uint8_t port_count;
};

int adin3310_get_port_count(const struct device *dev, uint8_t *port_count)
{
	const struct adin3310_config *config;

	if (dev == NULL || port_count == NULL) {
		return -EINVAL;
	}

	config = dev->config;
	*port_count = config->port_count;

	return 0;
}

int adin3310_reset(const struct device *dev)
{
	const struct adin3310_config *config;
	int ret;

	if (dev == NULL) {
		return -EINVAL;
	}

	config = dev->config;
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

int adin3310_transfer(const struct device *dev, const uint8_t *tx, uint8_t *rx, size_t len)
{
	const struct adin3310_config *config;
	struct spi_buf tx_buf;
	struct spi_buf rx_buf;
	struct spi_buf_set tx_set;
	struct spi_buf_set rx_set;

	if (dev == NULL || tx == NULL || rx == NULL || len == 0) {
		return -EINVAL;
	}

	config = dev->config;

	tx_buf = (struct spi_buf){
		.buf = (void *)tx,
		.len = len,
	};
	rx_buf = (struct spi_buf){
		.buf = rx,
		.len = len,
	};
	tx_set = (struct spi_buf_set){
		.buffers = &tx_buf,
		.count = 1,
	};
	rx_set = (struct spi_buf_set){
		.buffers = &rx_buf,
		.count = 1,
	};

	return spi_transceive_dt(&config->spi, &tx_set, &rx_set);
}

static int adin3310_init(const struct device *dev)
{
	const struct adin3310_config *config = dev->config;
	int ret;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

	ret = adin3310_reset(dev);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("ADIN3310 management interface ready");

	return 0;
}

#define ADIN3310_DEFINE(inst)                                                                      \
	static const struct adin3310_config adin3310_config_##inst = {                             \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8) |       \
							  SPI_TRANSFER_MSB),                       \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                        \
		.reset_assert_us = DT_INST_PROP(inst, reset_assert_us),                            \
		.reset_deassert_us = DT_INST_PROP(inst, reset_deassert_us),                        \
		.port_count = DT_INST_PROP(inst, port_count),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, adin3310_init, NULL, NULL, &adin3310_config_##inst,            \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ADIN3310_DEFINE)

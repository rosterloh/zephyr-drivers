/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_lan9252

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/ethercat/lan9252.h>

#define LAN9252_SPI_READ          0x03
#define LAN9252_SPI_WRITE         0x02
#define LAN9252_SPI_ADDR_AUTO_INC BIT(14)
#define LAN9252_SPI_ADDR_MASK     GENMASK(13, 0)

#define LAN9252_EMUL_REG_COUNT 256
#define LAN9252_EMUL_ESC_SIZE  0x1000

struct lan9252_emul_data {
	uint32_t regs[LAN9252_EMUL_REG_COUNT];
	uint8_t esc[LAN9252_EMUL_ESC_SIZE];
};

static uint32_t *lan9252_emul_reg(struct lan9252_emul_data *data, uint16_t addr)
{
	if ((addr % sizeof(uint32_t)) != 0) {
		return NULL;
	}

	size_t index = addr / sizeof(uint32_t);

	if (index >= ARRAY_SIZE(data->regs)) {
		return NULL;
	}

	return &data->regs[index];
}

static uint32_t lan9252_emul_load_le(const uint8_t *bytes, size_t len)
{
	uint32_t value = 0;

	for (size_t i = 0; i < len; i++) {
		value |= (uint32_t)bytes[i] << (i * 8);
	}

	return value;
}

static void lan9252_emul_store_le(uint32_t value, uint8_t *bytes, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		bytes[i] = (uint8_t)(value >> (i * 8));
	}
}

static void lan9252_emul_complete_csr_command(struct lan9252_emul_data *data, uint32_t command)
{
	uint16_t addr = command & GENMASK(15, 0);
	uint8_t size = FIELD_GET(GENMASK(18, 16), command);
	bool read = (command & BIT(30)) != 0;
	bool busy = (command & BIT(31)) != 0;
	uint32_t *csr_data = lan9252_emul_reg(data, LAN9252_REG_ECAT_CSR_DATA);
	uint32_t *csr_cmd = lan9252_emul_reg(data, LAN9252_REG_ECAT_CSR_CMD);

	if (!busy || csr_data == NULL || csr_cmd == NULL || size == 0 || size > sizeof(uint32_t) ||
	    addr + size > sizeof(data->esc)) {
		return;
	}

	if (read) {
		*csr_data = lan9252_emul_load_le(&data->esc[addr], size);
	} else {
		lan9252_emul_store_le(*csr_data, &data->esc[addr], size);
	}

	*csr_cmd = command & ~BIT(31);
}

static int lan9252_emul_copy_tx(const struct spi_buf_set *tx_bufs, uint8_t *tx, size_t tx_len)
{
	size_t offset = 0;

	if (tx_bufs == NULL) {
		return -EINVAL;
	}

	for (size_t i = 0; i < tx_bufs->count; i++) {
		const struct spi_buf *buf = &tx_bufs->buffers[i];

		if (buf->buf == NULL || offset + buf->len > tx_len) {
			return -EINVAL;
		}

		memcpy(&tx[offset], buf->buf, buf->len);
		offset += buf->len;
	}

	return (int)offset;
}

static int lan9252_emul_copy_rx(const struct spi_buf_set *rx_bufs, const uint8_t *rx, size_t rx_len)
{
	size_t offset = 0;

	if (rx_bufs == NULL) {
		return 0;
	}

	for (size_t i = 0; i < rx_bufs->count; i++) {
		const struct spi_buf *buf = &rx_bufs->buffers[i];

		if (buf->buf == NULL || offset + buf->len > rx_len) {
			return -EINVAL;
		}

		memcpy(buf->buf, &rx[offset], buf->len);
		offset += buf->len;
	}

	return 0;
}

static int lan9252_emul_io_spi(const struct emul *target, const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	ARG_UNUSED(config);

	struct lan9252_emul_data *data = target->data;
	uint8_t tx[64] = {0};
	uint8_t rx[64] = {0};
	int tx_len = lan9252_emul_copy_tx(tx_bufs, tx, sizeof(tx));

	if (tx_len < 3) {
		return -EINVAL;
	}

	uint16_t raw_addr = sys_get_be16(&tx[1]);
	uint16_t addr = raw_addr & LAN9252_SPI_ADDR_MASK;
	bool autoinc = (raw_addr & LAN9252_SPI_ADDR_AUTO_INC) != 0;
	size_t data_len = tx_len - 3;

	if ((data_len % sizeof(uint32_t)) != 0) {
		return -EINVAL;
	}

	for (size_t offset = 0; offset < data_len; offset += sizeof(uint32_t)) {
		uint16_t reg_addr = autoinc ? addr + offset : addr;
		uint32_t *reg = lan9252_emul_reg(data, reg_addr);

		if (reg == NULL) {
			return -EIO;
		}

		switch (tx[0]) {
		case LAN9252_SPI_READ:
			sys_put_le32(*reg, &rx[3 + offset]);
			break;
		case LAN9252_SPI_WRITE:
			*reg = sys_get_le32(&tx[3 + offset]);
			if (reg_addr == LAN9252_REG_ECAT_CSR_CMD) {
				lan9252_emul_complete_csr_command(data, *reg);
			}
			break;
		default:
			return -ENOTSUP;
		}
	}

	return lan9252_emul_copy_rx(rx_bufs, rx, sizeof(rx));
}

static int lan9252_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);

	struct lan9252_emul_data *data = target->data;

	memset(data, 0, sizeof(*data));
	*lan9252_emul_reg(data, LAN9252_REG_ID_REV) = (LAN9252_CHIP_ID << 16) | 0x0001;
	*lan9252_emul_reg(data, LAN9252_REG_BYTE_TEST) = LAN9252_BYTE_TEST_VALUE;
	*lan9252_emul_reg(data, LAN9252_REG_HW_CFG) = LAN9252_HW_CFG_READY;

	return 0;
}

static const struct spi_emul_api lan9252_emul_api = {
	.io = lan9252_emul_io_spi,
};

#define LAN9252_EMUL_DEFINE(inst)                                                                  \
	static struct lan9252_emul_data lan9252_emul_data_##inst;                                  \
	EMUL_DT_INST_DEFINE(inst, lan9252_emul_init, &lan9252_emul_data_##inst, NULL,              \
			    &lan9252_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(LAN9252_EMUL_DEFINE)

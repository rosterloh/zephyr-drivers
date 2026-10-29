/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adin3310

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>

static int adin3310_emul_copy_tx(const struct spi_buf_set *tx_bufs, uint8_t *tx, size_t tx_len)
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

static int adin3310_emul_copy_rx(const struct spi_buf_set *rx_bufs, const uint8_t *rx,
				 size_t rx_len)
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

static int adin3310_emul_io_spi(const struct emul *target, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	ARG_UNUSED(target);
	ARG_UNUSED(config);

	uint8_t tx[64] = {0};
	int tx_len = adin3310_emul_copy_tx(tx_bufs, tx, sizeof(tx));

	if (tx_len <= 0) {
		return -EINVAL;
	}

	return adin3310_emul_copy_rx(rx_bufs, tx, tx_len);
}

static int adin3310_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	return 0;
}

static const struct spi_emul_api adin3310_emul_api = {
	.io = adin3310_emul_io_spi,
};

#define ADIN3310_EMUL_DEFINE(inst)                                                                 \
	EMUL_DT_INST_DEFINE(inst, adin3310_emul_init, NULL, NULL, &adin3310_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(ADIN3310_EMUL_DEFINE)

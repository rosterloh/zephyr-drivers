/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "dynamixel_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(dynamixel, CONFIG_DYNAMIXEL_LOG_LEVEL);

#define DXL_INST_SYNC_READ   0x82
#define DXL_INST_SYNC_WRITE  0x83
#define DXL_INST_BULK_READ   0x92
#define DXL_INST_BULK_WRITE  0x93

/* Driver-side cap on per-call entries for BULK paths. Bounds the
 * stack-allocated `addrs[]` and `widths[]` working arrays. Sync paths do not
 * need this cap because they don't allocate per-entry stack arrays.
 */
#define DXL_BULK_MAX_ENTRIES 256

/* Pack a value of `width` bytes (1/2/4) into dst in little-endian. */
static inline void pack_le(uint8_t *dst, uint8_t width, uint32_t v)
{
	switch (width) {
	case 1: dst[0] = (uint8_t)v; break;
	case 2: sys_put_le16((uint16_t)v, dst); break;
	case 4: sys_put_le32(v, dst); break;
	default: break;
	}
}

static int sync_write_n(int iface, enum dxl_control item, uint8_t expected_width,
			const uint8_t ids[], const void *vals, size_t n)
{
	if (iface < 0 || n == 0 || ids == NULL || vals == NULL) {
		return -EINVAL;
	}

	struct dxl_context *ctx = dxl_get_context((uint8_t)iface);
	uint16_t addr;
	uint8_t width;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (dxl_table_lookup(item, &addr, &width) != 0) {
		return -EINVAL;
	}
	if (width != expected_width) {
		return -EINVAL;
	}

	/* params = addr_le16 (2) + data_len_le16 (2) + N * (1 + width).
	 * Use size_t through the math so huge n cannot wrap a uint16_t.
	 */
	size_t params_len = 4U + n * (1U + width);
	size_t length_field = 1U /* inst */ + params_len + 2U /* crc */;

	if (length_field + 7U > CONFIG_DYNAMIXEL_BUFFER_SIZE) {
		return -ENOSPC;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = 0xFE; /* broadcast */
	ctx->tx_frame.length = (uint16_t)length_field;
	ctx->tx_frame.ic = DXL_INST_SYNC_WRITE;
	sys_put_le16(addr, &ctx->tx_frame.data[0]);
	sys_put_le16(width, &ctx->tx_frame.data[2]);

	uint8_t *p = &ctx->tx_frame.data[4];
	const uint8_t *vbytes = vals; /* used for u8 path */
	const uint16_t *v16 = vals;
	const uint32_t *v32 = vals;

	for (size_t i = 0; i < n; i++) {
		*p++ = ids[i];
		switch (width) {
		case 1: pack_le(p, 1, vbytes[i]); break;
		case 2: pack_le(p, 2, v16[i]);    break;
		case 4: pack_le(p, 4, v32[i]);    break;
		}
		p += width;
	}

	/* SYNC_WRITE is broadcast; no status reply. The TX is IRQ-driven and
	 * runs to completion asynchronously after we release the mutex; the
	 * caller does not need to wait.
	 */
	dxl_serial_tx(ctx);

	k_mutex_unlock(&ctx->iface_lock);
	return 0;
}

int dxl_sync_write_u8(int iface, enum dxl_control item,
		      const uint8_t ids[], const uint8_t vals[], size_t n)
{
	return sync_write_n(iface, item, 1, ids, vals, n);
}

int dxl_sync_write_u16(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint16_t vals[], size_t n)
{
	return sync_write_n(iface, item, 2, ids, vals, n);
}

int dxl_sync_write_u32(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint32_t vals[], size_t n)
{
	return sync_write_n(iface, item, 4, ids, vals, n);
}

int dxl_sync_read_u8(int iface, enum dxl_control item,
		     const uint8_t ids[], uint8_t vals[], int errs[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(errs);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_read_u16(int iface, enum dxl_control item,
		      const uint8_t ids[], uint16_t vals[], int errs[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(errs);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_read_u32(int iface, enum dxl_control item,
		      const uint8_t ids[], uint32_t vals[], int errs[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(errs);
	ARG_UNUSED(n);
	return -ENOSYS;
}

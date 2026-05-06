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

int dxl_sync_write_u8(int iface, enum dxl_control item,
		      const uint8_t ids[], const uint8_t vals[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_write_u16(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint16_t vals[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_write_u32(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint32_t vals[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(n);
	return -ENOSYS;
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

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/dynamixel.h>

#include "fake_bus.h"

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE  DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

#define FAKE_BUS_RXWAIT_TO_US 10000U

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_bus bus;
static int iface;

static void sync_before_each(void *fixture)
{
	ARG_UNUSED(fixture);

	const uint8_t ids[] = {1, 2, 3, 4};
	struct dxl_iface_param p = {
		.rx_timeout = FAKE_BUS_RXWAIT_TO_US,
		.serial = {.baud = 115200, .parity = UART_CFG_PARITY_NONE},
	};

	fake_bus_init(&bus, ids, ARRAY_SIZE(ids));
	fake_bus_attach(&bus, uart_dev);

	iface = dxl_iface_get_by_name(DXL_IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(dxl_init(iface, p), "dxl_init failed");
}

static void sync_after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	dxl_disable(iface);
}

ZTEST_SUITE(dynamixel_sync, NULL, NULL, sync_before_each, sync_after_each, NULL);

ZTEST(dynamixel_sync, test_sync_write_u32_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	const uint32_t vals[] = {0x100, 0x200, 0x300, 0x400};

	zassert_ok(dxl_sync_write_u32(iface, GOAL_POSITION, ids, vals, ARRAY_SIZE(ids)),
		   "sync_write failed");

	/* GOAL_POSITION is at addr 116 (4 bytes). Each servo should have its slice. */
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 1), 116), 0x100, "id 1");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 2), 116), 0x200, "id 2");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 3), 116), 0x300, "id 3");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 4), 116), 0x400, "id 4");
}

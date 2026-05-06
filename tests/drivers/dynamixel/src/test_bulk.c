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

static void bulk_before_each(void *fixture)
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

static void bulk_after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	dxl_disable(iface);
}

ZTEST_SUITE(dynamixel_bulk, NULL, NULL, bulk_before_each, bulk_after_each, NULL);

ZTEST(dynamixel_bulk, test_bulk_read_happy_mixed_widths)
{
	const struct dxl_bulk_read_entry req[] = {
		{ .id = 1, .item = TORQUE_ENABLE    }, /* width 1, addr 64 */
		{ .id = 2, .item = GOAL_PWM         }, /* width 2, addr 100 */
		{ .id = 3, .item = PRESENT_POSITION }, /* width 4, addr 132 */
		{ .id = 4, .item = TORQUE_ENABLE    }, /* width 1 */
	};
	uint32_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	fake_bus_set_u8 (&bus, 1, 64,  1);
	fake_bus_set_u16(&bus, 2, 100, 0xCAFE);
	fake_bus_set_u32(&bus, 3, 132, 0xDEADBEEF);
	fake_bus_set_u8 (&bus, 4, 64,  0);

	zassert_ok(dxl_bulk_read(iface, req, vals, errs, ARRAY_SIZE(req)),
		   "bulk_read failed");

	zassert_equal(vals[0], 1, "vals[0]");
	zassert_equal(vals[1], 0xCAFE, "vals[1]");
	zassert_equal(vals[2], 0xDEADBEEF, "vals[2]");
	zassert_equal(vals[3], 0, "vals[3]");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], 0, "errs[2]");
	zassert_equal(errs[3], 0, "errs[3]");
}

ZTEST(dynamixel_bulk, test_bulk_read_validation_einval)
{
	const struct dxl_bulk_read_entry req[] = {
		{ .id = 1, .item = PRESENT_POSITION },
		{ .id = 2, .item = PRESENT_POSITION },
	};
	const struct dxl_bulk_read_entry bad_req[] = {
		{ .id = 1, .item = (enum dxl_control)9999 },
	};
	uint32_t vals[2] = {0};

	zassert_equal(dxl_bulk_read(iface, req, vals, NULL, 0),
		      -EINVAL, "n=0");
	zassert_equal(dxl_bulk_read(iface, NULL, vals, NULL, 2),
		      -EINVAL, "NULL req");
	zassert_equal(dxl_bulk_read(iface, req, NULL, NULL, 2),
		      -EINVAL, "NULL vals");
	zassert_equal(dxl_bulk_read(iface, bad_req, vals, NULL, 1),
		      -EINVAL, "bad item");
}

ZTEST(dynamixel_bulk, test_bulk_read_enospc_when_oversized)
{
	struct dxl_bulk_read_entry req[64];
	uint32_t vals[64];
	for (size_t i = 0; i < ARRAY_SIZE(req); i++) {
		req[i].id = (uint8_t)((i % 250) + 1);
		req[i].item = PRESENT_POSITION;
		vals[i] = 0;
	}

	/* BULK_READ packet is 10 + N*5 bytes. For N=64 -> 330 > 256 default. */
	zassert_equal(dxl_bulk_read(iface, req, vals, NULL, 64),
		      -ENOSPC, "oversized BULK_READ must return -ENOSPC");
}

ZTEST(dynamixel_bulk, test_bulk_write_happy_mixed_widths)
{
	const struct dxl_bulk_write_entry req[] = {
		{ .id = 1, .item = TORQUE_ENABLE,    .value = 1          },
		{ .id = 2, .item = GOAL_PWM,         .value = 0x0BEE     },
		{ .id = 3, .item = GOAL_POSITION,    .value = 0xDEADBEEF },
		{ .id = 4, .item = TORQUE_ENABLE,    .value = 0          },
	};

	zassert_ok(dxl_bulk_write(iface, req, ARRAY_SIZE(req)),
		   "bulk_write failed");

	/* BULK_WRITE is fire-and-forget at the protocol layer. The emulated
	 * UART drains TX bytes through a worker thread, so we yield long
	 * enough for the bytes to reach the fake_bus dispatcher before
	 * checking register state.
	 */
	k_sleep(K_MSEC(2));

	zassert_equal(fake_servo_get_u8 (fake_bus_get(&bus, 1), 64),  1,          "id 1");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 2), 100), 0x0BEE,     "id 2");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 3), 116), 0xDEADBEEF, "id 3");
	zassert_equal(fake_servo_get_u8 (fake_bus_get(&bus, 4), 64),  0,          "id 4");
}

ZTEST(dynamixel_bulk, test_bulk_write_validation_einval)
{
	const struct dxl_bulk_write_entry req[] = {
		{ .id = 1, .item = GOAL_POSITION, .value = 0 },
	};
	const struct dxl_bulk_write_entry bad_req[] = {
		{ .id = 1, .item = (enum dxl_control)9999, .value = 0 },
	};

	zassert_equal(dxl_bulk_write(iface, req, 0),
		      -EINVAL, "n=0");
	zassert_equal(dxl_bulk_write(iface, NULL, 1),
		      -EINVAL, "NULL req");
	zassert_equal(dxl_bulk_write(iface, bad_req, 1),
		      -EINVAL, "bad item");
}

ZTEST(dynamixel_bulk, test_bulk_write_enospc_when_oversized)
{
	struct dxl_bulk_write_entry req[32];
	for (size_t i = 0; i < ARRAY_SIZE(req); i++) {
		req[i].id = (uint8_t)((i % 250) + 1);
		req[i].item = GOAL_POSITION; /* width 4 */
		req[i].value = 0;
	}

	/* BULK_WRITE packet is 10 + N*(5+L). For N=32, L=4 -> 298 > 256 default. */
	zassert_equal(dxl_bulk_write(iface, req, 32),
		      -ENOSPC, "oversized BULK_WRITE must return -ENOSPC");
}

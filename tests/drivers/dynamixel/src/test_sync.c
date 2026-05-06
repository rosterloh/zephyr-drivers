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

	/* SYNC_WRITE is fire-and-forget at the protocol layer. The emulated
	 * UART drains TX bytes through a worker thread, so we yield long
	 * enough for the bytes to reach the fake_bus dispatcher before
	 * checking register state. 2 ms is far more than the ~30-byte packet
	 * needs at 115200 baud (~2.6 ms wall) and far less than the test
	 * fixture's rxwait_to.
	 */
	k_sleep(K_MSEC(2));

	/* GOAL_POSITION is at addr 116 (4 bytes). Each servo should have its slice. */
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 1), 116), 0x100, "id 1");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 2), 116), 0x200, "id 2");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 3), 116), 0x300, "id 3");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 4), 116), 0x400, "id 4");
}

ZTEST(dynamixel_sync, test_sync_write_u8_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	const uint8_t vals[] = {1, 0, 1, 0};

	zassert_ok(dxl_sync_write_u8(iface, TORQUE_ENABLE, ids, vals, ARRAY_SIZE(ids)),
		   "sync_write_u8 failed");

	/* SYNC_WRITE is fire-and-forget at the protocol layer; allow the
	 * emulated UART worker to drain before checking register state.
	 */
	k_sleep(K_MSEC(2));

	/* TORQUE_ENABLE is at addr 64 (1 byte). */
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 1), 64), 1, "id 1");
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 2), 64), 0, "id 2");
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 3), 64), 1, "id 3");
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 4), 64), 0, "id 4");
}

ZTEST(dynamixel_sync, test_sync_write_u16_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	const uint16_t vals[] = {0x0AAA, 0x0BBB, 0x0CCC, 0x0DDD};

	zassert_ok(dxl_sync_write_u16(iface, GOAL_PWM, ids, vals, ARRAY_SIZE(ids)),
		   "sync_write_u16 failed");

	k_sleep(K_MSEC(2));

	/* GOAL_PWM is at addr 100 (2 bytes). */
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 1), 100), 0x0AAA, "id 1");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 2), 100), 0x0BBB, "id 2");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 3), 100), 0x0CCC, "id 3");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 4), 100), 0x0DDD, "id 4");
}

ZTEST(dynamixel_sync, test_sync_write_width_mismatch_returns_einval)
{
	const uint8_t ids[] = {1, 2};
	const uint8_t vals_u8[] = {0, 0};
	const uint16_t vals_u16[] = {0, 0};

	/* u8 helper called on a 4-byte register (GOAL_POSITION). */
	zassert_equal(dxl_sync_write_u8(iface, GOAL_POSITION, ids, vals_u8, 2),
		      -EINVAL, "u8 on 4-byte register");

	/* u16 helper called on a 1-byte register (TORQUE_ENABLE). */
	zassert_equal(dxl_sync_write_u16(iface, TORQUE_ENABLE, ids, vals_u16, 2),
		      -EINVAL, "u16 on 1-byte register");
}

ZTEST(dynamixel_sync, test_sync_write_validation_einval)
{
	const uint8_t ids[] = {1, 2};
	const uint32_t vals[] = {0, 0};

	/* n = 0 is invalid. */
	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, ids, vals, 0),
		      -EINVAL, "n=0");

	/* NULL ids. */
	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, NULL, vals, 2),
		      -EINVAL, "NULL ids");

	/* NULL vals. */
	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, ids, NULL, 2),
		      -EINVAL, "NULL vals");

	/* Out-of-range item. */
	zassert_equal(dxl_sync_write_u32(iface, (enum dxl_control)9999, ids, vals, 2),
		      -EINVAL, "bad item");
}

ZTEST(dynamixel_sync, test_sync_write_enospc_when_oversized)
{
	/* CONFIG_DYNAMIXEL_BUFFER_SIZE defaults to 256. SYNC_WRITE u32 with N
	 * servos needs 14 + N*5 bytes. N=49 -> 259 > 256. Use a fabricated
	 * over-large id list (the dispatcher won't actually fire — call returns
	 * -ENOSPC before TX).
	 */
	uint8_t ids[64];
	uint32_t vals[64];
	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		ids[i] = (uint8_t)(i + 1);
		vals[i] = 0;
	}

	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, ids, vals, 64),
		      -ENOSPC, "oversized request must return -ENOSPC");
}

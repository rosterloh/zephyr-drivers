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

ZTEST(dynamixel_sync, test_sync_read_u32_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	/* PRESENT_POSITION at addr 132 (4 bytes). */
	fake_bus_set_u32(&bus, 1, 132, 0x11111111);
	fake_bus_set_u32(&bus, 2, 132, 0x22222222);
	fake_bus_set_u32(&bus, 3, 132, 0x33333333);
	fake_bus_set_u32(&bus, 4, 132, 0x44444444);

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));
	zassert_ok(rc, "sync_read returned %d", rc);

	zassert_equal(vals[0], 0x11111111, "vals[0]");
	zassert_equal(vals[1], 0x22222222, "vals[1]");
	zassert_equal(vals[2], 0x33333333, "vals[2]");
	zassert_equal(vals[3], 0x44444444, "vals[3]");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], 0, "errs[2]");
	zassert_equal(errs[3], 0, "errs[3]");
}

ZTEST(dynamixel_sync, test_sync_read_u8_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint8_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	/* TORQUE_ENABLE at addr 64 (1 byte). */
	fake_bus_set_u8(&bus, 1, 64, 1);
	fake_bus_set_u8(&bus, 2, 64, 0);
	fake_bus_set_u8(&bus, 3, 64, 1);
	fake_bus_set_u8(&bus, 4, 64, 0);

	zassert_ok(dxl_sync_read_u8(iface, TORQUE_ENABLE, ids, vals, errs,
				    ARRAY_SIZE(ids)),
		   "sync_read_u8 failed");
	zassert_equal(vals[0], 1, "");
	zassert_equal(vals[1], 0, "");
	zassert_equal(vals[2], 1, "");
	zassert_equal(vals[3], 0, "");
}

ZTEST(dynamixel_sync, test_sync_read_u16_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint16_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	/* GOAL_PWM at addr 100 (2 bytes). */
	fake_bus_set_u16(&bus, 1, 100, 0x1111);
	fake_bus_set_u16(&bus, 2, 100, 0x2222);
	fake_bus_set_u16(&bus, 3, 100, 0x3333);
	fake_bus_set_u16(&bus, 4, 100, 0x4444);

	zassert_ok(dxl_sync_read_u16(iface, GOAL_PWM, ids, vals, errs,
				     ARRAY_SIZE(ids)),
		   "sync_read_u16 failed");
	zassert_equal(vals[0], 0x1111, "");
	zassert_equal(vals[1], 0x2222, "");
	zassert_equal(vals[2], 0x3333, "");
	zassert_equal(vals[3], 0x4444, "");
}

ZTEST(dynamixel_sync, test_sync_read_partial_drop)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED};
	int errs[4] = {0};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	/* Servo 3 silently fails to respond. Servos 1/2/4 reply normally. */
	fake_bus_get(&bus, 3)->drop_response = true;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO, "summary must be -EIO when any slot fails, got %d", rc);
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], -ETIMEDOUT, "errs[2] expected -ETIMEDOUT, got %d", errs[2]);
	zassert_equal(errs[3], 0, "errs[3]");

	zassert_equal(vals[0], 0xAAAA0001, "vals[0]");
	zassert_equal(vals[1], 0xAAAA0002, "vals[1]");
	zassert_equal(vals[2], 0xFEEDFEED, "vals[2] must be untouched on drop");
	zassert_equal(vals[3], 0xAAAA0004, "vals[3]");
}

ZTEST(dynamixel_sync, test_sync_read_device_error_byte)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0};
	int errs[4] = {0};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	/* Servo 2 returns a device-error byte. The driver treats it as a
	 * positive return code in errs[1]; vals[1] is left untouched.
	 */
	fake_bus_get(&bus, 2)->error_byte = DXL_ERR_DATA_RANGE;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO, "summary must be -EIO");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], DXL_ERR_DATA_RANGE,
		      "errs[1] must be device-error byte, got %d", errs[1]);
	zassert_equal(errs[2], 0, "errs[2]");
	zassert_equal(errs[3], 0, "errs[3]");
	zassert_equal(vals[1], 0, "vals[1] must be untouched on dev err");
	zassert_equal(vals[0], 0xAAAA0001, "vals[0]");
	zassert_equal(vals[2], 0xAAAA0003, "vals[2]");
	zassert_equal(vals[3], 0xAAAA0004, "vals[3]");
}

ZTEST(dynamixel_sync, test_sync_read_wrong_id_reply_is_timeout)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0};
	int errs[4] = {0};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	/* For slot 2 (id=3 expected), make servo 3 reply with a different id (5).
	 * The driver discards the wrong-id frame in dxl_rx_handler (-EBADMSG)
	 * and silently re-enables RX. With no other reply incoming, the slot
	 * times out as -ETIMEDOUT. Subsequent slots still proceed.
	 */
	struct fake_servo *s3 = fake_bus_get(&bus, 3);
	s3->id = 5; /* respond as id 5 even though id 3 was requested */
	s3->answer_any_id = true;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO, "summary must be -EIO");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], -ETIMEDOUT,
		      "errs[2] expected -ETIMEDOUT for wrong-id, got %d", errs[2]);
	zassert_equal(errs[3], 0, "errs[3]");
}

ZTEST(dynamixel_sync, test_sync_read_errs_null_collapses_to_eio)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	fake_bus_get(&bus, 3)->drop_response = true;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals,
				   /*errs=*/NULL, ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO,
		      "errs=NULL must still collapse failures to -EIO, got %d", rc);
	zassert_equal(vals[0], 0xAAAA0001, "vals[0]");
	zassert_equal(vals[1], 0xAAAA0002, "vals[1]");
	zassert_equal(vals[2], 0xFEEDFEED,
		      "vals[2] must be untouched even when errs is NULL");
	zassert_equal(vals[3], 0xAAAA0004, "vals[3]");
}

ZTEST(dynamixel_sync, test_sync_read_validation_einval)
{
	const uint8_t ids[] = {1, 2};
	uint32_t vals[2] = {0};

	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, NULL, 0),
		      -EINVAL, "n=0");
	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, NULL, vals, NULL, 2),
		      -EINVAL, "NULL ids");
	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, ids, NULL, NULL, 2),
		      -EINVAL, "NULL vals");
	zassert_equal(dxl_sync_read_u32(iface, (enum dxl_control)9999, ids, vals, NULL, 2),
		      -EINVAL, "bad item");

	uint8_t vals_u8[2] = {0};
	zassert_equal(dxl_sync_read_u8(iface, PRESENT_POSITION, ids, vals_u8, NULL, 2),
		      -EINVAL, "u8 on 4-byte register");
}

ZTEST(dynamixel_sync, test_sync_read_enospc_when_oversized)
{
	uint8_t ids[256];
	uint32_t vals[256];
	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		ids[i] = (uint8_t)((i % 250) + 1);
		vals[i] = 0;
	}

	/* SYNC_READ packet is 14 + N bytes. For N=256 -> 270 > 256 default. */
	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, NULL, 256),
		      -ENOSPC, "oversized SYNC_READ must return -ENOSPC");
}

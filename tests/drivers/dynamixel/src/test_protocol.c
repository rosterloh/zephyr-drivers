/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Phase 1 BUG-PINNING TESTS. These intentionally assert current
 * (buggy) behavior so they are green now. Phase 3 of the cleanup
 * flips them.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/dynamixel.h>

#include "fake_servo.h"

#define DXL_BUS_NODE      DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE     DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME    DEVICE_DT_NAME(DXL_BUS_NODE)

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_servo srv;
static int iface;

static void bring_up(uint8_t id)
{
	struct dxl_iface_param p = {
		.rx_timeout = 50000,
		.serial = { .baud = 115200, .parity = UART_CFG_PARITY_NONE },
	};

	fake_servo_init(&srv, id);
	fake_servo_attach(&srv, uart_dev);

	iface = dxl_iface_get_by_name(DXL_IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(dxl_init(iface, p), "dxl_init failed");
}

static void tear_down(void)
{
	dxl_disable(iface);
}

ZTEST(dynamixel_protocol, test_phase1_ping_request_bytes)
{
	bring_up(1);

	zassert_ok(dxl_ping(iface, 1), "ping failed");

	zassert_equal(srv.last_tx[0], 0xFF, "header byte 0");
	zassert_equal(srv.last_tx[1], 0xFF, "header byte 1");
	zassert_equal(srv.last_tx[2], 0xFD, "header byte 2");
	zassert_equal(srv.last_tx[3], 0x00, "reserved");
	zassert_equal(srv.last_tx[4], 1,    "id");
	zassert_equal(srv.last_tx[7], 0x01, "ping instruction");
	zassert_equal(srv.last_instruction, 0x01, "captured instruction");

	tear_down();
}

ZTEST(dynamixel_protocol, test_phase1_read_u32_truncated)
{
	uint32_t val32 = 0;

	bring_up(1);
	fake_servo_set_u32(&srv, 132 /* PRESENT_POSITION addr */, 0x12345678);

	(void)dxl_read(iface, 1, PRESENT_POSITION, &val32);

	/* BUG: existing code uses sys_get_le16 even for 4-byte regs. */
	zassert_equal(val32, 0x00005678,
		      "phase 1 expects truncated low 16 bits, got 0x%08x", val32);

	tear_down();
}

ZTEST(dynamixel_protocol, test_phase1_timeout_overwrites_err)
{
	uint32_t val32 = 0;
	int rc;

	bring_up(1);
	srv.drop_response = true;

	rc = dxl_read(iface, 1, PRESENT_POSITION, &val32);

	/* BUG: dxl_read overwrites the timeout return code with rx_frame.data[0],
	 * which on timeout is uninitialised / leftover. We assert that rc is NOT
	 * -ETIMEDOUT — the right value the API ought to return — to make the bug
	 * visible. Phase 3 inverts this: rc MUST be -ETIMEDOUT.
	 */
	zassert_not_equal(rc, -ETIMEDOUT,
			  "phase 1 expects err to be overwritten, not -ETIMEDOUT");

	tear_down();
}

ZTEST(dynamixel_protocol, test_phase1_write_u8_round_trip)
{
	bring_up(1);

	zassert_ok(dxl_write(iface, 1, TORQUE_ENABLE, 1), "write failed");

	zassert_equal(srv.last_instruction, 0x03, "write instruction");
	zassert_equal(srv.last_addr,        64,   "TORQUE_ENABLE addr");
	zassert_equal(srv.last_length,      1,    "1-byte param");
	zassert_equal(fake_servo_get_u8(&srv, 64), 1, "RAM updated");

	tear_down();
}

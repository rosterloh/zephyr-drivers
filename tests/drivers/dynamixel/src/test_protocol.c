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

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE  DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_servo srv;
static int iface;

static void bring_up_default(void)
{
	struct dxl_iface_param p = {
		.rx_timeout = 50000,
		.serial = {.baud = 115200, .parity = UART_CFG_PARITY_NONE},
	};

	fake_servo_init(&srv, 1);
	fake_servo_attach(&srv, uart_dev);

	iface = dxl_iface_get_by_name(DXL_IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(dxl_init(iface, p), "dxl_init failed");
}

static void protocol_before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	bring_up_default();
}

static void protocol_after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	dxl_disable(iface);
}

ZTEST_SUITE(dynamixel_protocol, NULL, NULL, protocol_before_each, protocol_after_each, NULL);

ZTEST(dynamixel_protocol, test_phase1_ping_request_bytes)
{
	zassert_ok(dxl_ping(iface, 1), "ping failed");

	zassert_equal(srv.last_tx[0], 0xFF, "header byte 0");
	zassert_equal(srv.last_tx[1], 0xFF, "header byte 1");
	zassert_equal(srv.last_tx[2], 0xFD, "header byte 2");
	zassert_equal(srv.last_tx[3], 0x00, "reserved");
	zassert_equal(srv.last_tx[4], 1, "id");
	zassert_equal(srv.last_tx[7], 0x01, "ping instruction");
	zassert_equal(srv.last_instruction, 0x01, "captured instruction");
}

ZTEST(dynamixel_protocol, test_timeout_returns_etimedout)
{
	uint32_t val32 = 0;
	int rc;

	srv.drop_response = true;

	rc = dxl_read_u32(iface, 1, PRESENT_POSITION, &val32);

	zassert_equal(rc, -ETIMEDOUT, "timeout must return -ETIMEDOUT, got %d", rc);
}

ZTEST(dynamixel_protocol, test_write_u8_round_trip)
{
	zassert_ok(dxl_write_u8(iface, 1, TORQUE_ENABLE, 1), "write failed");

	zassert_equal(srv.last_instruction, 0x03, "write instruction");
	zassert_equal(srv.last_addr, 64, "TORQUE_ENABLE addr");
	zassert_equal(srv.last_length, 1, "1-byte param");
	zassert_equal(fake_servo_get_u8(&srv, 64), 1, "RAM updated");
}

ZTEST(dynamixel_protocol, test_phase3_wrong_id_response_rejected)
{
	int rc;

	/* Force the fake to reply to the request even though its id (2) does not
	 * match the addressed id (1). The driver should detect the mismatched
	 * response id and discard it silently (no k_sem_give), so the
	 * wait_sem genuinely times out and the client sees -ETIMEDOUT.
	 */
	srv.id = 2;
	srv.answer_any_id = true;

	rc = dxl_ping(iface, 1);

	zassert_equal(rc, -ETIMEDOUT, "wrong-id reply should look like a timeout, got %d", rc);
}

ZTEST(dynamixel_protocol, test_read_u32_full_value)
{
	uint32_t val = 0;

	fake_servo_set_u32(&srv, 132 /* PRESENT_POSITION addr */, 0x12345678);

	zassert_ok(dxl_read_u32(iface, 1, PRESENT_POSITION, &val), "read failed");
	zassert_equal(val, 0x12345678, "got 0x%08x", val);
}

ZTEST(dynamixel_protocol, test_read_u16_full_value)
{
	uint16_t val = 0;

	fake_servo_set_u16(&srv, 100 /* GOAL_PWM addr */, 0xCAFE);

	zassert_ok(dxl_read_u16(iface, 1, GOAL_PWM, &val), "read failed");
	zassert_equal(val, 0xCAFE, "got 0x%04x", val);
}

ZTEST(dynamixel_protocol, test_read_u8_full_value)
{
	uint8_t val = 0;

	fake_servo_set_u8(&srv, 64 /* TORQUE_ENABLE addr */, 0xA5);

	zassert_ok(dxl_read_u8(iface, 1, TORQUE_ENABLE, &val), "read failed");
	zassert_equal(val, 0xA5, "got 0x%02x", val);
}

ZTEST(dynamixel_protocol, test_write_u32_round_trip)
{
	zassert_ok(dxl_write_u32(iface, 1, GOAL_POSITION, 0xDEADBEEF), "write failed");

	zassert_equal(srv.last_addr, 116, "GOAL_POSITION addr");
	zassert_equal(srv.last_length, 4, "4-byte param");
	zassert_equal(fake_servo_get_u32(&srv, 116), 0xDEADBEEF, "RAM");
}

ZTEST(dynamixel_protocol, test_width_mismatch_returns_einval)
{
	uint8_t val8;

	zassert_equal(dxl_read_u8(iface, 1, PRESENT_POSITION, &val8), -EINVAL,
		      "u8 read of 4-byte register");

	zassert_equal(dxl_write_u16(iface, 1, TORQUE_ENABLE, 0), -EINVAL,
		      "u16 write of 1-byte register");
}

ZTEST(dynamixel_protocol, test_invalid_register_returns_einval)
{
	uint32_t val32;

	zassert_equal(dxl_read_u32(iface, 1, (enum dxl_control)9999, &val32), -EINVAL,
		      "out-of-range register");
}

ZTEST(dynamixel_protocol, test_device_error_byte_returned_positive)
{
	uint32_t val = 0;
	int rc;

	srv.error_byte = DXL_ERR_DATA_RANGE;
	fake_servo_set_u32(&srv, 132, 0x11);

	rc = dxl_read_u32(iface, 1, PRESENT_POSITION, &val);

	zassert_equal(rc, DXL_ERR_DATA_RANGE,
		      "device error must be returned as positive enum, got %d", rc);
}

ZTEST(dynamixel_protocol, test_crc_error_returns_eio)
{
	uint32_t val = 0;
	int rc;

	srv.corrupt_crc = true;
	fake_servo_set_u32(&srv, 132 /* PRESENT_POSITION */, 0xFEEDFACE);

	rc = dxl_read_u32(iface, 1, PRESENT_POSITION, &val);

	zassert_equal(rc, -EIO, "corrupted CRC should return -EIO, got %d", rc);
}

ZTEST(dynamixel_protocol, test_write_u16_round_trip)
{
	zassert_ok(dxl_write_u16(iface, 1, GOAL_PWM, 0x0BEE), "write failed");

	zassert_equal(srv.last_instruction, 0x03, "write instruction");
	zassert_equal(srv.last_addr, 100, "GOAL_PWM addr");
	zassert_equal(srv.last_length, 2, "2-byte param");
	zassert_equal(fake_servo_get_u16(&srv, 100), 0x0BEE, "RAM updated");
}

ZTEST(dynamixel_protocol, test_reboot_request_bytes)
{
	zassert_ok(dxl_reboot(iface, 1), "reboot failed");

	zassert_equal(srv.last_instruction, 0x08, "reboot instruction");
	zassert_equal(srv.last_tx[4], 1, "id");
}

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/actuator_group.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>
#include <drivers/bus_servo.h>

#include "fake_bus.h"

#define BUS_NODE   DT_NODELABEL(bus_servo0)
#define UART_NODE  DT_PARENT(BUS_NODE)
#define IFACE_NAME DEVICE_DT_NAME(BUS_NODE)

#define PAN      DEVICE_DT_GET(DT_NODELABEL(pan))
#define TILT     DEVICE_DT_GET(DT_NODELABEL(tilt))
#define INVERTED DEVICE_DT_GET(DT_NODELABEL(inverted))

#define PI_F 3.1415927f

ACTUATOR_GROUP_DEFINE(gimbal, PAN, TILT);
ACTUATOR_GROUP_DEFINE(oversized_gimbal, PAN, TILT, PAN, TILT, PAN, TILT, PAN, TILT);

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static struct fake_bus bus;
static int iface;
static volatile int pan_state_cb_count;
static enum actuator_state pan_last_state_seen;

static void on_pan_state(const struct device *dev, enum actuator_state state, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	pan_state_cb_count++;
	pan_last_state_seen = state;
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	struct bus_servo_iface_param param = {
		.rx_timeout_us = 50000,
		.serial = {.baud = 115200, .parity = UART_CFG_PARITY_NONE},
	};

	fake_bus_init(&bus);
	fake_bus_attach(&bus, uart_dev);
	iface = bus_servo_iface_get_by_name(IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(bus_servo_init(iface, param), "bus_servo_init failed");
}

static void after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	bus_servo_disable(iface);
}

ZTEST_SUITE(bus_servo_actuator, NULL, NULL, before_each, after_each, NULL);

ZTEST(bus_servo_actuator, test_enable_writes_torque_on)
{
	zassert_true(device_is_ready(PAN));
	zassert_ok(actuator_enable(PAN));

	const uint8_t expected[] = {
		0xff, 0xff, 0x02, 0x04, 0x03, 0x28, 0x01, 0xcd,
	};
	zassert_equal(bus.last_tx_len, sizeof(expected));
	zassert_mem_equal(bus.last_tx, expected, sizeof(expected));
}

ZTEST(bus_servo_actuator, test_set_position_writes_position_ex_frame)
{
	zassert_ok(actuator_enable(PAN));
	bus.last_tx_len = 0;

	zassert_ok(actuator_set_position(PAN, PI_F / 2.0f));

	const uint8_t expected[] = {
		0xff, 0xff, 0x02, 0x0a, 0x03, 0x29, 0x14, 0xff, 0x0b, 0x00, 0x00, 0x2c, 0x01, 0x7c,
	};
	zassert_equal(bus.last_tx_len, sizeof(expected));
	zassert_mem_equal(bus.last_tx, expected, sizeof(expected));
}

/*
 * Present-state block, read in one transaction from reg 56:
 *
 *   position   3071 ticks -> +1024 from the 2047 zero, a quarter turn
 *   speed    0x8064       -> sign bit 15 set, magnitude 100 ticks/s, so -100
 *   load     0x05f4       -> sign bit 10 set, magnitude 500 per-mille, so -500
 *   voltage  0x78         -> 12.0 V, not decoded (no actuator_feedback field)
 *   temp     0x2d         -> 45 degC
 *
 * The fields are sign-magnitude, NOT two's complement, and the sign bit sits at
 * a different place for speed than for load.
 */
#define PRESENT_BLOCK_BYTES 0xff, 0x0b, 0x64, 0x80, 0xf4, 0x05, 0x78, 0x2d

#define EXPECT_VELOCITY_RADS (-0.15339808f)
#define EXPECT_EFFORT_NM     (-1.471f)

ZTEST(bus_servo_actuator, test_read_feedback_reads_present_block)
{
	struct actuator_feedback fb;
	const uint8_t response[] = {
		0xff, 0xff, 0x02, 0x0a, 0x00, PRESENT_BLOCK_BYTES, 0x67,
	};

	fake_bus_queue_rx(&bus, response, sizeof(response));
	zassert_ok(actuator_read_feedback(PAN, &fb));

	/* One READ of 8 bytes from reg 56, not a read per signal. */
	const uint8_t expected_request[] = {
		0xff, 0xff, 0x02, 0x04, 0x02, 0x38, 0x08, 0xb7,
	};
	zassert_equal(bus.last_tx_len, sizeof(expected_request));
	zassert_mem_equal(bus.last_tx, expected_request, sizeof(expected_request));

	zassert_true((fb.valid_mask & ACTUATOR_FB_POSITION) != 0);
	zassert_true((fb.valid_mask & ACTUATOR_FB_VELOCITY) != 0);
	zassert_true((fb.valid_mask & ACTUATOR_FB_TEMPERATURE) != 0);
	zassert_within(fb.position, PI_F / 2.0f, 0.002f);
	/* Two's-complement decoding would give +33380 ticks/s here, not -100. */
	zassert_within(fb.velocity, EXPECT_VELOCITY_RADS, 0.0001f);
	zassert_within(fb.temperature, 45.0f, 0.0001f);
}

ZTEST(bus_servo_actuator, test_read_feedback_omits_effort_without_stall_torque)
{
	struct actuator_feedback fb;
	const uint8_t response[] = {
		0xff, 0xff, 0x02, 0x0a, 0x00, PRESENT_BLOCK_BYTES, 0x67,
	};

	fake_bus_queue_rx(&bus, response, sizeof(response));
	zassert_ok(actuator_read_feedback(PAN, &fb));

	/* Load is per-mille of stall torque and effort is newton-metres, so with
	 * no stall torque in devicetree the flag must stay clear rather than
	 * publishing a percentage dressed up as a torque. */
	zassert_false((fb.valid_mask & ACTUATOR_FB_EFFORT) != 0);
}

ZTEST(bus_servo_actuator, test_read_feedback_inverts_rate_and_effort)
{
	struct actuator_feedback fb;
	const uint8_t response[] = {
		0xff, 0xff, 0x03, 0x0a, 0x00, PRESENT_BLOCK_BYTES, 0x66,
	};

	fake_bus_queue_rx(&bus, response, sizeof(response));
	zassert_ok(actuator_read_feedback(INVERTED, &fb));

	/* Velocity differentiates position and effort acts along it, so both have
	 * to follow the same sign convention or they disagree with the position
	 * about which way the joint is going. */
	zassert_within(fb.position, -PI_F / 2.0f, 0.002f);
	zassert_within(fb.velocity, -EXPECT_VELOCITY_RADS, 0.0001f);

	zassert_true((fb.valid_mask & ACTUATOR_FB_EFFORT) != 0);
	/* Sign bit 10, not 15: decoding load as bit 15 gives +1524 per-mille. */
	zassert_within(fb.effort, -EXPECT_EFFORT_NM, 0.0001f);
}

ZTEST(bus_servo_actuator, test_read_feedback_preserves_protocol_errno)
{
	struct actuator_feedback fb;
	const uint8_t response[] = {
		0xff, 0xff, 0x02, 0x05, 0x00, 0xff, 0x0b, 0x55, 0x99,
	};

	fake_bus_queue_rx(&bus, response, sizeof(response));

	zassert_equal(actuator_read_feedback(PAN, &fb), -EBADMSG);
}

ZTEST(bus_servo_actuator, test_group_set_position_uses_sync_write)
{
	const float goals[] = {PI_F / 2.0f, -PI_F / 2.0f};

	zassert_ok(actuator_group_enable(&gimbal));
	pan_state_cb_count = 0;
	zassert_ok(actuator_register_state_cb(PAN, on_pan_state, NULL));
	bus.last_tx_len = 0;

	zassert_ok(actuator_group_set_position(&gimbal, goals));

	const uint8_t expected[] = {
		0xff, 0xff, 0xfe, 0x14, 0x83, 0x29, 0x07, 0x02, 0x14, 0xff, 0x0b, 0x00,
		0x00, 0x2c, 0x01, 0x01, 0x0a, 0xff, 0x03, 0x00, 0x00, 0xfa, 0x00, 0xe6,
	};
	zassert_equal(bus.last_tx_len, sizeof(expected));
	zassert_mem_equal(bus.last_tx, expected, sizeof(expected));
	zassert_equal(actuator_get_state(PAN), ACTUATOR_STATE_ACTIVE);
	zassert_equal(actuator_get_state(TILT), ACTUATOR_STATE_ACTIVE);
	zassert_equal(pan_state_cb_count, 1);
	zassert_equal(pan_last_state_seen, ACTUATOR_STATE_ACTIVE);
}

ZTEST(bus_servo_actuator, test_group_set_position_rejects_oversized_group)
{
	const float goals[8] = {0};

	zassert_equal(actuator_group_set_position(&oversized_gimbal, goals), -EMSGSIZE);
}

ZTEST(bus_servo_actuator, test_group_set_position_failure_does_not_promote_state)
{
	const float goals[] = {PI_F / 2.0f, -PI_F / 2.0f};

	zassert_ok(actuator_group_enable(&gimbal));
	zassert_equal(actuator_get_state(PAN), ACTUATOR_STATE_READY);
	zassert_equal(actuator_get_state(TILT), ACTUATOR_STATE_READY);

	zassert_ok(bus_servo_disable(iface));
	zassert_equal(actuator_group_set_position(&gimbal, goals), -EINVAL);
	zassert_equal(actuator_get_state(PAN), ACTUATOR_STATE_READY);
	zassert_equal(actuator_get_state(TILT), ACTUATOR_STATE_READY);
}

ZTEST(bus_servo_actuator, test_velocity_and_effort_are_not_supported)
{
	zassert_equal(actuator_set_velocity(PAN, 1.0f), -ENOTSUP);
	zassert_equal(actuator_set_effort(PAN, 1.0f), -ENOTSUP);
	const float goals[] = {1.0f, 1.0f};

	zassert_equal(actuator_group_set_velocity(&gimbal, goals), -ENOTSUP);
	zassert_equal(actuator_group_set_effort(&gimbal, goals), -ENOTSUP);
}

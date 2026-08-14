/*
 * Decode tests for the bus-servo actuator's present-state block.
 *
 * The block is read in one transaction and its fields are sign-magnitude with
 * the sign bit in a different place for speed (15) than for load (10). Decoding
 * either as two's complement turns a small negative rate into a large positive
 * one, which no build error catches and which looks plausible on a plot.
 */

#include <zephyr/actuator/actuator.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>
#include <drivers/bus_servo.h>

#include "fake_bus.h"

#define BUS_NODE   DT_NODELABEL(bus_servo0)
#define UART_NODE  DT_PARENT(BUS_NODE)
#define IFACE_NAME DEVICE_DT_NAME(BUS_NODE)

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct device *servo = DEVICE_DT_GET(DT_NODELABEL(test_servo));
static const struct device *servo_inv = DEVICE_DT_GET(DT_NODELABEL(test_servo_inverted));
static struct fake_bus bus;
static int iface;

/*
 * position    3071 ticks  -> +1024 from the 2047 zero, a quarter turn
 * speed     0x8064        -> sign bit 15 set, magnitude 100 ticks/s, so -100
 * load      0x05f4        -> sign bit 10 set, magnitude 500 per-mille, so -500
 * voltage   0x78          -> 12.0 V, not decoded (no actuator_feedback field)
 * temp      0x2d          -> 45 degC
 */
static const uint8_t present_block_id1[] = {
	0xff, 0xff, 0x01, 0x0a, 0x00, 0xff, 0x0b, 0x64, 0x80, 0xf4, 0x05, 0x78, 0x2d, 0x68,
};
static const uint8_t present_block_id2[] = {
	0xff, 0xff, 0x02, 0x0a, 0x00, 0xff, 0x0b, 0x64, 0x80, 0xf4, 0x05, 0x78, 0x2d, 0x67,
};

#define EXPECT_POSITION_RAD  1.5707963f
#define EXPECT_VELOCITY_RADS (-0.15339808f)
#define EXPECT_EFFORT_NM     (-1.471f)
#define TOLERANCE            1e-4f

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

ZTEST_SUITE(bus_servo_feedback, NULL, NULL, before_each, after_each, NULL);

ZTEST(bus_servo_feedback, test_reads_whole_block_in_one_transaction)
{
	struct actuator_feedback fb;

	fake_bus_queue_rx(&bus, present_block_id1, sizeof(present_block_id1));
	zassert_ok(actuator_read_feedback(servo, &fb));

	/* One READ of 8 bytes from reg 56, not five single-register reads. */
	const uint8_t expected_request[] = {
		0xff, 0xff, 0x01, 0x04, 0x02, 0x38, 0x08, 0xb8,
	};
	zassert_equal(bus.last_tx_len, sizeof(expected_request));
	zassert_mem_equal(bus.last_tx, expected_request, sizeof(expected_request));
}

ZTEST(bus_servo_feedback, test_decodes_sign_magnitude_fields)
{
	struct actuator_feedback fb;

	fake_bus_queue_rx(&bus, present_block_id1, sizeof(present_block_id1));
	zassert_ok(actuator_read_feedback(servo, &fb));

	zassert_true(fb.valid_mask & ACTUATOR_FB_POSITION);
	zassert_true(fb.valid_mask & ACTUATOR_FB_VELOCITY);
	zassert_true(fb.valid_mask & ACTUATOR_FB_TEMPERATURE);
	zassert_true(fb.valid_mask & ACTUATOR_FB_EFFORT);

	zassert_within(fb.position, EXPECT_POSITION_RAD, TOLERANCE, "position %f", (double)fb.position);
	/* Two's-complement decoding would give +33380 ticks/s here, not -100. */
	zassert_within(fb.velocity, EXPECT_VELOCITY_RADS, TOLERANCE, "velocity %f",
		       (double)fb.velocity);
	/* Sign bit 10, not 15: reading this as bit 15 would give +1524 per-mille. */
	zassert_within(fb.effort, EXPECT_EFFORT_NM, TOLERANCE, "effort %f", (double)fb.effort);
	zassert_within(fb.temperature, 45.0f, TOLERANCE, "temperature %f", (double)fb.temperature);
}

ZTEST(bus_servo_feedback, test_invert_position_also_inverts_velocity)
{
	struct actuator_feedback fb;

	fake_bus_queue_rx(&bus, present_block_id2, sizeof(present_block_id2));
	zassert_ok(actuator_read_feedback(servo_inv, &fb));

	/* Velocity differentiates position, so it has to carry the same sign
	 * convention or the two disagree about which way the joint is going. */
	zassert_within(fb.position, -EXPECT_POSITION_RAD, TOLERANCE, "position %f",
		       (double)fb.position);
	zassert_within(fb.velocity, -EXPECT_VELOCITY_RADS, TOLERANCE, "velocity %f",
		       (double)fb.velocity);
}

ZTEST(bus_servo_feedback, test_effort_unreported_without_stall_torque)
{
	struct actuator_feedback fb;

	fake_bus_queue_rx(&bus, present_block_id2, sizeof(present_block_id2));
	zassert_ok(actuator_read_feedback(servo_inv, &fb));

	/* Load is per-mille of stall torque; with no stall torque configured the
	 * flag must stay clear rather than publishing a percentage as newton-metres. */
	zassert_false(fb.valid_mask & ACTUATOR_FB_EFFORT);
}

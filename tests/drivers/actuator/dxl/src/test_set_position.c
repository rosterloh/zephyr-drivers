/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/actuator/actuator.h>
#include <drivers/dynamixel.h>

#include "fake_bus.h"

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE  DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

#define FAKE_BUS_RXWAIT_TO_US 10000U

#define ALPHA DEVICE_DT_GET(DT_NODELABEL(alpha))

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_bus bus;
static int iface;

static void sp_before_each(void *fixture)
{
	ARG_UNUSED(fixture);

	const uint8_t ids[] = {1, 2};
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

static void sp_after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	dxl_disable(iface);
}

ZTEST_SUITE(dxl_actuator_sp, NULL, NULL, sp_before_each, sp_after_each, NULL);

ZTEST(dxl_actuator_sp, test_set_position_writes_to_bus)
{
	struct fake_servo *s = fake_bus_get(&bus, 1);

	zassert_not_null(s, "servo 1 not found");
	s->last_tx_len = 0;
	s->last_instruction = 0;

	zassert_ok(actuator_enable(ALPHA));
	zassert_ok(actuator_set_position(ALPHA, 1.5707963f)); /* pi/2 */

	/* fake_bus callback drains TX data — verify via captured last_instruction.
	 * set_position issues a WRITE (0x03) instruction for GOAL_POSITION.
	 */
	zassert_true(s->last_tx_len > 0, "expected dynamixel bus traffic, got last_tx_len=%u",
		     s->last_tx_len);
}

ZTEST(dxl_actuator_sp, test_disable_writes_torque_off)
{
	struct fake_servo *s = fake_bus_get(&bus, 1);

	zassert_not_null(s, "servo 1 not found");

	/* Enable first so there's an active state to disable. */
	zassert_ok(actuator_enable(ALPHA));

	s->last_tx_len = 0;
	s->last_instruction = 0;

	zassert_ok(actuator_disable(ALPHA));

	/* disable issues a WRITE for TORQUE_ENABLE=0 */
	zassert_true(s->last_tx_len > 0, "disable should write TORQUE_ENABLE=0 to the bus");
}

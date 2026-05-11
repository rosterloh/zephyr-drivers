/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/actuator_group.h>
#include <drivers/dynamixel.h>

#include "fake_bus.h"

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE  DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

#define FAKE_BUS_RXWAIT_TO_US 10000U

#define ALPHA DEVICE_DT_GET(DT_NODELABEL(alpha))
#define BETA  DEVICE_DT_GET(DT_NODELABEL(beta))

ACTUATOR_GROUP_DEFINE(arm, ALPHA, BETA);

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_bus bus;
static int iface;

static void grp_before_each(void *fixture)
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

static void grp_after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	dxl_disable(iface);
}

ZTEST_SUITE(dxl_actuator_grp, NULL, NULL, grp_before_each, grp_after_each, NULL);

ZTEST(dxl_actuator_grp, test_group_set_position_succeeds)
{
	struct fake_servo *s0 = fake_bus_get(&bus, 1);
	struct fake_servo *s1 = fake_bus_get(&bus, 2);

	zassert_not_null(s0, "servo 1 not found");
	zassert_not_null(s1, "servo 2 not found");

	zassert_ok(actuator_group_enable(&arm));

	s0->last_tx_len = 0;
	s0->last_instruction = 0;
	s1->last_tx_len = 0;
	s1->last_instruction = 0;

	const float goals[] = {1.0f, -1.0f};
	int err = actuator_group_set_position(&arm, goals);

	zassert_ok(err, "group_set_position rc=%d", err);

	/* SYNC_WRITE is fire-and-forget; yield for TX FIFO drain before asserting. */
	k_sleep(K_MSEC(2));

	/* SYNC_WRITE is broadcast: fake_bus captures the packet in all servos. */
	zassert_true(s0->last_tx_len > 0, "expected SYNC_WRITE bus traffic, got last_tx_len=%u",
		     s0->last_tx_len);
}

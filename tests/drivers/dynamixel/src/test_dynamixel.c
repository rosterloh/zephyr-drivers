/*
 * Copyright (c) 2024 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_dynamixel.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dxl_test, LOG_LEVEL_INF);

static uint8_t dxl_iface;

uint8_t test_get_dxl_iface(void)
{
	return dxl_iface;
}

static struct dxl_iface_param dxl_param = {
	.rx_timeout = DXL_TEST_RESPONSE_TO,
	.serial =
		{
			.baud = DXL_TEST_BAUDRATE_LOW,
			.parity = DXL_TEST_PARITY,
		},
};

/*
 * This test performed on hardware requires two UART controllers
 * on the board (with RX/TX lines connected crosswise).
 * The exact mapping is not required, we assume that both controllers
 * have similar capabilities and use the instance with index 0
 * as interface for the client.
 */
#if DT_NODE_EXISTS(DT_INST(0, robotis_dynamixel))
static const char iface_name[] = {DEVICE_DT_NAME(DT_INST(0, robotis_dynamixel))};
#else
static const char iface_name[] = "";
#endif

void test_setup_iface(void)
{
	int err;

	dxl_iface = dxl_iface_get_by_name(iface_name);
	dxl_param.serial.baud = DXL_TEST_BAUDRATE_LOW;
	dxl_param.serial.parity = UART_CFG_PARITY_NONE;

	err = dxl_init(dxl_iface, dxl_param);
	zassert_equal(err, 0, "Failed to configure DYNAMIXEL interface");
}

void test_disable(void)
{
	int err;

	err = dxl_disable(dxl_iface);
	zassert_equal(err, 0, "Failed to disable DYNAMIXEL interface");
}
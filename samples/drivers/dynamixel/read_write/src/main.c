/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <drivers/dynamixel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sample_dynamixel, LOG_LEVEL_INF);

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

#define DEFAULT_MOTOR_ID 1
#define BLINK_COUNT      3
#define BLINK_PERIOD_MS  300

static int read_and_log(int iface, uint8_t id)
{
	uint16_t model;
	uint8_t firmware;
	uint32_t position;
	uint8_t temperature;
	int rc;

	rc = dxl_read_u16(iface, id, MODEL_NUMBER, &model);
	if (rc != 0) {
		LOG_ERR("read MODEL_NUMBER failed: %d", rc);
		return rc;
	}
	LOG_INF("Model number:     %u", model);

	rc = dxl_read_u8(iface, id, FIRMWARE_VERSION, &firmware);
	if (rc != 0) {
		LOG_ERR("read FIRMWARE_VERSION failed: %d", rc);
		return rc;
	}
	LOG_INF("Firmware version: %u", firmware);

	rc = dxl_read_u32(iface, id, PRESENT_POSITION, &position);
	if (rc != 0) {
		LOG_ERR("read PRESENT_POSITION failed: %d", rc);
		return rc;
	}
	LOG_INF("Present position: %u", position);

	rc = dxl_read_u8(iface, id, PRESENT_TEMPERATURE, &temperature);
	if (rc != 0) {
		LOG_ERR("read PRESENT_TEMPERATURE failed: %d", rc);
		return rc;
	}
	LOG_INF("Temperature:      %u C", temperature);

	return 0;
}

static int blink_led(int iface, uint8_t id)
{
	int rc;

	for (int i = 0; i < BLINK_COUNT; i++) {
		rc = dxl_write_u8(iface, id, LED, 1);
		if (rc != 0) {
			LOG_ERR("LED on failed: %d", rc);
			return rc;
		}
		k_sleep(K_MSEC(BLINK_PERIOD_MS));

		rc = dxl_write_u8(iface, id, LED, 0);
		if (rc != 0) {
			LOG_ERR("LED off failed: %d", rc);
			return rc;
		}
		k_sleep(K_MSEC(BLINK_PERIOD_MS));
	}
	return 0;
}

int main(void)
{
	const uint8_t id = DEFAULT_MOTOR_ID;
	int iface;
	int rc;

	struct dxl_iface_param param = {
		.rx_timeout = 50000,
		.serial =
			{
				.baud = 57600,
				.parity = UART_CFG_PARITY_NONE,
			},
	};

	iface = dxl_iface_get_by_name(DXL_IFACE_NAME);
	if (iface < 0) {
		LOG_ERR("Interface '%s' not found: %d", DXL_IFACE_NAME, iface);
		return iface;
	}

	rc = dxl_init(iface, param);
	if (rc != 0) {
		LOG_ERR("dxl_init failed: %d", rc);
		return rc;
	}

	LOG_INF("Pinging servo id=%u", id);
	rc = dxl_ping(iface, id);
	if (rc != 0) {
		LOG_ERR("Ping failed: %d", rc);
		dxl_disable(iface);
		return rc;
	}
	LOG_INF("Ping OK");

	rc = read_and_log(iface, id);
	if (rc == 0) {
		LOG_INF("Blinking LED %d times", BLINK_COUNT);
		rc = blink_led(iface, id);
	}

	dxl_disable(iface);

	if (rc == 0) {
		LOG_INF("Sample done");
	}
	return rc;
}

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dynamixel group-instruction sample.
 *
 * Walks through SYNC_WRITE, BULK_WRITE, SYNC_READ and BULK_READ across all
 * servos discovered in devicetree, in that order. Each phase logs its inputs
 * and outputs so the on-the-wire transactions map clearly back to the API.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <drivers/dynamixel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sample_dynamixel_group, LOG_LEVEL_INF);

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

#define MAX_MOTORS  8
#define HOLD_MS     500
#define SWEEP_LOW   1024
#define SWEEP_HIGH  3072
#define PROFILE_VEL 60

static uint8_t motor_ids[MAX_MOTORS];
static size_t motor_count;

static size_t discover_motors(int iface)
{
	const size_t total = dxl_motor_count();
	size_t found = 0;

	for (size_t i = 0; i < total && found < MAX_MOTORS; i++) {
		const struct dxl_motor *m = dxl_motor_get(i);

		if (m == NULL || m->iface != iface) {
			continue;
		}
		if (dxl_ping(iface, m->id) != 0) {
			LOG_WRN("Servo '%s' (id=%u) did not respond to ping",
				m->label ? m->label : "?", m->id);
			continue;
		}
		motor_ids[found++] = m->id;
	}
	return found;
}

static int demo_sync_write_config(int iface)
{
	uint8_t off[MAX_MOTORS] = {0};
	uint8_t mode[MAX_MOTORS];
	uint8_t on[MAX_MOTORS];
	int rc;

	for (size_t i = 0; i < motor_count; i++) {
		mode[i] = DXL_OP_POSITION;
		on[i] = 1;
	}

	LOG_INF("== SYNC_WRITE: TORQUE_ENABLE=0, OPERATING_MODE=POSITION, "
		"TORQUE_ENABLE=1 across %u servo(s)",
		(unsigned)motor_count);

	rc = dxl_sync_write_u8(iface, TORQUE_ENABLE, motor_ids, off, motor_count);
	if (rc != 0) {
		LOG_ERR("sync_write TORQUE_ENABLE off rc=%d", rc);
		return rc;
	}
	rc = dxl_sync_write_u8(iface, OPERATING_MODE, motor_ids, mode, motor_count);
	if (rc != 0) {
		LOG_ERR("sync_write OPERATING_MODE rc=%d", rc);
		return rc;
	}
	rc = dxl_sync_write_u8(iface, TORQUE_ENABLE, motor_ids, on, motor_count);
	if (rc != 0) {
		LOG_ERR("sync_write TORQUE_ENABLE on rc=%d", rc);
		return rc;
	}
	return 0;
}

static int demo_bulk_write_init(int iface)
{
	struct dxl_bulk_write_entry req[MAX_MOTORS * 2];
	size_t n = 0;

	for (size_t i = 0; i < motor_count && n + 1 < ARRAY_SIZE(req); i++) {
		req[n++] = (struct dxl_bulk_write_entry){
			.id = motor_ids[i],
			.item = PROFILE_VELOCITY,
			.value = PROFILE_VEL,
		};
		req[n++] = (struct dxl_bulk_write_entry){
			.id = motor_ids[i],
			.item = LED,
			.value = 1,
		};
	}

	LOG_INF("== BULK_WRITE: PROFILE_VELOCITY=%u + LED=1, %u entries", PROFILE_VEL, (unsigned)n);

	int rc = dxl_bulk_write(iface, req, n);

	if (rc != 0) {
		LOG_ERR("bulk_write rc=%d", rc);
	}
	return rc;
}

static int demo_sync_write_sweep(int iface, uint32_t target)
{
	uint32_t goals[MAX_MOTORS];

	for (size_t i = 0; i < motor_count; i++) {
		goals[i] = target;
	}

	LOG_INF("-- SYNC_WRITE: GOAL_POSITION=%u to all servos", target);

	int rc = dxl_sync_write_u32(iface, GOAL_POSITION, motor_ids, goals, motor_count);

	if (rc != 0) {
		LOG_ERR("sync_write GOAL_POSITION rc=%d", rc);
	}
	return rc;
}

static int demo_sync_read_position(int iface)
{
	uint32_t pos[MAX_MOTORS];
	int errs[MAX_MOTORS];

	LOG_INF("== SYNC_READ: PRESENT_POSITION across %u servo(s)", (unsigned)motor_count);

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, motor_ids, pos, errs, motor_count);

	for (size_t i = 0; i < motor_count; i++) {
		if (errs[i] == 0) {
			LOG_INF("   servo id=%u position=%u", motor_ids[i], pos[i]);
		} else {
			LOG_WRN("   servo id=%u err=%d", motor_ids[i], errs[i]);
		}
	}
	if (rc != 0 && rc != -EIO) {
		LOG_ERR("sync_read rc=%d", rc);
	}
	return rc;
}

static int demo_bulk_read_status(int iface)
{
	/* Mix register widths in a single transaction: PRESENT_POSITION is u32,
	 * PRESENT_TEMPERATURE is u8. Both come back as uint32_t. */
	struct dxl_bulk_read_entry req[MAX_MOTORS * 2];
	uint32_t vals[MAX_MOTORS * 2];
	int errs[MAX_MOTORS * 2];
	size_t n = 0;

	for (size_t i = 0; i < motor_count && n + 1 < ARRAY_SIZE(req); i++) {
		req[n++] = (struct dxl_bulk_read_entry){
			.id = motor_ids[i],
			.item = PRESENT_POSITION,
		};
		req[n++] = (struct dxl_bulk_read_entry){
			.id = motor_ids[i],
			.item = PRESENT_TEMPERATURE,
		};
	}

	LOG_INF("== BULK_READ: PRESENT_POSITION + PRESENT_TEMPERATURE, %u entries", (unsigned)n);

	int rc = dxl_bulk_read(iface, req, vals, errs, n);

	for (size_t i = 0; i < n; i++) {
		const char *label = (req[i].item == PRESENT_POSITION) ? "position" : "temperature";

		if (errs[i] == 0) {
			LOG_INF("   servo id=%u %s=%u", req[i].id, label, vals[i]);
		} else {
			LOG_WRN("   servo id=%u %s err=%d", req[i].id, label, errs[i]);
		}
	}
	if (rc != 0 && rc != -EIO) {
		LOG_ERR("bulk_read rc=%d", rc);
	}
	return rc;
}

int main(void)
{
	struct dxl_iface_param param = {
		.rx_timeout = 50000,
		.serial =
			{
				.baud = 57600,
				.parity = UART_CFG_PARITY_NONE,
			},
	};
	int iface;
	int rc;

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

	motor_count = discover_motors(iface);
	if (motor_count == 0) {
		LOG_ERR("No reachable servos discovered in devicetree");
		dxl_disable(iface);
		return -ENODEV;
	}
	LOG_INF("Discovered %u servo(s)", (unsigned)motor_count);

	rc = demo_sync_write_config(iface);
	if (rc != 0) {
		goto out;
	}

	rc = demo_bulk_write_init(iface);
	if (rc != 0) {
		goto out;
	}

	rc = demo_sync_write_sweep(iface, SWEEP_LOW);
	if (rc != 0) {
		goto out;
	}
	k_sleep(K_MSEC(HOLD_MS));
	(void)demo_sync_read_position(iface);

	rc = demo_sync_write_sweep(iface, SWEEP_HIGH);
	if (rc != 0) {
		goto out;
	}
	k_sleep(K_MSEC(HOLD_MS));
	(void)demo_sync_read_position(iface);

	(void)demo_bulk_read_status(iface);

	/* Park: torque off and LEDs off in one BULK_WRITE. */
	struct dxl_bulk_write_entry park[MAX_MOTORS * 2];
	size_t n = 0;

	for (size_t i = 0; i < motor_count && n + 1 < ARRAY_SIZE(park); i++) {
		park[n++] = (struct dxl_bulk_write_entry){
			.id = motor_ids[i],
			.item = TORQUE_ENABLE,
			.value = 0,
		};
		park[n++] = (struct dxl_bulk_write_entry){
			.id = motor_ids[i],
			.item = LED,
			.value = 0,
		};
	}
	LOG_INF("== BULK_WRITE: park (TORQUE_ENABLE=0 + LED=0)");
	(void)dxl_bulk_write(iface, park, n);

out:
	dxl_disable(iface);

	if (rc == 0) {
		LOG_INF("Sample done");
	}
	return rc;
}

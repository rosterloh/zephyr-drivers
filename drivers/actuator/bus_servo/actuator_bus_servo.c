/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rosterloh_actuator_bus_servo

#include <errno.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdint.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include <drivers/bus_servo.h>
#include <zephyr/actuator/internal/capabilities.h>

#include "../actuator_internal.h"
#include "../../bus_servo/bus_servo_internal.h"

LOG_MODULE_REGISTER(actuator_bus_servo, CONFIG_ACTUATOR_LOG_LEVEL);

#define BUS_SERVO_CB_POOL                    CONFIG_ACTUATOR_MAX_CALLBACKS_PER_DEVICE
/* Match drivers/bus_servo/bus_servo.c packet layout for SYNC_WRITE position-ex frames. */
#define BUS_SERVO_PACKET_OVERHEAD            6
#define BUS_SERVO_POSITION_EX_LEN            7
#define BUS_SERVO_SYNC_POSITION_EX_ENTRY_LEN (1 + BUS_SERVO_POSITION_EX_LEN)
#define BUS_SERVO_ACTUATOR_SYNC_MAX                                                                \
	((BUS_SERVO_MAX_PACKET_SIZE - BUS_SERVO_PACKET_OVERHEAD - 2) /                             \
	 BUS_SERVO_SYNC_POSITION_EX_ENTRY_LEN)

struct bus_servo_actuator_data {
	struct actuator_common_data common;
	struct actuator_cb_storage cb_storage;
	struct actuator_cb_node cb_pool[BUS_SERVO_CB_POOL];
	const struct device *self;
	int iface;
};

struct bus_servo_actuator_config {
	struct actuator_cb_offsets cb_offsets; /* must be first */
	const char *parent_name;
	uint8_t bus_id;
	uint32_t caps;
	int32_t pos_min_milli;
	int32_t pos_max_milli;
	uint16_t ticks_per_rev;
	uint16_t rad_zero_tick;
	uint16_t speed;
	uint8_t accel;
	bool invert_position;
};

static uint16_t rad_to_ticks(const struct bus_servo_actuator_config *cfg, float rad)
{
	if (cfg->pos_min_milli != INT32_MIN) {
		rad = MAX(rad, (float)cfg->pos_min_milli / 1000.0f);
	}
	if (cfg->pos_max_milli != INT32_MAX) {
		rad = MIN(rad, (float)cfg->pos_max_milli / 1000.0f);
	}
	if (cfg->invert_position) {
		rad = -rad;
	}
	float ticks =
		(float)cfg->rad_zero_tick + rad * (float)cfg->ticks_per_rev / (2.0f * (float)M_PI);
	ticks = CLAMP(ticks, 0.0f, (float)(cfg->ticks_per_rev - 1));
	return (uint16_t)(ticks + 0.5f);
}

static float ticks_to_rad(const struct bus_servo_actuator_config *cfg, uint16_t ticks)
{
	float rad = ((float)ticks - (float)cfg->rad_zero_tick) * (2.0f * (float)M_PI) /
		    (float)cfg->ticks_per_rev;

	return cfg->invert_position ? -rad : rad;
}

static int bus_servo_actuator_enable(const struct device *dev)
{
	const struct bus_servo_actuator_config *cfg = dev->config;
	struct bus_servo_actuator_data *d = dev->data;

	return bus_servo_write_u8(d->iface, cfg->bus_id, BUS_SERVO_REG_TORQUE_ENABLE, 1);
}

static int bus_servo_actuator_disable(const struct device *dev)
{
	const struct bus_servo_actuator_config *cfg = dev->config;
	struct bus_servo_actuator_data *d = dev->data;

	return bus_servo_write_u8(d->iface, cfg->bus_id, BUS_SERVO_REG_TORQUE_ENABLE, 0);
}

static int bus_servo_actuator_clear_fault(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int bus_servo_actuator_set_setpoint(const struct device *dev, enum actuator_mode mode,
					   float value)
{
	const struct bus_servo_actuator_config *cfg = dev->config;
	struct bus_servo_actuator_data *d = dev->data;

	switch (mode) {
	case ACTUATOR_MODE_POSITION:
		return bus_servo_write_position_ex(d->iface, cfg->bus_id, rad_to_ticks(cfg, value),
						   cfg->speed, cfg->accel);
	case ACTUATOR_MODE_VELOCITY:
	case ACTUATOR_MODE_EFFORT:
	default:
		return -ENOTSUP;
	}
}

static int bus_servo_actuator_read_feedback(const struct device *dev, struct actuator_feedback *out)
{
	const struct bus_servo_actuator_config *cfg = dev->config;
	struct bus_servo_actuator_data *d = dev->data;
	uint16_t pos;
	int rc;

	rc = bus_servo_read_u16(d->iface, cfg->bus_id, BUS_SERVO_REG_PRESENT_POSITION_L, &pos);
	if (rc != 0) {
		return rc;
	}

	*out = (struct actuator_feedback){
		.valid_mask = ACTUATOR_FB_POSITION,
		.position = ticks_to_rad(cfg, pos),
		.timestamp_us = (uint64_t)k_uptime_get() * 1000,
	};
	return 0;
}

static int validate_group_member(const struct device *dev, enum actuator_mode mode,
				 enum actuator_state *validated_state)
{
	struct bus_servo_actuator_data *d = dev->data;
	enum actuator_state next;
	int rc;

	rc = actuator_cap_check_mode(d->common.caps, mode);
	if (rc != 0) {
		return rc;
	}

	k_spinlock_key_t key = k_spin_lock(&d->common.lock);
	*validated_state = d->common.state;
	rc = actuator_sm_step(*validated_state, ACTUATOR_SM_EVT_SETPOINT, d->common.caps, &next);
	k_spin_unlock(&d->common.lock, key);

	return rc;
}

static int bus_servo_group_set_setpoints(const struct device *const *devs, size_t n,
					 enum actuator_mode mode, const float *values)
{
	if (n == 0) {
		return 0;
	}
	if (mode != ACTUATOR_MODE_POSITION) {
		return -ENOTSUP;
	}
	if (n > BUS_SERVO_ACTUATOR_SYNC_MAX) {
		return -EMSGSIZE;
	}

	const struct bus_servo_actuator_data *d0 = devs[0]->data;
	uint8_t ids[BUS_SERVO_ACTUATOR_SYNC_MAX];
	uint16_t positions[BUS_SERVO_ACTUATOR_SYNC_MAX];
	uint16_t speeds[BUS_SERVO_ACTUATOR_SYNC_MAX];
	uint8_t accels[BUS_SERVO_ACTUATOR_SYNC_MAX];
	enum actuator_state validated_states[BUS_SERVO_ACTUATOR_SYNC_MAX];
	int rc;

	for (size_t i = 0; i < n; i++) {
		const struct bus_servo_actuator_config *cfg = devs[i]->config;
		const struct bus_servo_actuator_data *d = devs[i]->data;

		if (d->iface != d0->iface) {
			return -ENOTSUP;
		}
		rc = validate_group_member(devs[i], mode, &validated_states[i]);
		if (rc != 0) {
			return rc;
		}
		ids[i] = cfg->bus_id;
		positions[i] = rad_to_ticks(cfg, values[i]);
		speeds[i] = cfg->speed;
		accels[i] = cfg->accel;
	}

	rc = bus_servo_sync_write_position_ex(d0->iface, ids, positions, speeds, accels, n);
	if (rc != 0) {
		return rc;
	}

	for (size_t i = 0; i < n; i++) {
		rc = actuator_report_setpoint_if_state(devs[i], mode, validated_states[i]);
		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

static const struct actuator_driver_api bus_servo_actuator_api = {
	.enable = bus_servo_actuator_enable,
	.disable = bus_servo_actuator_disable,
	.clear_fault = bus_servo_actuator_clear_fault,
	.set_setpoint = bus_servo_actuator_set_setpoint,
	.read_feedback = bus_servo_actuator_read_feedback,
	.group_set_setpoints = bus_servo_group_set_setpoints,
};

static int bus_servo_actuator_init(const struct device *dev)
{
	const struct bus_servo_actuator_config *cfg = dev->config;
	struct bus_servo_actuator_data *d = dev->data;

	d->iface = bus_servo_iface_get_by_name(cfg->parent_name);
	if (d->iface < 0) {
		LOG_ERR("bus-servo iface '%s' not found", cfg->parent_name);
		return -ENODEV;
	}

	d->self = dev;
	d->common.state = ACTUATOR_STATE_DISABLED;
	d->common.caps = cfg->caps;
	d->cb_storage.pool = d->cb_pool;
	d->cb_storage.pool_n = BUS_SERVO_CB_POOL;
	atomic_set(&d->cb_storage.used, 0);
	sys_slist_init(&d->cb_storage.list);

	return 0;
}

#define BUS_SERVO_ACTUATOR_CAPS (ACTUATOR_CAP_POSITION | ACTUATOR_CAP_GROUP_NATIVE)

#define BUS_SERVO_POSITION_MIN_MILLI(inst) DT_INST_PROP_OR(inst, position_min_rad_milli, INT32_MIN)
#define BUS_SERVO_POSITION_MAX_MILLI(inst) DT_INST_PROP_OR(inst, position_max_rad_milli, INT32_MAX)

#define BUS_SERVO_ACTUATOR_DEFINE(inst)                                                            \
	BUILD_ASSERT(DT_INST_PROP(inst, ticks_per_rev) > 0,                                        \
		     "ticks-per-rev must be greater than zero");                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, ticks_per_rev) <= UINT16_MAX,                              \
		     "ticks-per-rev must fit in uint16_t");                                        \
	BUILD_ASSERT(DT_INST_PROP(inst, rad_zero_tick) < DT_INST_PROP(inst, ticks_per_rev),        \
		     "rad-zero-tick must be less than ticks-per-rev");                             \
	BUILD_ASSERT(DT_INST_PROP(inst, rad_zero_tick) <= UINT16_MAX,                              \
		     "rad-zero-tick must fit in uint16_t");                                        \
	BUILD_ASSERT(DT_INST_REG_ADDR(inst) <= UINT8_MAX, "reg must fit in uint8_t");              \
	BUILD_ASSERT(DT_INST_PROP(inst, speed) >= 0, "speed must be non-negative");                \
	BUILD_ASSERT(DT_INST_PROP(inst, speed) <= UINT16_MAX, "speed must fit in uint16_t");       \
	BUILD_ASSERT(DT_INST_PROP(inst, accel) >= 0, "accel must be non-negative");                \
	BUILD_ASSERT(DT_INST_PROP(inst, accel) <= UINT8_MAX, "accel must fit in uint8_t");         \
	BUILD_ASSERT(BUS_SERVO_POSITION_MIN_MILLI(inst) <= BUS_SERVO_POSITION_MAX_MILLI(inst),     \
		     "position-min-rad-milli must be <= position-max-rad-milli");                  \
	static struct bus_servo_actuator_data bus_servo_actuator_data_##inst;                      \
	static const struct bus_servo_actuator_config bus_servo_actuator_config_##inst = {         \
		.cb_offsets =                                                                      \
			{                                                                          \
				.storage_offset =                                                  \
					offsetof(struct bus_servo_actuator_data, cb_storage),      \
			},                                                                         \
		.parent_name = DEVICE_DT_NAME(DT_INST_PARENT(inst)),                               \
		.bus_id = DT_INST_REG_ADDR(inst),                                                  \
		.caps = BUS_SERVO_ACTUATOR_CAPS,                                                   \
		.pos_min_milli = BUS_SERVO_POSITION_MIN_MILLI(inst),                               \
		.pos_max_milli = BUS_SERVO_POSITION_MAX_MILLI(inst),                               \
		.ticks_per_rev = DT_INST_PROP(inst, ticks_per_rev),                                \
		.rad_zero_tick = DT_INST_PROP(inst, rad_zero_tick),                                \
		.speed = DT_INST_PROP(inst, speed),                                                \
		.accel = DT_INST_PROP(inst, accel),                                                \
		.invert_position = DT_INST_PROP(inst, invert_position),                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, bus_servo_actuator_init, NULL,                                 \
			      &bus_servo_actuator_data_##inst, &bus_servo_actuator_config_##inst,  \
			      POST_KERNEL, CONFIG_ACTUATOR_INIT_PRIORITY,                          \
			      &bus_servo_actuator_api);

DT_INST_FOREACH_STATUS_OKAY(BUS_SERVO_ACTUATOR_DEFINE)

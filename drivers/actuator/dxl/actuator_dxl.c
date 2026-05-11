/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rosterloh_actuator_dxl

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>
#include <zephyr/actuator/actuator.h>
#include <drivers/dynamixel.h>

#include "../actuator_internal.h"

LOG_MODULE_REGISTER(actuator_dxl, CONFIG_ACTUATOR_LOG_LEVEL);

#define DXL_CB_POOL CONFIG_ACTUATOR_MAX_CALLBACKS_PER_DEVICE

/* X-series velocity register: ~0.02398 rad/s per tick (0.229 rev/min). */
#define DXL_VEL_TICKS_PER_RAD_S (1.0f / 0.02398f)
/* X-series goal current: 1 tick = 2.69 mA. */
#define DXL_CURRENT_MA_PER_TICK (2.69f)

struct dxl_data {
	struct actuator_common_data common;
	struct actuator_cb_storage cb_storage;
	struct actuator_cb_node cb_pool[DXL_CB_POOL];
	const struct device *self;
	int iface;
	struct k_work_delayable feedback_work; /* initialised in T20 */
};

struct dxl_config {
	struct actuator_cb_offsets cb_offsets; /* must be first */
	const char *parent_name;
	uint8_t bus_id;
	uint32_t caps;
	uint32_t update_period_ms;
	enum actuator_mode default_mode;
	int32_t torque_const_mnm_per_a;
	int32_t pos_min_milli;
	int32_t pos_max_milli;
	int32_t gear_num;
	int32_t gear_den;
	uint16_t ticks_per_rev;
	uint16_t rad_zero_tick;
	int32_t profile_velocity;
};

/* Unit conversions */

static uint32_t rad_to_pos_ticks(const struct dxl_config *cfg, float rad)
{
	float scaled = rad * (float)cfg->gear_num / (float)cfg->gear_den;
	float ticks_per_rad = (float)cfg->ticks_per_rev / (2.0f * (float)M_PI);
	float ticks = (float)cfg->rad_zero_tick + scaled * ticks_per_rad;

	if (ticks < 0.0f) {
		ticks = 0.0f;
	}
	if (ticks > (float)(cfg->ticks_per_rev - 1)) {
		ticks = (float)(cfg->ticks_per_rev - 1);
	}
	return (uint32_t)(ticks + 0.5f);
}

static float pos_ticks_to_rad(const struct dxl_config *cfg, uint32_t ticks)
{
	float ticks_per_rad = (float)cfg->ticks_per_rev / (2.0f * (float)M_PI);
	float rad = ((float)ticks - (float)cfg->rad_zero_tick) / ticks_per_rad;

	return rad * (float)cfg->gear_den / (float)cfg->gear_num;
}

static int32_t rad_s_to_vel_ticks(float rad_s)
{
	float t = rad_s * DXL_VEL_TICKS_PER_RAD_S;

	if (t < INT16_MIN) {
		t = (float)INT16_MIN;
	}
	if (t > INT16_MAX) {
		t = (float)INT16_MAX;
	}
	return (int32_t)t;
}

static int32_t nm_to_current_ticks(const struct dxl_config *cfg, float nm)
{
	if (cfg->torque_const_mnm_per_a == 0) {
		return 0;
	}
	float amps = nm * 1000.0f / (float)cfg->torque_const_mnm_per_a;
	float ticks = amps * 1000.0f / DXL_CURRENT_MA_PER_TICK;

	if (ticks < INT16_MIN) {
		ticks = (float)INT16_MIN;
	}
	if (ticks > INT16_MAX) {
		ticks = (float)INT16_MAX;
	}
	return (int32_t)ticks;
}

/* Driver ops */

static int dxl_actuator_enable(const struct device *dev)
{
	const struct dxl_config *cfg = dev->config;
	struct dxl_data *d = dev->data;
	uint8_t mode_val;
	enum actuator_mode current_mode = d->common.current_mode;

	if (current_mode == ACTUATOR_MODE_DISABLED) {
		current_mode = cfg->default_mode;
	}
	switch (current_mode) {
	case ACTUATOR_MODE_VELOCITY:
		mode_val = DXL_OP_VELOCITY;
		break;
	case ACTUATOR_MODE_EFFORT:
		mode_val = DXL_OP_CURRENT;
		break;
	case ACTUATOR_MODE_POSITION:
	default:
		mode_val = DXL_OP_POSITION;
		break;
	}

	int err = dxl_write_u8(d->iface, cfg->bus_id, TORQUE_ENABLE, 0);

	err |= dxl_write_u8(d->iface, cfg->bus_id, OPERATING_MODE, mode_val);
	err |= dxl_write_u8(d->iface, cfg->bus_id, TORQUE_ENABLE, 1);
	if (err) {
		return -EIO;
	}
	k_work_schedule(&d->feedback_work, K_MSEC(cfg->update_period_ms));
	return 0;
}

static int dxl_actuator_disable(const struct device *dev)
{
	const struct dxl_config *cfg = dev->config;
	struct dxl_data *d = dev->data;

	(void)dxl_write_u8(d->iface, cfg->bus_id, TORQUE_ENABLE, 0);
	k_work_cancel_delayable(&d->feedback_work);
	return 0;
}

static int dxl_actuator_clear_fault(const struct device *dev)
{
	const struct dxl_config *cfg = dev->config;
	struct dxl_data *d = dev->data;

	/* Dynamixel hardware error status is read-only; cleared by reboot. */
	return dxl_reboot(d->iface, cfg->bus_id) == 0 ? 0 : -EIO;
}

static int dxl_actuator_set_setpoint(const struct device *dev, enum actuator_mode mode, float value)
{
	const struct dxl_config *cfg = dev->config;
	struct dxl_data *d = dev->data;

	switch (mode) {
	case ACTUATOR_MODE_POSITION:
		return dxl_write_u32(d->iface, cfg->bus_id, GOAL_POSITION,
				     rad_to_pos_ticks(cfg, value))
			       ? -EIO
			       : 0;
	case ACTUATOR_MODE_VELOCITY:
		return dxl_write_u32(d->iface, cfg->bus_id, GOAL_VELOCITY,
				     (uint32_t)rad_s_to_vel_ticks(value))
			       ? -EIO
			       : 0;
	case ACTUATOR_MODE_EFFORT:
		return dxl_write_u16(d->iface, cfg->bus_id, GOAL_CURRENT,
				     (uint16_t)nm_to_current_ticks(cfg, value))
			       ? -EIO
			       : 0;
	default:
		return -ENOTSUP;
	}
}

static int dxl_actuator_read_feedback(const struct device *dev, struct actuator_feedback *out)
{
	const struct dxl_config *cfg = dev->config;
	struct dxl_data *d = dev->data;
	uint32_t pos = 0, vel = 0;
	uint8_t temp = 0, hw_err = 0;
	int e1 = dxl_read_u32(d->iface, cfg->bus_id, PRESENT_POSITION, &pos);
	int e2 = dxl_read_u32(d->iface, cfg->bus_id, PRESENT_VELOCITY, &vel);
	int e3 = dxl_read_u8(d->iface, cfg->bus_id, PRESENT_TEMPERATURE, &temp);
	int e4 = dxl_read_u8(d->iface, cfg->bus_id, HARDWARE_ERROR_STATUS, &hw_err);

	if (e1 || e2 || e3 || e4) {
		return -EIO;
	}

	*out = (struct actuator_feedback){
		.valid_mask = ACTUATOR_FB_POSITION | ACTUATOR_FB_VELOCITY | ACTUATOR_FB_TEMPERATURE,
		.position = pos_ticks_to_rad(cfg, pos),
		.velocity = ((int32_t)vel) / DXL_VEL_TICKS_PER_RAD_S,
		.temperature = (float)temp,
		.fault_flags = 0,
		.timestamp_us = (uint64_t)k_uptime_get() * 1000,
	};
	if (hw_err & DXL_INPUT_VOLTAGE_ERR) {
		out->fault_flags |= ACTUATOR_FAULT_UNDERVOLTAGE;
	}
	if (hw_err & DXL_OVERHEATING_ERR) {
		out->fault_flags |= ACTUATOR_FAULT_OVERTEMP;
	}
	if (hw_err & DXL_ELECTRICAL_SHOCK_ERR) {
		out->fault_flags |= ACTUATOR_FAULT_DRIVER(0);
	}
	if (hw_err & DXL_OVERLOAD_ERR) {
		out->fault_flags |= ACTUATOR_FAULT_OVERLOAD;
	}
	return 0;
}

static int dxl_group_set_setpoints(const struct device *const *devs, size_t n,
				   enum actuator_mode mode, const float *values)
{
	if (n == 0) {
		return 0;
	}
	const struct dxl_data *d0 = devs[0]->data;
	uint8_t ids[n];
	for (size_t i = 0; i < n; i++) {
		const struct dxl_config *ci = devs[i]->config;
		const struct dxl_data *di = devs[i]->data;
		if (di->iface != d0->iface) {
			/* Cross-iface group — fall back. */
			return -ENOTSUP;
		}
		ids[i] = ci->bus_id;
	}

	switch (mode) {
	case ACTUATOR_MODE_POSITION: {
		uint32_t goals[n];
		for (size_t i = 0; i < n; i++) {
			goals[i] = rad_to_pos_ticks(devs[i]->config, values[i]);
		}
		return dxl_sync_write_u32(d0->iface, GOAL_POSITION, ids, goals, n) ? -EIO : 0;
	}
	case ACTUATOR_MODE_VELOCITY: {
		uint32_t goals[n];
		for (size_t i = 0; i < n; i++) {
			goals[i] = (uint32_t)rad_s_to_vel_ticks(values[i]);
		}
		return dxl_sync_write_u32(d0->iface, GOAL_VELOCITY, ids, goals, n) ? -EIO : 0;
	}
	case ACTUATOR_MODE_EFFORT: {
		uint16_t goals[n];
		for (size_t i = 0; i < n; i++) {
			goals[i] = (uint16_t)nm_to_current_ticks(
				(const struct dxl_config *)devs[i]->config, values[i]);
		}
		return dxl_sync_write_u16(d0->iface, GOAL_CURRENT, ids, goals, n) ? -EIO : 0;
	}
	default:
		return -ENOTSUP;
	}
}

static int dxl_group_read_feedback(const struct device *const *devs, size_t n,
				   struct actuator_feedback *out)
{
	if (n == 0) {
		return 0;
	}
	const struct dxl_data *d0 = devs[0]->data;
	uint8_t ids[n];
	uint32_t positions[n];
	int errs[n];

	for (size_t i = 0; i < n; i++) {
		const struct dxl_config *ci = devs[i]->config;
		const struct dxl_data *di = devs[i]->data;
		if (di->iface != d0->iface) {
			return -ENOTSUP;
		}
		ids[i] = ci->bus_id;
	}
	int rc = dxl_sync_read_u32(d0->iface, PRESENT_POSITION, ids, positions, errs, n);
	for (size_t i = 0; i < n; i++) {
		if (errs[i] != 0) {
			out[i] = (struct actuator_feedback){.valid_mask = 0};
			continue;
		}
		out[i] = (struct actuator_feedback){
			.valid_mask = ACTUATOR_FB_POSITION,
			.position = pos_ticks_to_rad(devs[i]->config, positions[i]),
			.timestamp_us = (uint64_t)k_uptime_get() * 1000,
		};
	}
	return rc;
}

static const struct actuator_driver_api dxl_actuator_api = {
	.enable = dxl_actuator_enable,
	.disable = dxl_actuator_disable,
	.clear_fault = dxl_actuator_clear_fault,
	.set_setpoint = dxl_actuator_set_setpoint,
	.read_feedback = dxl_actuator_read_feedback,
	.group_set_setpoints = dxl_group_set_setpoints,
	.group_read_feedback = dxl_group_read_feedback,
};

static void dxl_feedback_work_handler(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct dxl_data *d = CONTAINER_OF(dw, struct dxl_data, feedback_work);
	const struct device *dev = d->self;
	const struct dxl_config *cfg = dev->config;

	struct actuator_feedback fb;
	if (dxl_actuator_read_feedback(dev, &fb) == 0) {
		actuator_report_feedback(dev, &fb);
	}
	if (d->common.state != ACTUATOR_STATE_DISABLED) {
		k_work_schedule(&d->feedback_work, K_MSEC(cfg->update_period_ms));
	}
}

static int dxl_actuator_init(const struct device *dev)
{
	const struct dxl_config *cfg = dev->config;
	struct dxl_data *d = dev->data;

	d->iface = dxl_iface_get_by_name(cfg->parent_name);
	if (d->iface < 0) {
		LOG_ERR("dynamixel iface '%s' not found", cfg->parent_name);
		return -ENODEV;
	}
	d->self = dev;
	d->common.state = ACTUATOR_STATE_DISABLED;
	d->common.caps = cfg->caps;
	d->cb_storage.pool = d->cb_pool;
	d->cb_storage.pool_n = DXL_CB_POOL;
	atomic_set(&d->cb_storage.used, 0);
	sys_slist_init(&d->cb_storage.list);
	k_work_init_delayable(&d->feedback_work, dxl_feedback_work_handler);

	if (cfg->profile_velocity >= 0) {
		(void)dxl_write_u32(d->iface, cfg->bus_id, PROFILE_VELOCITY,
				    (uint32_t)cfg->profile_velocity);
	}
	return 0;
}

#define DXL_CAPS(inst)                                                                             \
	(ACTUATOR_CAP_POSITION | ACTUATOR_CAP_VELOCITY |                                           \
	 ((DT_INST_PROP_OR(inst, torque_constant_mnm_per_a, 0) > 0) ? ACTUATOR_CAP_EFFORT : 0) |   \
	 ACTUATOR_CAP_GROUP_NATIVE)

#define DXL_DEFAULT_MODE(inst)                                                                     \
	((DT_INST_ENUM_IDX_OR(inst, default_mode, 0) == 1)                                         \
		 ? ACTUATOR_MODE_VELOCITY                                                          \
		 : ((DT_INST_ENUM_IDX_OR(inst, default_mode, 0) == 2) ? ACTUATOR_MODE_EFFORT       \
								      : ACTUATOR_MODE_POSITION))

#define DXL_ACTUATOR_DEFINE(inst)                                                                  \
	static struct dxl_data dxl_data_##inst;                                                    \
	static const struct dxl_config dxl_config_##inst = {                                       \
		.cb_offsets =                                                                      \
			{                                                                          \
				.storage_offset = offsetof(struct dxl_data, cb_storage),           \
			},                                                                         \
		.parent_name = DEVICE_DT_NAME(DT_INST_PARENT(inst)),                               \
		.bus_id = DT_INST_REG_ADDR(inst),                                                  \
		.caps = DXL_CAPS(inst),                                                            \
		.update_period_ms = DT_INST_PROP(inst, update_period_ms),                          \
		.default_mode = DXL_DEFAULT_MODE(inst),                                            \
		.torque_const_mnm_per_a = DT_INST_PROP_OR(inst, torque_constant_mnm_per_a, 0),     \
		.pos_min_milli = DT_INST_PROP_OR(inst, position_min_rad_milli, INT32_MIN),         \
		.pos_max_milli = DT_INST_PROP_OR(inst, position_max_rad_milli, INT32_MAX),         \
		.gear_num = DT_INST_PROP(inst, gear_ratio_num),                                    \
		.gear_den = DT_INST_PROP(inst, gear_ratio_den),                                    \
		.ticks_per_rev = DT_INST_PROP(inst, ticks_per_rev),                                \
		.rad_zero_tick = DT_INST_PROP(inst, rad_zero_tick),                                \
		.profile_velocity = DT_INST_PROP_OR(inst, profile_velocity, -1),                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, dxl_actuator_init, NULL, &dxl_data_##inst, &dxl_config_##inst, \
			      POST_KERNEL, CONFIG_ACTUATOR_INIT_PRIORITY, &dxl_actuator_api);

DT_INST_FOREACH_STATUS_OKAY(DXL_ACTUATOR_DEFINE)

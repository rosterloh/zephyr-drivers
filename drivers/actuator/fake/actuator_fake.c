/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rosterloh_actuator_fake

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>
#include <zephyr/actuator/actuator.h>
#include "../actuator_internal.h"

#define FAKE_CB_POOL CONFIG_ACTUATOR_MAX_CALLBACKS_PER_DEVICE

struct fake_data {
	struct actuator_common_data common;
	struct actuator_cb_storage cb_storage;
	struct actuator_cb_node cb_pool[FAKE_CB_POOL];
	float last_setpoint;
	enum actuator_mode last_mode;
};

struct fake_config {
	struct actuator_cb_offsets cb_offsets; /* must be first */
	uint32_t caps;
};

static int fake_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_clear_fault(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_set_setpoint(const struct device *dev, enum actuator_mode mode, float value)
{
	struct fake_data *d = dev->data;

	d->last_setpoint = value;
	d->last_mode = mode;
	return 0;
}

static int fake_read_feedback(const struct device *dev, struct actuator_feedback *out)
{
	struct fake_data *d = dev->data;

	*out = (struct actuator_feedback){
		.valid_mask = ACTUATOR_FB_POSITION | ACTUATOR_FB_VELOCITY,
		.position = (d->last_mode == ACTUATOR_MODE_POSITION) ? d->last_setpoint : 0.0f,
		.velocity = (d->last_mode == ACTUATOR_MODE_VELOCITY) ? d->last_setpoint : 0.0f,
		.timestamp_us = (uint64_t)k_uptime_get() * 1000,
	};
	return 0;
}

static const struct actuator_driver_api fake_api = {
	.enable = fake_enable,
	.disable = fake_disable,
	.clear_fault = fake_clear_fault,
	.set_setpoint = fake_set_setpoint,
	.read_feedback = fake_read_feedback,
};

static int fake_init(const struct device *dev)
{
	struct fake_data *d = dev->data;
	const struct fake_config *cfg = dev->config;

	d->common.state = ACTUATOR_STATE_DISABLED;
	d->common.caps = cfg->caps;
	d->cb_storage.pool = d->cb_pool;
	d->cb_storage.pool_n = FAKE_CB_POOL;
	atomic_set(&d->cb_storage.used, 0);
	sys_slist_init(&d->cb_storage.list);
	return 0;
}

#define FAKE_DEFINE(inst)                                                                          \
	static struct fake_data fake_data_##inst;                                                  \
	static const struct fake_config fake_config_##inst = {                                     \
		.cb_offsets =                                                                      \
			{                                                                          \
				.storage_offset = offsetof(struct fake_data, cb_storage),          \
			},                                                                         \
		.caps = DT_INST_PROP(inst, capabilities),                                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, fake_init, NULL, &fake_data_##inst, &fake_config_##inst,       \
			      POST_KERNEL, CONFIG_ACTUATOR_INIT_PRIORITY, &fake_api);

DT_INST_FOREACH_STATUS_OKAY(FAKE_DEFINE)

void fake_force_fault(const struct device *dev)
{
	actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, ACTUATOR_FAULT_DRIVER(0));
}

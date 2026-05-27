/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/internal/state_machine.h>
#include <zephyr/actuator/internal/capabilities.h>

#include "../../drivers/actuator/actuator_internal.h"

LOG_MODULE_REGISTER(actuator, CONFIG_ACTUATOR_LOG_LEVEL);

/* Implemented in actuator_callbacks.c. */
void actuator_callbacks_fire_state(const struct device *dev, enum actuator_state new_state);

static struct actuator_common_data *common(const struct device *dev)
{
	/* Backends place struct actuator_common_data as the first field of
	 * their per-device data struct. */
	return (struct actuator_common_data *)dev->data;
}

static const struct actuator_driver_api *api(const struct device *dev)
{
	return (const struct actuator_driver_api *)dev->api;
}

/* Step the state machine under the device lock. The caller MUST hold cd->lock.
 * Returns:
 *   0  no transition (event was a self-loop)
 *   1  transition happened; caller should fire state callbacks AFTER unlocking
 *  <0  rejected; *next is unchanged
 */
static int sm_step_locked(const struct device *dev, enum actuator_sm_event evt,
			  uint32_t fault_flags)
{
	struct actuator_common_data *cd = common(dev);
	enum actuator_state next;
	int err = actuator_sm_step(cd->state, evt, cd->caps, &next);
	if (err != 0) {
		return err;
	}
	if (next == cd->state) {
		return 0;
	}
	cd->state = next;
	if (next == ACTUATOR_STATE_FAULT) {
		cd->cached_fb.fault_flags |= fault_flags;
	} else if (next == ACTUATOR_STATE_DISABLED) {
		cd->cached_fb.fault_flags = 0;
	}
	return 1;
}

void actuator_report_state(const struct device *dev, enum actuator_sm_event event,
			   uint32_t fault_flags)
{
	struct actuator_common_data *cd = common(dev);
	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, event, fault_flags);
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
	if (new_state == ACTUATOR_STATE_FAULT) {
		extern void actuator_group_on_member_fault(const struct device *dev);
		actuator_group_on_member_fault(dev);
	}
}

int z_impl_actuator_enable(const struct device *dev)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_ENABLE, 0);
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc < 0) {
		return rc;
	}
	int err = api(dev)->enable(dev);
	if (err != 0) {
		actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, ACTUATOR_FAULT_DRIVER(0));
		return err;
	}
	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
	return 0;
}

int z_impl_actuator_disable(const struct device *dev)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_DISABLE, 0);
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	int err = api(dev)->disable(dev);
	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
	return err;
}

int z_impl_actuator_clear_fault(const struct device *dev)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_CLEAR_FAULT, 0);
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc == -ENOTSUP) {
		/* Non-latching backend: clear is a no-op success. */
		return 0;
	}
	if (rc < 0) {
		return rc;
	}
	int err = api(dev)->clear_fault ? api(dev)->clear_fault(dev) : 0;
	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
	return err;
}

enum actuator_state z_impl_actuator_get_state(const struct device *dev)
{
	return common(dev)->state;
}

uint32_t z_impl_actuator_get_capabilities(const struct device *dev)
{
	return common(dev)->caps;
}

int z_impl_actuator_set_limits(const struct device *dev, const struct actuator_limits *limits)
{
	if (api(dev)->set_limits != NULL) {
		return api(dev)->set_limits(dev, limits);
	}
	struct actuator_common_data *cd = common(dev);
	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	cd->limits = *limits;
	k_spin_unlock(&cd->lock, key);
	return 0;
}

int z_impl_actuator_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode)
{
	struct actuator_common_data *cd = common(dev);

	if ((cd->caps & ACTUATOR_CAP_DRIVE_MODE) == 0) {
		return -ENOTSUP;
	}
	if (api(dev)->set_drive_mode == NULL) {
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	enum actuator_state s = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (s == ACTUATOR_STATE_DISABLED || s == ACTUATOR_STATE_FAULT) {
		return -EPERM;
	}

	int err = api(dev)->set_drive_mode(dev, mode);
	if (err != 0) {
		actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, ACTUATOR_FAULT_DRIVER(0));
	}
	return err;
}

static int set_setpoint_typed(const struct device *dev, enum actuator_mode mode, float value)
{
	struct actuator_common_data *cd = common(dev);

	int err = actuator_cap_check_mode(cd->caps, mode);
	if (err != 0) {
		return err;
	}

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_SETPOINT, 0);
	if (rc >= 0) {
		cd->current_mode = mode;
	}
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc < 0) {
		return rc;
	}

	err = api(dev)->set_setpoint(dev, mode, value);
	if (err != 0) {
		actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, ACTUATOR_FAULT_DRIVER(0));
		return err;
	}
	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
	return 0;
}

int z_impl_actuator_set_position(const struct device *dev, float rad)
{
	return set_setpoint_typed(dev, ACTUATOR_MODE_POSITION, rad);
}

int z_impl_actuator_set_velocity(const struct device *dev, float rad_s)
{
	return set_setpoint_typed(dev, ACTUATOR_MODE_VELOCITY, rad_s);
}

int z_impl_actuator_set_effort(const struct device *dev, float nm)
{
	return set_setpoint_typed(dev, ACTUATOR_MODE_EFFORT, nm);
}

/* Implemented in actuator_callbacks.c. */
extern void actuator_callbacks_fire_feedback(const struct device *dev,
					     const struct actuator_feedback *fb);

void actuator_report_feedback(const struct device *dev, const struct actuator_feedback *fb)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	cd->cached_fb = *fb;
	k_spin_unlock(&cd->lock, key);

	if (fb->fault_flags != 0) {
		actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, fb->fault_flags);
	}
	actuator_callbacks_fire_feedback(dev, fb);
}

int z_impl_actuator_read_feedback(const struct device *dev, struct actuator_feedback *out)
{
	int err = api(dev)->read_feedback(dev, out);
	if (err == 0) {
		actuator_report_feedback(dev, out);
	}
	return err;
}

int z_impl_actuator_get_feedback(const struct device *dev, struct actuator_feedback *out)
{
	struct actuator_common_data *cd = common(dev);
	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	*out = cd->cached_fb;
	k_spin_unlock(&cd->lock, key);
	return 0;
}

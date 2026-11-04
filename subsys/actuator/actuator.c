/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/internal/state_machine.h>
#include <zephyr/actuator/internal/capabilities.h>

LOG_MODULE_REGISTER(actuator, CONFIG_ACTUATOR_LOG_LEVEL);

/* Implemented in actuator_callbacks.c. */
void actuator_callbacks_fire_state(const struct device *dev, enum actuator_state new_state);
/* Implemented in actuator_group.c. */
void actuator_group_on_member_fault(const struct device *dev);

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
	if (evt == ACTUATOR_SM_EVT_FAULT && next == ACTUATOR_STATE_STOPPED) {
		cd->stopped_faults |= fault_flags;
	}
	/* A fault the stop lands on is outstanding too. The cached flags may have
	 * been overwritten by a later sample, so fall back to a generic bit. */
	if (evt == ACTUATOR_SM_EVT_STOP && cd->state == ACTUATOR_STATE_FAULT) {
		cd->stopped_faults |= cd->cached_fb.fault_flags != 0U ? cd->cached_fb.fault_flags
								      : ACTUATOR_FAULT_DRIVER(0);
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
		actuator_group_on_member_fault(dev);
	}
}

int actuator_report_setpoint_if_state(const struct device *dev, enum actuator_mode mode,
				      enum actuator_state expected_state)
{
	struct actuator_common_data *cd = common(dev);
	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	if (cd->state != expected_state) {
		k_spin_unlock(&cd->lock, key);
		return 0;
	}

	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_SETPOINT, 0);
	if (rc >= 0) {
		cd->current_mode = mode;
	}
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc < 0) {
		return rc;
	}
	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
	return 0;
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
	/* A stop or disable that landed while the op ran did its I/O before the
	 * enable write: redo it, or the drive is left energised under it. */
	key = k_spin_lock(&cd->lock);
	enum actuator_state now = cd->state;
	k_spin_unlock(&cd->lock, key);
	if (now == ACTUATOR_STATE_STOPPED) {
		(void)(api(dev)->stop != NULL ? api(dev)->stop(dev) : api(dev)->disable(dev));
		return -EPERM;
	}
	if (now == ACTUATOR_STATE_DISABLED) {
		(void)api(dev)->disable(dev);
		return -EPERM;
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

int z_impl_actuator_stop_latch(const struct device *dev)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_STOP, 0);
	k_spin_unlock(&cd->lock, key);

	if (rc == 1) {
		actuator_callbacks_fire_state(dev, ACTUATOR_STATE_STOPPED);
	}
	return rc < 0 ? rc : 0;
}

int z_impl_actuator_stop(const struct device *dev)
{
	int rc = z_impl_actuator_stop_latch(dev);

	if (rc != 0) {
		return rc;
	}
	rc = api(dev)->stop != NULL ? api(dev)->stop(dev) : api(dev)->disable(dev);
	/* A stopped member is lost to its group as surely as a faulted one. Run
	 * here, not in the latch: DISABLE_ALL and ESTOP do I/O on the peers. */
	actuator_group_on_member_fault(dev);
	return rc;
}

int z_impl_actuator_clear_stop(const struct device *dev)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	uint32_t faults = cd->stopped_faults;
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_CLEAR_STOP, 0);

	if (rc >= 0) {
		cd->stopped_faults = 0;
		if (faults != 0U) {
			/* The stop masked these; surface them so clear_fault resets
			 * the drive rather than an enable discovering them. */
			(void)sm_step_locked(dev, ACTUATOR_SM_EVT_FAULT, faults);
		}
	}
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc < 0) {
		return rc;
	}
	actuator_callbacks_fire_state(dev, new_state);
	return 0;
}

int actuator_report_reference_begin(const struct device *dev)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_REFERENCE, 0);

	if (rc >= 0) {
		/* Dropped up front: an attempt that does not finish leaves the axis
		 * somewhere the old reference no longer describes. */
		cd->referenced = false;
	}
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc < 0) {
		return rc;
	}
	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
	return 0;
}

void actuator_report_referenced(const struct device *dev)
{
	struct actuator_common_data *cd = common(dev);

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = sm_step_locked(dev, ACTUATOR_SM_EVT_ALIGNED, 0);

	if (rc >= 0) {
		cd->referenced = true;
	}
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc == 1) {
		actuator_callbacks_fire_state(dev, new_state);
	}
}

/* Also behind actuator_group_reference(). */
int actuator_reference_devs(const struct device *const *devs, size_t n)
{
	const struct actuator_driver_api *shared = api(devs[0]);

	for (size_t i = 0; i < n; i++) {
		struct actuator_common_data *cd = common(devs[i]);
		enum actuator_state next;

		if (!(cd->caps & ACTUATOR_CAP_NEEDS_REFERENCE) || api(devs[i])->reference == NULL) {
			return -ENOTSUP;
		}
		/* Refuse before anything moves, not halfway through a set. */
		k_spinlock_key_t key = k_spin_lock(&cd->lock);
		int err = actuator_sm_step(cd->state, ACTUATOR_SM_EVT_REFERENCE, cd->caps, &next);
		k_spin_unlock(&cd->lock, key);
		if (err != 0) {
			return err;
		}
		if (api(devs[i]) != shared) {
			shared = NULL;
		}
	}

	int rc = 0;

	if (shared != NULL) {
		rc = shared->reference(devs, n);
	} else {
		for (size_t i = 0; i < n && rc == 0; i++) {
			rc = api(devs[i])->reference(&devs[i], 1);
		}
	}

	/* Safety net: nothing may be left ALIGNING, which refuses setpoints yet
	 * reads as a run in progress. */
	for (size_t i = 0; i < n; i++) {
		struct actuator_common_data *cd = common(devs[i]);

		k_spinlock_key_t key = k_spin_lock(&cd->lock);
		bool stuck = cd->state == ACTUATOR_STATE_ALIGNING;
		k_spin_unlock(&cd->lock, key);
		if (stuck) {
			actuator_report_state(devs[i], ACTUATOR_SM_EVT_FAULT,
					      ACTUATOR_FAULT_REFERENCE);
			rc = rc != 0 ? rc : -EIO;
		}
	}
	return rc;
}

int z_impl_actuator_reference(const struct device *dev)
{
	return actuator_reference_devs(&dev, 1);
}

bool z_impl_actuator_is_referenced(const struct device *dev)
{
	return common(dev)->referenced;
}

void z_impl_actuator_invalidate_reference(const struct device *dev)
{
	common(dev)->referenced = false;
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
	/* Without a backend op only what this layer can check is accepted:
	 * ramps need the backend's trajectory. */
	if (!isnan(limits->max_accel) || !isnan(limits->max_decel)) {
		return -ENOTSUP;
	}
	bool any = !isnan(limits->min_position) || !isnan(limits->max_position) ||
		   !isnan(limits->max_velocity) || !isnan(limits->max_effort);
	struct actuator_common_data *cd = common(dev);
	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	cd->limits = *limits;
	cd->limits_set = any;
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

	/* STOPPED too: NORMAL re-energises a coasted stage, which would undo a stop
	 * (and on a drive with a queued target, resume it). */
	if (s == ACTUATOR_STATE_DISABLED || s == ACTUATOR_STATE_FAULT ||
	    s == ACTUATOR_STATE_STOPPED) {
		return -EPERM;
	}

	int err = api(dev)->set_drive_mode(dev, mode);
	if (err != 0) {
		actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, ACTUATOR_FAULT_DRIVER(0));
	}
	return err;
}

/* NaN bounds are unset, and every comparison with NaN is false. */
static bool outside_limits(const struct actuator_limits *l, enum actuator_mode mode, float value)
{
	switch (mode) {
	case ACTUATOR_MODE_POSITION:
		return value < l->min_position || value > l->max_position;
	case ACTUATOR_MODE_VELOCITY:
		return fabsf(value) > l->max_velocity;
	case ACTUATOR_MODE_EFFORT:
		return fabsf(value) > l->max_effort;
	default:
		return false;
	}
}

/* The reference and limit gates. The caller holds cd->lock. */
static int setpoint_refusal_locked(const struct actuator_common_data *cd, enum actuator_mode mode,
				   float value)
{
	if (mode == ACTUATOR_MODE_POSITION && (cd->caps & ACTUATOR_CAP_NEEDS_REFERENCE) &&
	    !cd->referenced) {
		return -EACCES;
	}
	if (cd->limits_set && outside_limits(&cd->limits, mode, value)) {
		return -ERANGE;
	}
	return 0;
}

/* Dry run of every per-device setpoint gate; a native group op bypasses them.
 * Also behind actuator_group_set_*(). */
int actuator_setpoint_check(const struct device *dev, enum actuator_mode mode, float value)
{
	struct actuator_common_data *cd = common(dev);

	if (mode == ACTUATOR_MODE_DUTY && isnan(value)) {
		return -EINVAL;
	}
	int rc = actuator_cap_check_mode(cd->caps, mode);

	if (rc != 0) {
		return rc;
	}
	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	enum actuator_state next;

	rc = setpoint_refusal_locked(cd, mode, value);
	if (rc == 0) {
		rc = actuator_sm_step(cd->state, ACTUATOR_SM_EVT_SETPOINT, cd->caps, &next);
	}
	k_spin_unlock(&cd->lock, key);
	return rc;
}

static int set_setpoint_typed(const struct device *dev, enum actuator_mode mode, float value)
{
	struct actuator_common_data *cd = common(dev);

	int err = actuator_cap_check_mode(cd->caps, mode);
	if (err != 0) {
		return err;
	}
	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	int rc = setpoint_refusal_locked(cd, mode, value);

	if (rc == 0) {
		rc = sm_step_locked(dev, ACTUATOR_SM_EVT_SETPOINT, 0);
	}
	if (rc >= 0) {
		cd->current_mode = mode;
	}
	enum actuator_state new_state = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (rc < 0) {
		return rc;
	}

	if ((cd->caps & ACTUATOR_CAP_DRIVE_MODE) && api(dev)->set_drive_mode != NULL) {
		err = api(dev)->set_drive_mode(dev, ACTUATOR_DRIVE_MODE_NORMAL);
		if (err != 0) {
			actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, ACTUATOR_FAULT_DRIVER(0));
			return err;
		}
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

int z_impl_actuator_set_duty(const struct device *dev, float duty)
{
	if (isnan(duty)) {
		return -EINVAL;
	}
	return set_setpoint_typed(dev, ACTUATOR_MODE_DUTY, duty);
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

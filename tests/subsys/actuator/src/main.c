/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/actuator_group.h>

#define FAKE0     DEVICE_DT_GET(DT_NODELABEL(fake0))
#define FAKE1     DEVICE_DT_GET(DT_NODELABEL(fake1))
#define FAKE_NODM DEVICE_DT_GET(DT_NODELABEL(fake_nodm))
#define FAKE_REF  DEVICE_DT_GET(DT_NODELABEL(fake_ref))

ACTUATOR_GROUP_DEFINE(arm, FAKE0, FAKE1);
/* Its own group: an ESTOP policy latches the group for good. */
ACTUATOR_GROUP_DEFINE(stop_pair, FAKE1, FAKE_NODM);
ACTUATOR_GROUP_DEFINE(ref_grp, FAKE_REF);

extern void fake_force_fault(const struct device *dev);
extern int fake_stop_calls(const struct device *dev);
extern void fake_set_reference_rc(const struct device *dev, int rc);
extern bool fake_is_energised(const struct device *dev);
extern void fake_stop_during_enable(const struct device *dev);
extern int fake_group_set_calls(const struct device *dev);

static void before_each(void *data)
{
	ARG_UNUSED(data);
	/* Ensure each test starts from a known DISABLED state. A stop survives
	 * a disable, and its clear may reveal a fault, so clear it first. */
	const struct device *const devs[] = {FAKE0, FAKE1, FAKE_NODM, FAKE_REF};

	for (size_t i = 0; i < ARRAY_SIZE(devs); i++) {
		(void)actuator_clear_stop(devs[i]);
		(void)actuator_set_limits(devs[i], &(struct actuator_limits)ACTUATOR_LIMITS_NONE);
		actuator_disable(devs[i]);
	}
	fake_set_reference_rc(FAKE_REF, 0);
	actuator_invalidate_reference(FAKE_REF);
}

ZTEST_SUITE(actuator_subsys, NULL, NULL, before_each, NULL, NULL);

ZTEST(actuator_subsys, test_starts_disabled)
{
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_DISABLED);
}

ZTEST(actuator_subsys, test_enable_disable_round_trip)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_READY);
	zassert_ok(actuator_disable(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_DISABLED);
}

ZTEST(actuator_subsys, test_set_position_after_enable_is_active)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_position(FAKE0, 1.57f));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_ACTIVE);
}

ZTEST(actuator_subsys, test_setpoint_in_disabled_refused)
{
	zassert_equal(actuator_set_position(FAKE0, 1.57f), -EPERM);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_DISABLED);
}

static volatile int state_cb_count;
static enum actuator_state last_state_seen;

static void on_state(const struct device *dev, enum actuator_state s, void *ud)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ud);
	state_cb_count++;
	last_state_seen = s;
}

ZTEST(actuator_subsys, test_state_callback_fires_on_transition)
{
	state_cb_count = 0;
	zassert_ok(actuator_register_state_cb(FAKE0, on_state, NULL));

	zassert_ok(actuator_disable(FAKE0));
	zassert_ok(actuator_enable(FAKE0));
	zassert_true(state_cb_count >= 1);
	zassert_equal(last_state_seen, ACTUATOR_STATE_READY);
}

ZTEST(actuator_subsys, test_get_feedback_returns_cached)
{
	struct actuator_feedback fb = {0};

	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_position(FAKE0, 0.5f));
	zassert_ok(actuator_read_feedback(FAKE0, &fb));
	zassert_within(fb.position, 0.5f, 1e-6f);

	struct actuator_feedback cached = {0};

	zassert_ok(actuator_get_feedback(FAKE0, &cached));
	zassert_within(cached.position, 0.5f, 1e-6f);
}

ZTEST(actuator_subsys, test_group_disable_all_on_fault)
{
	zassert_ok(actuator_disable(FAKE0));
	zassert_ok(actuator_disable(FAKE1));
	zassert_ok(actuator_group_set_fault_policy(&arm, ACTUATOR_GROUP_POLICY_DISABLE_ALL));
	zassert_ok(actuator_group_enable(&arm));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_READY);
	zassert_equal(actuator_get_state(FAKE1), ACTUATOR_STATE_READY);

	fake_force_fault(FAKE0);

	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_FAULT);
	zassert_equal(actuator_get_state(FAKE1), ACTUATOR_STATE_DISABLED);
}

ZTEST(actuator_subsys, test_set_drive_mode_rejected_without_cap)
{
	zassert_ok(actuator_enable(FAKE_NODM));
	zassert_equal(actuator_set_drive_mode(FAKE_NODM, ACTUATOR_DRIVE_MODE_BRAKE), -ENOTSUP);
}

extern enum actuator_drive_mode fake_get_drive_mode(const struct device *dev);

ZTEST(actuator_subsys, test_set_drive_mode_brake_in_ready)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_READY,
		      "drive mode must not change actuator state");
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_BRAKE);
}

ZTEST(actuator_subsys, test_set_drive_mode_coast_in_active)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_position(FAKE0, 1.0f));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_ACTIVE);
	zassert_ok(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_COAST));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_ACTIVE,
		      "drive mode must not change actuator state");
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_COAST);
}

ZTEST(actuator_subsys, test_set_drive_mode_rejected_when_disabled)
{
	zassert_ok(actuator_disable(FAKE0));
	zassert_equal(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE), -EPERM);
}

ZTEST(actuator_subsys, test_set_drive_mode_rejected_when_faulted)
{
	zassert_ok(actuator_enable(FAKE0));
	fake_force_fault(FAKE0);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_FAULT);
	zassert_equal(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE), -EPERM);
}

ZTEST(actuator_subsys, test_setpoint_auto_clears_drive_mode)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE));
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_BRAKE);

	zassert_ok(actuator_set_position(FAKE0, 0.5f));
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_NORMAL,
		      "setpoint must implicitly return to NORMAL");
}

ZTEST(actuator_subsys, test_set_duty_rejects_nan_and_missing_cap)
{
	zassert_equal(actuator_set_duty(FAKE0, NAN), -EINVAL);
	/* fake0 does not advertise ACTUATOR_CAP_DUTY */
	zassert_equal(actuator_set_duty(FAKE0, 0.5f), -ENOTSUP);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_DISABLED);
}

ZTEST(actuator_subsys, test_stop_latches_and_calls_backend)
{
	int before = fake_stop_calls(FAKE0);

	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
	zassert_equal(fake_stop_calls(FAKE0), before + 1);
	/* Idempotent: a repeat retries the backend stop. */
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(fake_stop_calls(FAKE0), before + 2);
}

ZTEST(actuator_subsys, test_stop_latch_does_no_io)
{
	int before = fake_stop_calls(FAKE0);

	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_stop_latch(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
	zassert_equal(fake_stop_calls(FAKE0), before);
}

ZTEST(actuator_subsys, test_stopped_refuses_enable_and_setpoint)
{
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(actuator_enable(FAKE0), -EPERM);
	zassert_equal(actuator_set_position(FAKE0, 0.1f), -EPERM);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
}

ZTEST(actuator_subsys, test_disable_keeps_stop)
{
	zassert_ok(actuator_stop(FAKE0));
	zassert_ok(actuator_disable(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
}

ZTEST(actuator_subsys, test_fault_while_stopped_then_clear_stop_is_fault)
{
	struct actuator_feedback fb;

	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_stop(FAKE0));
	fake_force_fault(FAKE0);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
	zassert_ok(actuator_clear_stop(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_FAULT);
	zassert_ok(actuator_get_feedback(FAKE0, &fb));
	zassert_true(fb.fault_flags & ACTUATOR_FAULT_DRIVER(0));
}

ZTEST(actuator_subsys, test_clear_stop_to_disabled)
{
	zassert_ok(actuator_stop(FAKE0));
	zassert_ok(actuator_clear_stop(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_DISABLED);
	zassert_equal(actuator_clear_stop(FAKE0), -EINVAL);
}

ZTEST(actuator_subsys, test_stop_fires_state_callback)
{
	state_cb_count = 0;
	zassert_ok(actuator_register_state_cb(FAKE0, on_state, NULL));
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(last_state_seen, ACTUATOR_STATE_STOPPED);
}

ZTEST(actuator_subsys, test_stop_runs_group_policy)
{
	const float pos[] = {0.1f, 0.1f};

	zassert_ok(actuator_group_set_fault_policy(&stop_pair, ACTUATOR_GROUP_POLICY_ESTOP));
	zassert_ok(actuator_group_enable(&stop_pair));
	zassert_ok(actuator_stop(FAKE1));
	zassert_equal(actuator_get_state(FAKE_NODM), ACTUATOR_STATE_DISABLED);
	zassert_equal(actuator_group_set_position(&stop_pair, pos), -EPERM);
}

ZTEST(actuator_subsys, test_reference_needs_cap)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_equal(actuator_reference(FAKE0), -ENOTSUP);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_READY);
}

ZTEST(actuator_subsys, test_reference_needs_enabled)
{
	zassert_equal(actuator_reference(FAKE_REF), -EPERM);
	zassert_equal(actuator_get_state(FAKE_REF), ACTUATOR_STATE_DISABLED);
	zassert_false(actuator_is_referenced(FAKE_REF));
}

ZTEST(actuator_subsys, test_position_refused_until_referenced)
{
	zassert_ok(actuator_enable(FAKE_REF));
	zassert_equal(actuator_set_position(FAKE_REF, 0.2f), -EACCES);
	zassert_ok(actuator_reference(FAKE_REF));
	zassert_equal(actuator_get_state(FAKE_REF), ACTUATOR_STATE_READY);
	zassert_true(actuator_is_referenced(FAKE_REF));
	zassert_ok(actuator_set_position(FAKE_REF, 0.2f));
}

ZTEST(actuator_subsys, test_reference_failure_faults)
{
	struct actuator_feedback fb;

	zassert_ok(actuator_enable(FAKE_REF));
	fake_set_reference_rc(FAKE_REF, -EIO);
	zassert_equal(actuator_reference(FAKE_REF), -EIO);
	zassert_equal(actuator_get_state(FAKE_REF), ACTUATOR_STATE_FAULT);
	zassert_ok(actuator_get_feedback(FAKE_REF, &fb));
	zassert_true(fb.fault_flags & ACTUATOR_FAULT_REFERENCE);
	zassert_false(actuator_is_referenced(FAKE_REF));
}

ZTEST(actuator_subsys, test_invalidate_reference_regates)
{
	zassert_ok(actuator_enable(FAKE_REF));
	zassert_ok(actuator_reference(FAKE_REF));
	actuator_invalidate_reference(FAKE_REF);
	zassert_false(actuator_is_referenced(FAKE_REF));
	zassert_equal(actuator_set_position(FAKE_REF, 0.2f), -EACCES);
}

ZTEST(actuator_subsys, test_reference_op_silent_failure_not_left_aligning)
{
	/* The fake begins, then returns success without finishing. */
	zassert_ok(actuator_enable(FAKE_REF));
	fake_set_reference_rc(FAKE_REF, 1);
	zassert_equal(actuator_reference(FAKE_REF), -EIO);
	zassert_equal(actuator_get_state(FAKE_REF), ACTUATOR_STATE_FAULT);
}

ZTEST(actuator_subsys, test_group_position_refused_until_referenced)
{
	const float pos[] = {0.3f};

	zassert_ok(actuator_group_enable(&ref_grp));
	zassert_equal(actuator_group_set_position(&ref_grp, pos), -EACCES);
	zassert_ok(actuator_group_reference(&ref_grp));
	zassert_true(actuator_is_referenced(FAKE_REF));
	zassert_ok(actuator_group_set_position(&ref_grp, pos));
}

ZTEST(actuator_subsys, test_stop_during_reference_wins)
{
	zassert_ok(actuator_enable(FAKE_REF));
	zassert_ok(actuator_report_reference_begin(FAKE_REF));
	zassert_ok(actuator_stop(FAKE_REF));
	actuator_report_referenced(FAKE_REF);
	zassert_equal(actuator_get_state(FAKE_REF), ACTUATOR_STATE_STOPPED);
	zassert_false(actuator_is_referenced(FAKE_REF));
}

ZTEST(actuator_subsys, test_no_limits_by_default)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_position(FAKE0, 100.0f));
}

ZTEST(actuator_subsys, test_limits_enforced_without_backend_op)
{
	struct actuator_limits lim = ACTUATOR_LIMITS_NONE;

	lim.min_position = -1.0f;
	lim.max_position = 1.0f;
	zassert_ok(actuator_set_limits(FAKE0, &lim));
	zassert_ok(actuator_enable(FAKE0));
	zassert_equal(actuator_set_position(FAKE0, 2.0f), -ERANGE);
	zassert_equal(actuator_set_position(FAKE0, -1.5f), -ERANGE);
	zassert_ok(actuator_set_position(FAKE0, 0.5f));
}

ZTEST(actuator_subsys, test_velocity_limit_enforced)
{
	struct actuator_limits lim = ACTUATOR_LIMITS_NONE;

	lim.max_velocity = 1.0f;
	zassert_ok(actuator_set_limits(FAKE0, &lim));
	zassert_ok(actuator_enable(FAKE0));
	zassert_equal(actuator_set_velocity(FAKE0, -2.0f), -ERANGE);
	zassert_ok(actuator_set_velocity(FAKE0, 0.5f));
}

ZTEST(actuator_subsys, test_limits_none_clears)
{
	struct actuator_limits lim = ACTUATOR_LIMITS_NONE;

	lim.max_position = 1.0f;
	zassert_ok(actuator_set_limits(FAKE0, &lim));
	zassert_ok(actuator_set_limits(FAKE0, &(struct actuator_limits)ACTUATOR_LIMITS_NONE));
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_position(FAKE0, 5.0f));
}

ZTEST(actuator_subsys, test_accel_limit_unsupported_without_op)
{
	struct actuator_limits lim = ACTUATOR_LIMITS_NONE;

	lim.max_position = 1.0f;
	lim.max_accel = 10.0f;
	zassert_equal(actuator_set_limits(FAKE0, &lim), -ENOTSUP);
	/* Nothing stored: the position bound did not take either. */
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_position(FAKE0, 5.0f));
}

ZTEST(actuator_subsys, test_fault_then_stop_then_clear_stop_is_fault)
{
	/* A drive that faulted and was then stopped must come back as FAULT, or
	 * clear_fault (which resets the drive's alarm) is skipped and every enable
	 * after it is refused by the still-alarmed drive. */
	zassert_ok(actuator_enable(FAKE0));
	fake_force_fault(FAKE0);
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
	zassert_ok(actuator_clear_stop(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_FAULT);
	/* FAKE0 is not fault-latching: a disable is what leaves FAULT. */
	zassert_ok(actuator_disable(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_DISABLED);
}

ZTEST(actuator_subsys, test_set_drive_mode_rejected_when_stopped)
{
	/* NORMAL re-energises a coasted stage: a stop must not be undone by it. */
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_NORMAL), -EPERM);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
}

ZTEST(actuator_subsys, test_stop_during_enable_leaves_drive_stopped)
{
	/* The stop's I/O ran before the enable's write: the drive must not be
	 * left energised under a STOPPED state. */
	fake_stop_during_enable(FAKE0);
	zassert_equal(actuator_enable(FAKE0), -EPERM);
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
	zassert_false(fake_is_energised(FAKE0));
}

ZTEST(actuator_subsys, test_native_group_write_refused_for_stopped_member)
{
	const float pos[] = {0.1f, 0.1f};
	int before = fake_group_set_calls(FAKE0);

	zassert_ok(actuator_group_set_fault_policy(&arm, ACTUATOR_GROUP_POLICY_ISOLATE));
	zassert_ok(actuator_group_enable(&arm));
	zassert_ok(actuator_stop_latch(FAKE1));
	zassert_equal(actuator_group_set_position(&arm, pos), -EPERM);
	zassert_equal(fake_group_set_calls(FAKE0), before);
}

ZTEST(actuator_subsys, test_native_group_write_enforces_limits)
{
	struct actuator_limits lim = ACTUATOR_LIMITS_NONE;
	const float pos[] = {2.0f, 0.0f};
	int before = fake_group_set_calls(FAKE0);

	lim.max_position = 1.0f;
	zassert_ok(actuator_set_limits(FAKE0, &lim));
	zassert_ok(actuator_group_set_fault_policy(&arm, ACTUATOR_GROUP_POLICY_ISOLATE));
	zassert_ok(actuator_group_enable(&arm));
	zassert_equal(actuator_group_set_position(&arm, pos), -ERANGE);
	zassert_equal(fake_group_set_calls(FAKE0), before);
}

ZTEST(actuator_subsys, test_stop_latch_defers_group_policy_io)
{
	zassert_ok(actuator_group_set_fault_policy(&arm, ACTUATOR_GROUP_POLICY_DISABLE_ALL));
	zassert_ok(actuator_group_enable(&arm));
	zassert_ok(actuator_stop_latch(FAKE0));
	/* No I/O on a peer from the latch... */
	zassert_equal(actuator_get_state(FAKE1), ACTUATOR_STATE_READY);
	zassert_true(fake_is_energised(FAKE1));
	/* ...the policy runs with the stop's I/O. */
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(actuator_get_state(FAKE1), ACTUATOR_STATE_DISABLED);
	zassert_false(fake_is_energised(FAKE1));
}

#ifdef CONFIG_USERSPACE
#include <zephyr/ztest_error_hook.h>

static void *user_setup(void)
{
	k_object_access_all_grant(FAKE0);
	k_object_access_all_grant(FAKE_REF);
	return NULL;
}

ZTEST_SUITE(actuator_user, NULL, user_setup, NULL, NULL, NULL);

ZTEST_USER(actuator_user, test_every_syscall_from_user_mode)
{
	struct actuator_limits lim = ACTUATOR_LIMITS_NONE;
	struct actuator_feedback fb;

	zassert_ok(actuator_set_limits(FAKE0, &lim));
	zassert_ok(actuator_enable(FAKE0));
	zassert_equal(actuator_get_capabilities(FAKE0) & ACTUATOR_CAP_POSITION,
		      ACTUATOR_CAP_POSITION);
	zassert_ok(actuator_set_position(FAKE0, 0.25f));
	zassert_ok(actuator_read_feedback(FAKE0, &fb));
	zassert_within(fb.position, 0.25f, 1e-6f);
	zassert_ok(actuator_get_feedback(FAKE0, &fb));
	zassert_ok(actuator_set_velocity(FAKE0, 0.1f));
	zassert_ok(actuator_set_effort(FAKE0, 0.1f));
	zassert_equal(actuator_set_duty(FAKE0, 0.1f), -ENOTSUP);
	zassert_ok(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE));
	zassert_ok(actuator_stop_latch(FAKE0));
	zassert_ok(actuator_stop(FAKE0));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_STOPPED);
	zassert_ok(actuator_clear_stop(FAKE0));
	zassert_equal(actuator_clear_fault(FAKE0), -EINVAL); /* not faulted */
	zassert_ok(actuator_disable(FAKE0));

	zassert_ok(actuator_enable(FAKE_REF));
	zassert_ok(actuator_reference(FAKE_REF));
	zassert_true(actuator_is_referenced(FAKE_REF));
	actuator_invalidate_reference(FAKE_REF);
	zassert_false(actuator_is_referenced(FAKE_REF));
	zassert_ok(actuator_disable(FAKE_REF));
}

ZTEST_USER(actuator_user, test_user_bad_pointer_is_refused)
{
	ztest_set_fault_valid(true);
	(void)actuator_get_feedback(FAKE0, (struct actuator_feedback *)FAKE0);
	ztest_test_fail();
}
#endif

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

ACTUATOR_GROUP_DEFINE(arm, FAKE0, FAKE1);
/* Its own group: an ESTOP policy latches the group for good. */
ACTUATOR_GROUP_DEFINE(stop_pair, FAKE1, FAKE_NODM);

extern void fake_force_fault(const struct device *dev);
extern int fake_stop_calls(const struct device *dev);

static void before_each(void *data)
{
	ARG_UNUSED(data);
	/* Ensure each test starts from a known DISABLED state. A stop survives
	 * a disable, and its clear may reveal a fault, so clear it first. */
	const struct device *const devs[] = {FAKE0, FAKE1, FAKE_NODM};

	for (size_t i = 0; i < ARRAY_SIZE(devs); i++) {
		(void)actuator_clear_stop(devs[i]);
		actuator_disable(devs[i]);
	}
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

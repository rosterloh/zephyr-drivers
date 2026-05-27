/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/actuator_group.h>

#define FAKE0 DEVICE_DT_GET(DT_NODELABEL(fake0))
#define FAKE1 DEVICE_DT_GET(DT_NODELABEL(fake1))

ACTUATOR_GROUP_DEFINE(arm, FAKE0, FAKE1);

extern void fake_force_fault(const struct device *dev);

static void before_each(void *data)
{
	ARG_UNUSED(data);
	/* Ensure each test starts from a known DISABLED state. */
	actuator_disable(FAKE0);
	actuator_disable(FAKE1);
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

ZTEST(actuator_subsys, test_set_position_promotes_to_active)
{
	zassert_ok(actuator_disable(FAKE0));
	zassert_ok(actuator_set_position(FAKE0, 1.57f));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_ACTIVE);
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
	/* fake0 currently has caps = 7 (POSITION|VELOCITY|EFFORT), no DRIVE_MODE. */
	zassert_ok(actuator_enable(FAKE0));
	zassert_equal(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE), -ENOTSUP);
}

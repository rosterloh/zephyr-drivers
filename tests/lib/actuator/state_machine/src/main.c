/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/actuator/actuator_types.h>
#include <zephyr/actuator/internal/state_machine.h>

ZTEST_SUITE(actuator_state_machine, NULL, NULL, NULL, NULL, NULL);

ZTEST(actuator_state_machine, test_disabled_to_ready_on_enable)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_DISABLED, ACTUATOR_SM_EVT_ENABLE,
				   /* caps */ ACTUATOR_CAP_POSITION, &next);
	zassert_equal(err, 0);
	zassert_equal(next, ACTUATOR_STATE_READY);
}

ZTEST(actuator_state_machine, test_disabled_to_aligning_when_needs_align)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_DISABLED, ACTUATOR_SM_EVT_ENABLE,
				   ACTUATOR_CAP_POSITION | ACTUATOR_CAP_NEEDS_ALIGN, &next);
	zassert_equal(err, 0);
	zassert_equal(next, ACTUATOR_STATE_ALIGNING);
}

ZTEST(actuator_state_machine, test_ready_to_active_on_setpoint)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_READY, ACTUATOR_SM_EVT_SETPOINT,
				   ACTUATOR_CAP_POSITION, &next);
	zassert_equal(err, 0);
	zassert_equal(next, ACTUATOR_STATE_ACTIVE);
}

ZTEST(actuator_state_machine, test_aligning_rejects_setpoint)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_ALIGNING, ACTUATOR_SM_EVT_SETPOINT,
				   ACTUATOR_CAP_POSITION | ACTUATOR_CAP_NEEDS_ALIGN, &next);
	zassert_equal(err, -EAGAIN);
}

ZTEST(actuator_state_machine, test_fault_blocks_setpoint)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_FAULT, ACTUATOR_SM_EVT_SETPOINT,
				   ACTUATOR_CAP_POSITION, &next);
	zassert_equal(err, -EPERM);
}

ZTEST(actuator_state_machine, test_fault_clears_when_latching)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_FAULT, ACTUATOR_SM_EVT_CLEAR_FAULT,
				   ACTUATOR_CAP_POSITION | ACTUATOR_CAP_FAULT_LATCHING, &next);
	zassert_equal(err, 0);
	zassert_equal(next, ACTUATOR_STATE_DISABLED);
}

ZTEST(actuator_state_machine, test_fault_event_from_any_state)
{
	enum actuator_state next;
	const enum actuator_state from[] = {
		ACTUATOR_STATE_DISABLED,
		ACTUATOR_STATE_READY,
		ACTUATOR_STATE_ALIGNING,
		ACTUATOR_STATE_ACTIVE,
	};
	for (size_t i = 0; i < ARRAY_SIZE(from); i++) {
		int err = actuator_sm_step(from[i], ACTUATOR_SM_EVT_FAULT, 0, &next);
		zassert_equal(err, 0, "from=%d", from[i]);
		zassert_equal(next, ACTUATOR_STATE_FAULT, "from=%d", from[i]);
	}
}

ZTEST(actuator_state_machine, test_disable_from_anywhere_goes_disabled)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_ACTIVE, ACTUATOR_SM_EVT_DISABLE, 0, &next);
	zassert_equal(err, 0);
	zassert_equal(next, ACTUATOR_STATE_DISABLED);

	err = actuator_sm_step(ACTUATOR_STATE_FAULT, ACTUATOR_SM_EVT_DISABLE, 0, &next);
	zassert_equal(err, 0);
	zassert_equal(next, ACTUATOR_STATE_DISABLED);
}

ZTEST(actuator_state_machine, test_aligning_done_to_ready)
{
	enum actuator_state next;
	int err = actuator_sm_step(ACTUATOR_STATE_ALIGNING, ACTUATOR_SM_EVT_ALIGNED,
				   ACTUATOR_CAP_NEEDS_ALIGN, &next);
	zassert_equal(err, 0);
	zassert_equal(next, ACTUATOR_STATE_READY);
}

static const enum actuator_state all_states[] = {
	ACTUATOR_STATE_DISABLED, ACTUATOR_STATE_READY, ACTUATOR_STATE_ALIGNING,
	ACTUATOR_STATE_ACTIVE,   ACTUATOR_STATE_FAULT, ACTUATOR_STATE_STOPPED,
};

ZTEST(actuator_state_machine, test_setpoint_from_disabled_refused)
{
	enum actuator_state next = ACTUATOR_STATE_DISABLED;
	int err = actuator_sm_step(ACTUATOR_STATE_DISABLED, ACTUATOR_SM_EVT_SETPOINT,
				   ACTUATOR_CAP_POSITION, &next);
	zassert_equal(err, -EPERM);
}

ZTEST(actuator_state_machine, test_stop_from_every_state)
{
	for (size_t i = 0; i < ARRAY_SIZE(all_states); i++) {
		enum actuator_state next;
		int err = actuator_sm_step(all_states[i], ACTUATOR_SM_EVT_STOP,
					   ACTUATOR_CAP_POSITION, &next);
		zassert_equal(err, 0, "from=%d", all_states[i]);
		zassert_equal(next, ACTUATOR_STATE_STOPPED, "from=%d", all_states[i]);
	}
}

ZTEST(actuator_state_machine, test_clear_stop_only_from_stopped)
{
	for (size_t i = 0; i < ARRAY_SIZE(all_states); i++) {
		enum actuator_state next;
		int err = actuator_sm_step(all_states[i], ACTUATOR_SM_EVT_CLEAR_STOP,
					   ACTUATOR_CAP_POSITION, &next);
		if (all_states[i] == ACTUATOR_STATE_STOPPED) {
			zassert_equal(err, 0);
			zassert_equal(next, ACTUATOR_STATE_DISABLED);
		} else {
			zassert_equal(err, -EINVAL, "from=%d", all_states[i]);
		}
	}
}

ZTEST(actuator_state_machine, test_stopped_is_sticky)
{
	enum actuator_state next;

	zassert_ok(actuator_sm_step(ACTUATOR_STATE_STOPPED, ACTUATOR_SM_EVT_FAULT, 0, &next));
	zassert_equal(next, ACTUATOR_STATE_STOPPED);
	zassert_ok(actuator_sm_step(ACTUATOR_STATE_STOPPED, ACTUATOR_SM_EVT_DISABLE, 0, &next));
	zassert_equal(next, ACTUATOR_STATE_STOPPED);
	zassert_equal(actuator_sm_step(ACTUATOR_STATE_STOPPED, ACTUATOR_SM_EVT_ALIGNED,
				       ACTUATOR_CAP_NEEDS_ALIGN, &next),
		      -EINVAL);
	zassert_equal(actuator_sm_step(ACTUATOR_STATE_STOPPED, ACTUATOR_SM_EVT_CLEAR_FAULT,
				       ACTUATOR_CAP_FAULT_LATCHING, &next),
		      -EINVAL);
}

ZTEST(actuator_state_machine, test_stopped_refuses_enable_and_setpoint)
{
	enum actuator_state next;

	zassert_equal(actuator_sm_step(ACTUATOR_STATE_STOPPED, ACTUATOR_SM_EVT_ENABLE,
				       ACTUATOR_CAP_POSITION, &next),
		      -EPERM);
	zassert_equal(actuator_sm_step(ACTUATOR_STATE_STOPPED, ACTUATOR_SM_EVT_SETPOINT,
				       ACTUATOR_CAP_POSITION, &next),
		      -EPERM);
}

ZTEST(actuator_state_machine, test_reference_from_ready_and_active)
{
	enum actuator_state next;

	zassert_ok(actuator_sm_step(ACTUATOR_STATE_READY, ACTUATOR_SM_EVT_REFERENCE,
				    ACTUATOR_CAP_NEEDS_REFERENCE, &next));
	zassert_equal(next, ACTUATOR_STATE_ALIGNING);
	zassert_ok(actuator_sm_step(ACTUATOR_STATE_ACTIVE, ACTUATOR_SM_EVT_REFERENCE,
				    ACTUATOR_CAP_NEEDS_REFERENCE, &next));
	zassert_equal(next, ACTUATOR_STATE_ALIGNING);
}

ZTEST(actuator_state_machine, test_reference_refused)
{
	enum actuator_state next;
	const uint32_t caps = ACTUATOR_CAP_NEEDS_REFERENCE;

	zassert_equal(
		actuator_sm_step(ACTUATOR_STATE_DISABLED, ACTUATOR_SM_EVT_REFERENCE, caps, &next),
		-EPERM);
	zassert_equal(
		actuator_sm_step(ACTUATOR_STATE_FAULT, ACTUATOR_SM_EVT_REFERENCE, caps, &next),
		-EPERM);
	zassert_equal(
		actuator_sm_step(ACTUATOR_STATE_STOPPED, ACTUATOR_SM_EVT_REFERENCE, caps, &next),
		-EPERM);
	zassert_equal(
		actuator_sm_step(ACTUATOR_STATE_ALIGNING, ACTUATOR_SM_EVT_REFERENCE, caps, &next),
		-EBUSY);
}

ZTEST(actuator_state_machine, test_enum_values_stable)
{
	zassert_equal(ACTUATOR_STATE_FAULT, 4);
	zassert_equal(ACTUATOR_STATE_STOPPED, 5);
	zassert_equal(ACTUATOR_SM_EVT_CLEAR_FAULT, 5);
	zassert_equal(ACTUATOR_CAP_NEEDS_REFERENCE, BIT(8));
	zassert_equal(ACTUATOR_FAULT_REFERENCE, BIT(6));
}

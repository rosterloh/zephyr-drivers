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

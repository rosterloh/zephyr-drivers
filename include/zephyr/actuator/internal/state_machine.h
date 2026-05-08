/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal state-machine helper. Not part of the public API; consumed by
 * subsys/actuator and exposed for unit tests in lib/actuator.
 */

#ifndef ZEPHYR_INCLUDE_ACTUATOR_INTERNAL_STATE_MACHINE_H_
#define ZEPHYR_INCLUDE_ACTUATOR_INTERNAL_STATE_MACHINE_H_

#include <zephyr/actuator/actuator_types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum actuator_sm_event {
	ACTUATOR_SM_EVT_ENABLE,
	ACTUATOR_SM_EVT_DISABLE,
	ACTUATOR_SM_EVT_SETPOINT,
	ACTUATOR_SM_EVT_ALIGNED,
	ACTUATOR_SM_EVT_FAULT,
	ACTUATOR_SM_EVT_CLEAR_FAULT,
};

/**
 * Compute the next state.
 *
 * @param current Current state.
 * @param event   Event being delivered.
 * @param caps    Driver capability bitmask (ACTUATOR_CAP_*).
 * @param next    On success, written with the next state.
 *
 * @retval 0          Transition is legal; *next is the new state.
 * @retval -EAGAIN    Setpoint while ALIGNING; caller should retry later.
 * @retval -EPERM     Setpoint while FAULT.
 * @retval -ENOTSUP   clear_fault on a non-latching backend (caller may treat
 *                    this as a no-op success).
 */
int actuator_sm_step(enum actuator_state current, enum actuator_sm_event event, uint32_t caps,
		     enum actuator_state *next);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ACTUATOR_INTERNAL_STATE_MACHINE_H_ */

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/actuator/internal/state_machine.h>

int actuator_sm_step(enum actuator_state current, enum actuator_sm_event event, uint32_t caps,
		     enum actuator_state *next)
{
	switch (event) {
	case ACTUATOR_SM_EVT_DISABLE:
		*next = ACTUATOR_STATE_DISABLED;
		return 0;

	case ACTUATOR_SM_EVT_FAULT:
		*next = ACTUATOR_STATE_FAULT;
		return 0;

	case ACTUATOR_SM_EVT_ENABLE:
		if (current == ACTUATOR_STATE_FAULT) {
			return -EPERM;
		}
		*next = (caps & ACTUATOR_CAP_NEEDS_ALIGN) ? ACTUATOR_STATE_ALIGNING
							  : ACTUATOR_STATE_READY;
		return 0;

	case ACTUATOR_SM_EVT_ALIGNED:
		if (current != ACTUATOR_STATE_ALIGNING) {
			return -EINVAL;
		}
		*next = ACTUATOR_STATE_READY;
		return 0;

	case ACTUATOR_SM_EVT_SETPOINT:
		switch (current) {
		case ACTUATOR_STATE_DISABLED:
			/* Implicit promotion. */
			*next = (caps & ACTUATOR_CAP_NEEDS_ALIGN) ? ACTUATOR_STATE_ALIGNING
								  : ACTUATOR_STATE_ACTIVE;
			return 0;
		case ACTUATOR_STATE_READY:
		case ACTUATOR_STATE_ACTIVE:
			*next = ACTUATOR_STATE_ACTIVE;
			return 0;
		case ACTUATOR_STATE_ALIGNING:
			return -EAGAIN;
		case ACTUATOR_STATE_FAULT:
			return -EPERM;
		}
		return -EINVAL;

	case ACTUATOR_SM_EVT_CLEAR_FAULT:
		if (current != ACTUATOR_STATE_FAULT) {
			return -EINVAL;
		}
		if (!(caps & ACTUATOR_CAP_FAULT_LATCHING)) {
			return -ENOTSUP;
		}
		*next = ACTUATOR_STATE_DISABLED;
		return 0;
	}
	return -EINVAL;
}

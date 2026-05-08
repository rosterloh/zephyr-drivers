/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/actuator/internal/capabilities.h>

int actuator_cap_check_mode(uint32_t caps, enum actuator_mode mode)
{
	uint32_t needed;
	switch (mode) {
	case ACTUATOR_MODE_POSITION:
		needed = ACTUATOR_CAP_POSITION;
		break;
	case ACTUATOR_MODE_VELOCITY:
		needed = ACTUATOR_CAP_VELOCITY;
		break;
	case ACTUATOR_MODE_EFFORT:
		needed = ACTUATOR_CAP_EFFORT;
		break;
	case ACTUATOR_MODE_DISABLED:
	default:
		return -EINVAL;
	}
	return (caps & needed) ? 0 : -ENOTSUP;
}

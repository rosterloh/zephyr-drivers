/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Callback registration and dispatch. Full implementation in T10.
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/actuator/actuator.h>

void actuator_callbacks_fire_state(const struct device *dev, enum actuator_state new_state)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(new_state);
	/* Real implementation lands in T10. */
}

void actuator_callbacks_fire_feedback(const struct device *dev, const struct actuator_feedback *fb)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(fb);
}

int actuator_register_state_cb(const struct device *dev, actuator_state_cb_t cb, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return -ENOTSUP;
}

int actuator_register_feedback_cb(const struct device *dev, actuator_feedback_cb_t cb,
				  void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return -ENOTSUP;
}

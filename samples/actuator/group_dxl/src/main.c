/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/actuator_group.h>

LOG_MODULE_REGISTER(actuator_group, LOG_LEVEL_INF);

#define ALPHA DEVICE_DT_GET(DT_NODELABEL(alpha))
#define BETA  DEVICE_DT_GET(DT_NODELABEL(beta))

ACTUATOR_GROUP_DEFINE(arm, ALPHA, BETA);

static void on_state(const struct device *dev, enum actuator_state s, void *ud)
{
	ARG_UNUSED(ud);
	LOG_INF("%s -> state %d", dev->name, s);
}

int main(void)
{
	(void)actuator_register_state_cb(ALPHA, on_state, NULL);
	(void)actuator_register_state_cb(BETA, on_state, NULL);

	(void)actuator_group_set_fault_policy(&arm, ACTUATOR_GROUP_POLICY_DISABLE_ALL);
	(void)actuator_group_enable(&arm);

	float t = 0.0f;
	while (1) {
		float goals[] = {sinf(t), -sinf(t)};
		(void)actuator_group_set_position(&arm, goals);

		struct actuator_feedback fb[2];
		if (actuator_group_read_feedback(&arm, fb) == 0) {
			LOG_INF("alpha=%.3f beta=%.3f", (double)fb[0].position,
				(double)fb[1].position);
		}
		t += 0.1f;
		k_sleep(K_MSEC(50));
	}
	return 0;
}

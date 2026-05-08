/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/actuator/actuator.h>

LOG_MODULE_REGISTER(actuator_basic, LOG_LEVEL_INF);

#define MOTOR DEVICE_DT_GET(DT_NODELABEL(alpha))

int main(void)
{
	if (!device_is_ready(MOTOR)) {
		LOG_ERR("actuator not ready");
		return -ENODEV;
	}
	int err = actuator_enable(MOTOR);
	if (err) {
		LOG_ERR("enable failed: %d", err);
		return err;
	}
	float angle = 0.0f;
	while (1) {
		(void)actuator_set_position(MOTOR, sinf(angle));
		angle += 0.1f;
		k_sleep(K_MSEC(50));
	}
	return 0;
}

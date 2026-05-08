/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_GROUP_H_
#define ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_GROUP_H_

#include <zephyr/device.h>
#include <zephyr/actuator/actuator_types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum actuator_group_fault_policy {
	ACTUATOR_GROUP_POLICY_ISOLATE = 0,
	ACTUATOR_GROUP_POLICY_DISABLE_ALL,
	ACTUATOR_GROUP_POLICY_ESTOP,
};

struct actuator_group_data {
	enum actuator_group_fault_policy policy;
	bool latched;
};

struct actuator_group {
	const struct device *const *devs;
	size_t n;
	struct actuator_group_data *data;
};

#define ACTUATOR_GROUP_DEFINE(name, ...)                                                           \
	static const struct device *_##name##_devs[] = {__VA_ARGS__};                              \
	static struct actuator_group_data _##name##_data;                                          \
	static const struct actuator_group name = {                                                \
		.devs = _##name##_devs,                                                            \
		.n = ARRAY_SIZE(_##name##_devs),                                                   \
		.data = &_##name##_data,                                                           \
	}

int actuator_group_enable(const struct actuator_group *grp);
int actuator_group_disable(const struct actuator_group *grp);
int actuator_group_clear_fault(const struct actuator_group *grp);

int actuator_group_set_position(const struct actuator_group *grp, const float rad[]);
int actuator_group_set_velocity(const struct actuator_group *grp, const float rad_s[]);
int actuator_group_set_effort(const struct actuator_group *grp, const float nm[]);

int actuator_group_read_feedback(const struct actuator_group *grp, struct actuator_feedback fb[]);

int actuator_group_set_fault_policy(const struct actuator_group *grp,
				    enum actuator_group_fault_policy policy);

#ifdef __cplusplus
}
#endif

#endif

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/kinematics/joint.h>

/* Scale factors: devicetree (scaled int) -> SI float. */
#define KIN_AXIS(node, i)   (DT_PROP_BY_IDX(node, axis, i) / 1000.0f)
#define KIN_XYZ(node, p, i) (DT_PROP_BY_IDX(node, p, i) / 1000000.0f)          /* um  -> m   */
#define KIN_RPY(node, i)    (DT_PROP_BY_IDX(node, origin_rpy, i) / 1000000.0f) /* urad -> rad */
#define KIN_ANG(node, p)    (DT_PROP_OR(node, p, 0) / 1000000.0f)              /* urad-> rad */
#define KIN_EFFORT(node)    (DT_PROP_OR(node, limit_effort, 0) / 1000.0f)      /* mNm -> Nm  */
#define KIN_MASS(node)      (DT_PROP(node, mass_mg) / 1000000.0f)              /* mg  -> kg  */
#define KIN_INERTIA(node, i)                                                                       \
	(DT_PROP_BY_IDX(node, inertia, i) / 1000000000.0f) /* g*mm^2 -> kg*m^2 */

#define KIN_HAS_LIMITS(node)                                                                       \
	(DT_NODE_HAS_PROP(node, limit_lower) || DT_NODE_HAS_PROP(node, limit_upper) ||             \
	 DT_NODE_HAS_PROP(node, limit_velocity) || DT_NODE_HAS_PROP(node, limit_effort))

#define KIN_DYNAMICS(node)                                                                         \
	COND_CODE_1(                                                                               \
		DT_NODE_HAS_PROP(node, mass_mg),                                                   \
		({.mass = KIN_MASS(node),                                                          \
		  .com = {KIN_XYZ(node, com_xyz, 0), KIN_XYZ(node, com_xyz, 1),                    \
			  KIN_XYZ(node, com_xyz, 2)},                                              \
		  .inertia = {KIN_INERTIA(node, 0), KIN_INERTIA(node, 1), KIN_INERTIA(node, 2),    \
			      KIN_INERTIA(node, 3), KIN_INERTIA(node, 4), KIN_INERTIA(node, 5)}}), \
		({0}))

#define KIN_ACTUATOR_NAME(node)                                                                    \
	COND_CODE_1(DT_NODE_HAS_PROP(node, actuator),                                              \
		    (DT_NODE_FULL_NAME(DT_PHANDLE(node, actuator))), (NULL))

#define JOINT_ENTRY(node)                                                                          \
	{                                                                                          \
		.parent_link = DT_PROP(node, link_parent),                                         \
		.child_link = DT_PROP(node, link_child),                                           \
		.type = (enum joint_type)DT_ENUM_IDX(node, joint_type),                            \
		.axis = {KIN_AXIS(node, 0), KIN_AXIS(node, 1), KIN_AXIS(node, 2)},                 \
		.origin_xyz = {KIN_XYZ(node, origin_xyz, 0), KIN_XYZ(node, origin_xyz, 1),         \
			       KIN_XYZ(node, origin_xyz, 2)},                                      \
		.origin_rpy = {KIN_RPY(node, 0), KIN_RPY(node, 1), KIN_RPY(node, 2)},              \
		.limits = {.lower = KIN_ANG(node, limit_lower),                                    \
			   .upper = KIN_ANG(node, limit_upper),                                    \
			   .velocity = KIN_ANG(node, limit_velocity),                              \
			   .effort = KIN_EFFORT(node)},                                            \
		.dynamics = KIN_DYNAMICS(node),                                                    \
		.actuator_name = KIN_ACTUATOR_NAME(node),                                          \
		.valid_mask = (KIN_HAS_LIMITS(node) ? JOINT_HAS_LIMITS : 0) |                      \
			      (DT_NODE_HAS_PROP(node, mass_mg) ? JOINT_HAS_DYNAMICS : 0) |         \
			      (DT_NODE_HAS_PROP(node, actuator) ? JOINT_HAS_ACTUATOR : 0),         \
	},

static const struct joint_descriptor joints[] = {
	DT_FOREACH_STATUS_OKAY(rosterloh_joint, JOINT_ENTRY)};

size_t kinematics_joint_count(void)
{
	return ARRAY_SIZE(joints);
}

const struct joint_descriptor *kinematics_joint_get(size_t idx)
{
	if (idx >= ARRAY_SIZE(joints)) {
		return NULL;
	}
	return &joints[idx];
}

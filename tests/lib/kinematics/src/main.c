/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>
#include <zephyr/kinematics/joint.h>

#define EPS 1e-4f

ZTEST_SUITE(kinematics_table, NULL, NULL, NULL, NULL, NULL);

ZTEST(kinematics_table, test_count)
{
	zassert_equal(kinematics_joint_count(), 2, "expected 2 joints");
}

ZTEST(kinematics_table, test_fixed_joint_mandatory)
{
	const struct joint_descriptor *j = kinematics_joint_get(0);

	zassert_not_null(j);
	zassert_str_equal(j->parent_link, "world");
	zassert_str_equal(j->child_link, "base_link");
	zassert_equal(j->type, JOINT_TYPE_FIXED);
	zassert_within(j->axis[2], 1.0f, EPS);
	zassert_within(j->origin_xyz[0], 0.0f, EPS);
	zassert_equal(j->valid_mask, 0, "fixed joint has no optional blocks");
}

ZTEST(kinematics_table, test_revolute_conversions)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_equal(j->type, JOINT_TYPE_REVOLUTE);
	zassert_within(j->origin_xyz[0], 0.05f, EPS);
	zassert_within(j->origin_xyz[2], 0.1f, EPS);
	zassert_within(j->origin_rpy[2], 1.570796f, EPS);
}

ZTEST(kinematics_table, test_revolute_limits_and_mask)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_true(j->valid_mask & JOINT_HAS_LIMITS);
	zassert_within(j->limits.lower, -1.570796f, EPS);
	zassert_within(j->limits.upper, 1.570796f, EPS);
	zassert_within(j->limits.velocity, 3.141592f, EPS);
	zassert_within(j->limits.effort, 2.5f, EPS);
}

ZTEST(kinematics_table, test_revolute_dynamics)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_true(j->valid_mask & JOINT_HAS_DYNAMICS);
	zassert_within(j->dynamics.mass, 0.25f, EPS);
	zassert_within(j->dynamics.com[2], 0.03f, EPS);
	zassert_within(j->dynamics.inertia[0], 1e-3f, EPS);
	zassert_within(j->dynamics.inertia[2], 5e-4f, EPS);
}

ZTEST(kinematics_table, test_actuator_link)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_true(j->valid_mask & JOINT_HAS_ACTUATOR);
	zassert_not_null(j->actuator_name);
	zassert_str_equal(j->actuator_name, "fake-act");
}

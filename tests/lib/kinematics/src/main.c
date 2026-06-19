/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>
#include <zephyr/kinematics/joint.h>
#include <zcbor_decode.h>
#include <zcbor_common.h>

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

ZTEST_SUITE(kinematics_cbor, NULL, NULL, NULL, NULL, NULL);

ZTEST(kinematics_cbor, test_too_small_buffer)
{
	uint8_t buf[4];
	size_t out_len = 0;

	zassert_equal(kinematics_encode_cbor(buf, sizeof(buf), &out_len), -ENOMEM);
}

ZTEST(kinematics_cbor, test_roundtrip_first_joint)
{
	uint8_t buf[512];
	size_t out_len = 0;

	zassert_equal(kinematics_encode_cbor(buf, sizeof(buf), &out_len), 0);
	zassert_true(out_len > 0);

	ZCBOR_STATE_D(ds, 4, buf, out_len, 1, 0);

	uint32_t version = 0;
	zassert_true(zcbor_map_start_decode(ds));
	zassert_true(zcbor_tstr_expect_lit(ds, "v"));
	zassert_true(zcbor_uint32_decode(ds, &version));
	zassert_equal(version, KINEMATICS_SCHEMA_VERSION);

	zassert_true(zcbor_tstr_expect_lit(ds, "joints"));
	zassert_true(zcbor_list_start_decode(ds));

	/* First joint: mandatory-only (the fixed "world"->"base_link" joint). */
	struct zcbor_string s;
	uint32_t type = 0;
	float axis[3], origin[6];

	zassert_true(zcbor_map_start_decode(ds));

	zassert_true(zcbor_tstr_expect_lit(ds, "parent"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_equal(s.len, strlen("world"));
	zassert_mem_equal(s.value, "world", s.len);

	zassert_true(zcbor_tstr_expect_lit(ds, "child"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_mem_equal(s.value, "base_link", s.len);

	zassert_true(zcbor_tstr_expect_lit(ds, "type"));
	zassert_true(zcbor_uint32_decode(ds, &type));
	zassert_equal(type, JOINT_TYPE_FIXED);

	zassert_true(zcbor_tstr_expect_lit(ds, "axis"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 3; i++) {
		zassert_true(zcbor_float32_decode(ds, &axis[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_within(axis[2], 1.0f, EPS);

	zassert_true(zcbor_tstr_expect_lit(ds, "origin"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 6; i++) {
		zassert_true(zcbor_float32_decode(ds, &origin[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_within(origin[0], 0.0f, EPS);

	/* joints[0] has no optional blocks, so the joint map ends here. */
	zassert_true(zcbor_map_end_decode(ds));
}

ZTEST(kinematics_cbor, test_roundtrip_optional_blocks)
{
	uint8_t buf[512];
	size_t out_len = 0;

	zassert_equal(kinematics_encode_cbor(buf, sizeof(buf), &out_len), 0);
	zassert_true(out_len > 0);

	ZCBOR_STATE_D(ds, 4, buf, out_len, 1, 0);

	/* Skip the top-level map header and version field. */
	uint32_t version = 0;

	zassert_true(zcbor_map_start_decode(ds));
	zassert_true(zcbor_tstr_expect_lit(ds, "v"));
	zassert_true(zcbor_uint32_decode(ds, &version));

	zassert_true(zcbor_tstr_expect_lit(ds, "joints"));
	zassert_true(zcbor_list_start_decode(ds));

	/* Skip joints[0] (mandatory-only fixed joint): parent, child, type, axis, origin. */
	struct zcbor_string s;
	uint32_t type = 0;
	float tmp[6];

	zassert_true(zcbor_map_start_decode(ds));
	zassert_true(zcbor_tstr_expect_lit(ds, "parent"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_true(zcbor_tstr_expect_lit(ds, "child"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_true(zcbor_tstr_expect_lit(ds, "type"));
	zassert_true(zcbor_uint32_decode(ds, &type));
	zassert_true(zcbor_tstr_expect_lit(ds, "axis"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 3; i++) {
		zassert_true(zcbor_float32_decode(ds, &tmp[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_true(zcbor_tstr_expect_lit(ds, "origin"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 6; i++) {
		zassert_true(zcbor_float32_decode(ds, &tmp[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_true(zcbor_map_end_decode(ds));

	/*
	 * joints[1]: revolute "base_link"->"upper_arm" with all three optional
	 * blocks (limits, dyn, actuator).  Key order matches the encoder.
	 */
	uint32_t type2 = 0;
	float axis2[3], origin2[6];
	float limits[4], dyn[10];
	struct zcbor_string act_name;

	zassert_true(zcbor_map_start_decode(ds));

	zassert_true(zcbor_tstr_expect_lit(ds, "parent"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_mem_equal(s.value, "base_link", s.len);

	zassert_true(zcbor_tstr_expect_lit(ds, "child"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_mem_equal(s.value, "upper_arm", s.len);

	zassert_true(zcbor_tstr_expect_lit(ds, "type"));
	zassert_true(zcbor_uint32_decode(ds, &type2));
	zassert_equal(type2, JOINT_TYPE_REVOLUTE);

	zassert_true(zcbor_tstr_expect_lit(ds, "axis"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 3; i++) {
		zassert_true(zcbor_float32_decode(ds, &axis2[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));

	zassert_true(zcbor_tstr_expect_lit(ds, "origin"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 6; i++) {
		zassert_true(zcbor_float32_decode(ds, &origin2[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));

	/* limits: [lower, upper, velocity, effort] — effort == 2.5 N*m */
	zassert_true(zcbor_tstr_expect_lit(ds, "limits"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 4; i++) {
		zassert_true(zcbor_float32_decode(ds, &limits[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_within(limits[3], 2.5f, EPS);

	/* dyn: [mass, com0, com1, com2, inertia0..5] — mass == 0.25 kg */
	zassert_true(zcbor_tstr_expect_lit(ds, "dyn"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 10; i++) {
		zassert_true(zcbor_float32_decode(ds, &dyn[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_within(dyn[0], 0.25f, EPS);

	/* actuator: tstr == "fake-act" */
	zassert_true(zcbor_tstr_expect_lit(ds, "actuator"));
	zassert_true(zcbor_tstr_decode(ds, &act_name));
	zassert_equal(act_name.len, strlen("fake-act"));
	zassert_mem_equal(act_name.value, "fake-act", act_name.len);

	zassert_true(zcbor_map_end_decode(ds));
}

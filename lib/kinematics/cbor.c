/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zcbor_common.h>
#include <zcbor_encode.h>
#include <zephyr/kinematics/joint.h>

#define KIN_STR_MAX   64 /* max link/actuator name length to scan */
#define KIN_JOINT_MAP 8  /* upper bound on key-value pairs per joint map */

static bool put_floats(zcbor_state_t *st, const float *v, size_t n)
{
	bool ok = zcbor_list_start_encode(st, n);

	for (size_t i = 0; i < n && ok; i++) {
		ok = ok && zcbor_float32_put(st, v[i]);
	}
	return ok && zcbor_list_end_encode(st, n);
}

static bool encode_joint(zcbor_state_t *st, const struct joint_descriptor *j)
{
	float origin[6] = {j->origin_xyz[0], j->origin_xyz[1], j->origin_xyz[2],
			   j->origin_rpy[0], j->origin_rpy[1], j->origin_rpy[2]};
	bool ok = zcbor_map_start_encode(st, KIN_JOINT_MAP);

	ok = ok && zcbor_tstr_put_lit(st, "parent");
	ok = ok && zcbor_tstr_put_term(st, j->parent_link, KIN_STR_MAX);
	ok = ok && zcbor_tstr_put_lit(st, "child");
	ok = ok && zcbor_tstr_put_term(st, j->child_link, KIN_STR_MAX);
	ok = ok && zcbor_tstr_put_lit(st, "type");
	ok = ok && zcbor_uint32_put(st, (uint32_t)j->type);
	ok = ok && zcbor_tstr_put_lit(st, "axis");
	ok = ok && put_floats(st, j->axis, 3);
	ok = ok && zcbor_tstr_put_lit(st, "origin");
	ok = ok && put_floats(st, origin, 6);

	if (ok && (j->valid_mask & JOINT_HAS_LIMITS)) {
		float limits[4] = {j->limits.lower, j->limits.upper, j->limits.velocity,
				   j->limits.effort};
		ok = ok && zcbor_tstr_put_lit(st, "limits");
		ok = ok && put_floats(st, limits, 4);
	}
	if (ok && (j->valid_mask & JOINT_HAS_DYNAMICS)) {
		float dyn[10] = {j->dynamics.mass,       j->dynamics.com[0],
				 j->dynamics.com[1],     j->dynamics.com[2],
				 j->dynamics.inertia[0], j->dynamics.inertia[1],
				 j->dynamics.inertia[2], j->dynamics.inertia[3],
				 j->dynamics.inertia[4], j->dynamics.inertia[5]};
		ok = ok && zcbor_tstr_put_lit(st, "dyn");
		ok = ok && put_floats(st, dyn, 10);
	}
	if (ok && (j->valid_mask & JOINT_HAS_ACTUATOR)) {
		ok = ok && zcbor_tstr_put_lit(st, "actuator");
		ok = ok && zcbor_tstr_put_term(st, j->actuator_name, KIN_STR_MAX);
	}

	return ok && zcbor_map_end_encode(st, KIN_JOINT_MAP);
}

int kinematics_encode_cbor(uint8_t *buf, size_t len, size_t *out_len)
{
	ZCBOR_STATE_E(st, 4, buf, len, 1);
	const size_t n = kinematics_joint_count();
	bool ok = zcbor_map_start_encode(st, 2);

	ok = ok && zcbor_tstr_put_lit(st, "v");
	ok = ok && zcbor_uint32_put(st, KINEMATICS_SCHEMA_VERSION);
	ok = ok && zcbor_tstr_put_lit(st, "joints");
	ok = ok && zcbor_list_start_encode(st, n);
	for (size_t i = 0; i < n && ok; i++) {
		ok = ok && encode_joint(st, kinematics_joint_get(i));
	}
	ok = ok && zcbor_list_end_encode(st, n);
	ok = ok && zcbor_map_end_encode(st, 2);

	if (!ok) {
		return -ENOMEM;
	}
	*out_len = (size_t)(st->payload - buf);
	return 0;
}

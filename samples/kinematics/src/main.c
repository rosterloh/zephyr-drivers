/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kinematics/joint.h>

static const char *type_str(enum joint_type t)
{
	switch (t) {
	case JOINT_TYPE_FIXED:
		return "fixed";
	case JOINT_TYPE_REVOLUTE:
		return "revolute";
	case JOINT_TYPE_CONTINUOUS:
		return "continuous";
	case JOINT_TYPE_PRISMATIC:
		return "prismatic";
	default:
		return "?";
	}
}

int main(void)
{
	const size_t n = kinematics_joint_count();

	printk("kinematics: %zu joint(s)\n", n);
	for (size_t i = 0; i < n; i++) {
		const struct joint_descriptor *j = kinematics_joint_get(i);

		printk("  [%zu] %s -> %s (%s) origin=(%.3f %.3f %.3f) actuated=%d\n", i,
		       j->parent_link, j->child_link, type_str(j->type), (double)j->origin_xyz[0],
		       (double)j->origin_xyz[1], (double)j->origin_xyz[2],
		       (j->valid_mask & JOINT_HAS_ACTUATOR) ? 1 : 0);
	}

	uint8_t buf[512];
	size_t out_len = 0;
	int rc = kinematics_encode_cbor(buf, sizeof(buf), &out_len);

	if (rc != 0) {
		printk("encode failed: %d\n", rc);
		return 0;
	}

	printk("CBOR (%zu bytes):", out_len);
	for (size_t i = 0; i < out_len; i++) {
		printk(" %02x", buf[i]);
	}
	printk("\n");
	return 0;
}

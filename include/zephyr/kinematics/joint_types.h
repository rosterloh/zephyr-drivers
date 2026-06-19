/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_KINEMATICS_JOINT_TYPES_H_
#define ZEPHYR_INCLUDE_KINEMATICS_JOINT_TYPES_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CBOR wire-format schema version. */
#define KINEMATICS_SCHEMA_VERSION 1

/** URDF joint types. Order matches the binding's joint-type enum. */
enum joint_type {
	JOINT_TYPE_FIXED = 0,
	JOINT_TYPE_REVOLUTE,
	JOINT_TYPE_CONTINUOUS,
	JOINT_TYPE_PRISMATIC,
};

/** Bits set in joint_descriptor.valid_mask for optional blocks. */
#define JOINT_HAS_LIMITS   BIT(0)
#define JOINT_HAS_DYNAMICS BIT(1)
#define JOINT_HAS_ACTUATOR BIT(2)

/**
 * One joint exposed by this device. All values are SI: metres, radians,
 * kg, kg*m^2, N*m, rad/s. Optional blocks are valid only when the matching
 * valid_mask bit is set.
 */
struct joint_descriptor {
	const char *parent_link;
	const char *child_link;
	enum joint_type type;
	float axis[3];       /**< unit vector            */
	float origin_xyz[3]; /**< m                      */
	float origin_rpy[3]; /**< rad                    */
	struct {
		float lower; /**< rad or m               */
		float upper;
		float velocity; /**< rad/s or m/s          */
		float effort;   /**< N*m or N               */
	} limits;
	struct {
		float mass;       /**< kg                 */
		float com[3];     /**< m                  */
		float inertia[6]; /**< kg*m^2 [ixx iyy izz ixy ixz iyz] */
	} dynamics;
	const char *actuator_name; /**< NULL unless JOINT_HAS_ACTUATOR */
	uint8_t valid_mask;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KINEMATICS_JOINT_TYPES_H_ */

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ACTUATOR_TYPES_H_
#define ZEPHYR_INCLUDE_ACTUATOR_TYPES_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Operating modes. Drivers advertise which they support via capabilities. */
enum actuator_mode {
	ACTUATOR_MODE_DISABLED = 0,
	ACTUATOR_MODE_POSITION, /**< setpoint in radians                  */
	ACTUATOR_MODE_VELOCITY, /**< setpoint in rad/s                    */
	ACTUATOR_MODE_EFFORT,   /**< setpoint in N*m (torque)             */
};

/** Driver capability bits. Returned by actuator_get_capabilities(). */
#define ACTUATOR_CAP_POSITION       BIT(0)
#define ACTUATOR_CAP_VELOCITY       BIT(1)
#define ACTUATOR_CAP_EFFORT         BIT(2)
#define ACTUATOR_CAP_NEEDS_ALIGN    BIT(3) /**< FOC sensorless / Hall align */
#define ACTUATOR_CAP_GROUP_NATIVE   BIT(4) /**< backend has a real group_op */
#define ACTUATOR_CAP_FAULT_LATCHING BIT(5) /**< faults need explicit clear  */
#define ACTUATOR_CAP_DRIVE_MODE     BIT(6) /**< supports actuator_set_drive_mode  */

/** State machine. Owned by the subsystem; transitions reported by drivers. */
enum actuator_state {
	ACTUATOR_STATE_DISABLED = 0,
	ACTUATOR_STATE_READY,    /**< enabled, holding zero effort  */
	ACTUATOR_STATE_ALIGNING, /**< FOC pre-run only              */
	ACTUATOR_STATE_ACTIVE,   /**< tracking a setpoint           */
	ACTUATOR_STATE_FAULT,
};

/** Output policy of the power stage. Orthogonal to actuator_state. */
enum actuator_drive_mode {
	ACTUATOR_DRIVE_MODE_NORMAL = 0, /**< PWM-controlled output (default) */
	ACTUATOR_DRIVE_MODE_BRAKE,      /**< short motor windings to GND     */
	ACTUATOR_DRIVE_MODE_COAST,      /**< high-Z, free-wheel              */
};

/** Generic fault flags. Driver-specific bits live in upper 16. */
#define ACTUATOR_FAULT_OVERCURRENT  BIT(0)
#define ACTUATOR_FAULT_OVERTEMP     BIT(1)
#define ACTUATOR_FAULT_OVERVOLTAGE  BIT(2)
#define ACTUATOR_FAULT_UNDERVOLTAGE BIT(3)
#define ACTUATOR_FAULT_OVERLOAD     BIT(4)
#define ACTUATOR_FAULT_COMM         BIT(5)
#define ACTUATOR_FAULT_DRIVER(n)    BIT(16 + (n))

/** Bits for actuator_feedback.valid_mask. */
#define ACTUATOR_FB_POSITION    BIT(0)
#define ACTUATOR_FB_VELOCITY    BIT(1)
#define ACTUATOR_FB_EFFORT      BIT(2)
#define ACTUATOR_FB_TEMPERATURE BIT(3)

/** Snapshot of feedback. valid_mask says which fields the backend filled. */
struct actuator_feedback {
	uint32_t valid_mask;
	float position;    /**< rad                            */
	float velocity;    /**< rad/s                          */
	float effort;      /**< N*m                            */
	float temperature; /**< degC; NaN if not present       */
	uint32_t fault_flags;
	uint64_t timestamp_us; /**< k_uptime_get() at sample time  */
};

/** Optional limits. Use NaN to leave a value at the backend default. */
struct actuator_limits {
	float min_position; /**< rad   */
	float max_position; /**< rad   */
	float max_velocity; /**< rad/s */
	float max_effort;   /**< N*m   */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ACTUATOR_TYPES_H_ */

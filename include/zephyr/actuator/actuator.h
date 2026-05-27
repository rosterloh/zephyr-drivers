/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_H_
#define ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_H_

#include <zephyr/device.h>
#include <zephyr/actuator/actuator_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup actuator Generic actuator API
 * @{
 */

__syscall int actuator_enable(const struct device *dev);
__syscall int actuator_disable(const struct device *dev);
__syscall int actuator_clear_fault(const struct device *dev);

__syscall enum actuator_state actuator_get_state(const struct device *dev);
__syscall uint32_t actuator_get_capabilities(const struct device *dev);

/**
 * Command a position setpoint (radians).
 * Implicitly transitions the actuator from DISABLED through READY into
 * ACTIVE if needed.
 *
 * @retval 0         Setpoint accepted.
 * @retval -ENOTSUP  Backend does not support position mode.
 * @retval -EAGAIN   Actuator is in ALIGNING state; retry later.
 * @retval -EPERM    Actuator is in FAULT state; clear fault first.
 */
__syscall int actuator_set_position(const struct device *dev, float rad);
__syscall int actuator_set_velocity(const struct device *dev, float rad_s);
__syscall int actuator_set_effort(const struct device *dev, float nm);

/**
 * Set the output policy of the actuator's power stage.
 *
 * Orthogonal to the actuator state machine: setting a drive mode does not
 * change actuator_get_state(). Setting any setpoint (position/velocity/effort)
 * implicitly returns the stage to ACTUATOR_DRIVE_MODE_NORMAL.
 *
 * NORMAL semantics: a backend that supports drive_mode treats NORMAL as
 * "release brake/coast and stop driving" — outputs are left at zero (high-Z
 * on IN1/IN2-style hardware) until the next setpoint is issued. Use NORMAL
 * to clear an explicit BRAKE/COAST policy; do not expect the motor to keep
 * tracking its previous setpoint after calling it standalone.
 *
 * @retval 0         Mode applied.
 * @retval -ENOTSUP  Backend does not advertise ACTUATOR_CAP_DRIVE_MODE.
 * @retval -EPERM    Actuator is in DISABLED or FAULT state.
 */
__syscall int actuator_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode);

/** Synchronous read: forces a backend transaction. */
__syscall int actuator_read_feedback(const struct device *dev, struct actuator_feedback *out);

/** Cached read: returns last sample populated by the driver-internal worker. */
__syscall int actuator_get_feedback(const struct device *dev, struct actuator_feedback *out);

__syscall int actuator_set_limits(const struct device *dev, const struct actuator_limits *limits);

typedef void (*actuator_state_cb_t)(const struct device *dev, enum actuator_state new_state,
				    void *user_data);
typedef void (*actuator_feedback_cb_t)(const struct device *dev, const struct actuator_feedback *fb,
				       void *user_data);

int actuator_register_state_cb(const struct device *dev, actuator_state_cb_t cb, void *user_data);
int actuator_register_feedback_cb(const struct device *dev, actuator_feedback_cb_t cb,
				  void *user_data);

/** @} */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/actuator.h>

#endif /* ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_H_ */

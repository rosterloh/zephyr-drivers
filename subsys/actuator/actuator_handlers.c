/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/actuator/actuator.h>
#include <zephyr/internal/syscall_handler.h>

static inline int z_vrfy_actuator_enable(const struct device *dev)
{
	K_OOPS(K_SYSCALL_DRIVER_ACTUATOR(dev, enable));
	return z_impl_actuator_enable(dev);
}
#include <zephyr/syscalls/actuator_enable_mrsh.c>

static inline int z_vrfy_actuator_disable(const struct device *dev)
{
	K_OOPS(K_SYSCALL_DRIVER_ACTUATOR(dev, disable));
	return z_impl_actuator_disable(dev);
}
#include <zephyr/syscalls/actuator_disable_mrsh.c>

static inline int z_vrfy_actuator_clear_fault(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_clear_fault(dev);
}
#include <zephyr/syscalls/actuator_clear_fault_mrsh.c>

static inline int z_vrfy_actuator_stop(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_stop(dev);
}
#include <zephyr/syscalls/actuator_stop_mrsh.c>

static inline int z_vrfy_actuator_stop_latch(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_stop_latch(dev);
}
#include <zephyr/syscalls/actuator_stop_latch_mrsh.c>

static inline int z_vrfy_actuator_clear_stop(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_clear_stop(dev);
}
#include <zephyr/syscalls/actuator_clear_stop_mrsh.c>

static inline int z_vrfy_actuator_reference(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_reference(dev);
}
#include <zephyr/syscalls/actuator_reference_mrsh.c>

static inline bool z_vrfy_actuator_is_referenced(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_is_referenced(dev);
}
#include <zephyr/syscalls/actuator_is_referenced_mrsh.c>

static inline void z_vrfy_actuator_invalidate_reference(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	z_impl_actuator_invalidate_reference(dev);
}
#include <zephyr/syscalls/actuator_invalidate_reference_mrsh.c>

static inline enum actuator_state z_vrfy_actuator_get_state(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_get_state(dev);
}
#include <zephyr/syscalls/actuator_get_state_mrsh.c>

static inline uint32_t z_vrfy_actuator_get_capabilities(const struct device *dev)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_get_capabilities(dev);
}
#include <zephyr/syscalls/actuator_get_capabilities_mrsh.c>

static inline int z_vrfy_actuator_set_position(const struct device *dev, float rad)
{
	K_OOPS(K_SYSCALL_DRIVER_ACTUATOR(dev, set_setpoint));
	return z_impl_actuator_set_position(dev, rad);
}
#include <zephyr/syscalls/actuator_set_position_mrsh.c>

static inline int z_vrfy_actuator_set_velocity(const struct device *dev, float rad_s)
{
	K_OOPS(K_SYSCALL_DRIVER_ACTUATOR(dev, set_setpoint));
	return z_impl_actuator_set_velocity(dev, rad_s);
}
#include <zephyr/syscalls/actuator_set_velocity_mrsh.c>

static inline int z_vrfy_actuator_set_effort(const struct device *dev, float nm)
{
	K_OOPS(K_SYSCALL_DRIVER_ACTUATOR(dev, set_setpoint));
	return z_impl_actuator_set_effort(dev, nm);
}
#include <zephyr/syscalls/actuator_set_effort_mrsh.c>

static inline int z_vrfy_actuator_set_duty(const struct device *dev, float duty)
{
	K_OOPS(K_SYSCALL_DRIVER_ACTUATOR(dev, set_setpoint));
	return z_impl_actuator_set_duty(dev, duty);
}
#include <zephyr/syscalls/actuator_set_duty_mrsh.c>

static inline int z_vrfy_actuator_set_drive_mode(const struct device *dev,
						 enum actuator_drive_mode mode)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	return z_impl_actuator_set_drive_mode(dev, mode);
}
#include <zephyr/syscalls/actuator_set_drive_mode_mrsh.c>

static inline int z_vrfy_actuator_read_feedback(const struct device *dev,
						struct actuator_feedback *out)
{
	K_OOPS(K_SYSCALL_DRIVER_ACTUATOR(dev, read_feedback));
	K_OOPS(K_SYSCALL_MEMORY_WRITE(out, sizeof(*out)));
	return z_impl_actuator_read_feedback(dev, out);
}
#include <zephyr/syscalls/actuator_read_feedback_mrsh.c>

static inline int z_vrfy_actuator_get_feedback(const struct device *dev,
					       struct actuator_feedback *out)
{
	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	K_OOPS(K_SYSCALL_MEMORY_WRITE(out, sizeof(*out)));
	return z_impl_actuator_get_feedback(dev, out);
}
#include <zephyr/syscalls/actuator_get_feedback_mrsh.c>

static inline int z_vrfy_actuator_set_limits(const struct device *dev,
					     const struct actuator_limits *limits)
{
	struct actuator_limits k;

	K_OOPS(K_SYSCALL_OBJ(dev, K_OBJ_DRIVER_ACTUATOR));
	/* Copied once: a user thread could rewrite it between check and use. */
	K_OOPS(k_usermode_from_copy(&k, limits, sizeof(k)));
	return z_impl_actuator_set_limits(dev, &k);
}
#include <zephyr/syscalls/actuator_set_limits_mrsh.c>

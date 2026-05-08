/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ACTUATOR_INTERNAL_CAPABILITIES_H_
#define ZEPHYR_INCLUDE_ACTUATOR_INTERNAL_CAPABILITIES_H_

#include <zephyr/actuator/actuator_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @retval 0       Mode is supported.
 * @retval -EINVAL Mode is ACTUATOR_MODE_DISABLED (not a setpoint mode).
 * @retval -ENOTSUP Backend does not advertise this mode.
 */
int actuator_cap_check_mode(uint32_t caps, enum actuator_mode mode);

#ifdef __cplusplus
}
#endif

#endif

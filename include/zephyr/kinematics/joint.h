/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_KINEMATICS_JOINT_H_
#define ZEPHYR_INCLUDE_KINEMATICS_JOINT_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kinematics/joint_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup kinematics Kinematics joint descriptors
 * @{
 */

/** Number of joints this device declares. */
size_t kinematics_joint_count(void);

/** Get joint @p idx, or NULL if out of range. */
const struct joint_descriptor *kinematics_joint_get(size_t idx);

/**
 * Serialize all joints to a CBOR payload (see docs CDDL).
 *
 * @param buf      Output buffer.
 * @param len      Buffer size in bytes.
 * @param out_len  Set to the encoded length on success.
 * @retval 0       Encoded.
 * @retval -ENOMEM Buffer too small.
 */
int kinematics_encode_cbor(uint8_t *buf, size_t len, size_t *out_len);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KINEMATICS_JOINT_H_ */

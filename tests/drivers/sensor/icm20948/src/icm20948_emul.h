/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TESTS_DRIVERS_SENSOR_ICM20948_EMUL_H_
#define TESTS_DRIVERS_SENSOR_ICM20948_EMUL_H_

#include <zephyr/drivers/emul.h>

#define ICM20948_EMUL_BANKS 4
#define ICM20948_EMUL_REGS  128

/** Restore power-on register contents (WHO_AM_I answering, bank 0 selected). */
void icm20948_emul_reset(const struct emul *target);

/** Write one register in @p bank. */
void icm20948_emul_set_reg(const struct emul *target, uint8_t bank, uint8_t reg, uint8_t val);

/** Read one register from @p bank. */
uint8_t icm20948_emul_get_reg(const struct emul *target, uint8_t bank, uint8_t reg);

/** Place a big-endian 16-bit sample at @p reg in bank 0. */
void icm20948_emul_set_sample(const struct emul *target, uint8_t reg, int16_t val);

/** The bank the device is currently pointed at. */
uint8_t icm20948_emul_current_bank(const struct emul *target);

#endif /* TESTS_DRIVERS_SENSOR_ICM20948_EMUL_H_ */

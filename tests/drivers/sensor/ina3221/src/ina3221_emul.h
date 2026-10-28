/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INA3221_EMUL_H_
#define INA3221_EMUL_H_

#include <stdint.h>

/** Set a 16-bit register as the device would hold it (before the driver's >> 3). */
void ina3221_emul_set_reg(uint8_t reg, uint16_t val);

/** Number of conversions the driver has triggered since boot. */
uint32_t ina3221_emul_conversions(void);

#endif /* INA3221_EMUL_H_ */

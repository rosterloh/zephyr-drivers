/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LTR308_H_
#define ZEPHYR_DRIVERS_SENSOR_LTR308_H_

#include <stdint.h>

#define LTR308_REG_MAIN_CTRL 0x00
#define LTR308_REG_MEAS_RATE 0x04
#define LTR308_REG_GAIN      0x05
#define LTR308_REG_PART_ID   0x06
#define LTR308_REG_STATUS    0x07
#define LTR308_REG_DATA_0    0x0d

#define LTR308_PART_ID    0xb1
#define LTR308_ALS_ENABLE 0x02
#define LTR308_DATA_MASK  0x0fffff
#define LTR308_STATUS_NEW 0x08

/*
 * Lux = counts * 0.6 / gain * (100 ms / integration time), from Lite-On's
 * datasheet (window factor 1). In micro-lux; 64-bit as 20-bit counts
 * overflow 32 bits.
 */
static inline uint64_t ltr308_ulux(uint32_t counts, uint8_t gain, uint16_t integration_ms)
{
	return (uint64_t)counts * 600000U * 100U / ((uint64_t)gain * integration_ms);
}

#endif /* ZEPHYR_DRIVERS_SENSOR_LTR308_H_ */

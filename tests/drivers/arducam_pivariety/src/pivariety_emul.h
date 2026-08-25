/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PIVARIETY_EMUL_H_
#define PIVARIETY_EMUL_H_

#include <stdbool.h>
#include <stdint.h>

#define DEVICE_REG_BASE    0x0100
#define PIXFORMAT_REG_BASE 0x0200
#define FORMAT_REG_BASE    0x0300
#define CTRL_REG_BASE      0x0400

#define MODE_SELECT_REG    (DEVICE_REG_BASE | 0x0000)
#define DEVICE_VERSION_REG (DEVICE_REG_BASE | 0x0001)
#define SENSOR_ID_REG      (DEVICE_REG_BASE | 0x0002)
#define DEVICE_ID_REG      (DEVICE_REG_BASE | 0x0003)
#define SYSTEM_IDLE_REG    (DEVICE_REG_BASE | 0x0007)

#define PIXFORMAT_INDEX_REG (PIXFORMAT_REG_BASE | 0x0000)
#define PIXFORMAT_TYPE_REG  (PIXFORMAT_REG_BASE | 0x0001)
#define PIXFORMAT_ORDER_REG (PIXFORMAT_REG_BASE | 0x0002)
#define MIPI_LANES_REG      (PIXFORMAT_REG_BASE | 0x0003)

#define RESOLUTION_INDEX_REG (FORMAT_REG_BASE | 0x0000)
#define FORMAT_WIDTH_REG     (FORMAT_REG_BASE | 0x0001)
#define FORMAT_HEIGHT_REG    (FORMAT_REG_BASE | 0x0002)

#define CTRL_INDEX_REG (CTRL_REG_BASE | 0x0000)
#define CTRL_ID_REG    (CTRL_REG_BASE | 0x0001)
#define CTRL_MIN_REG   (CTRL_REG_BASE | 0x0002)
#define CTRL_MAX_REG   (CTRL_REG_BASE | 0x0003)
#define CTRL_STEP_REG  (CTRL_REG_BASE | 0x0004)
#define CTRL_DEF_REG   (CTRL_REG_BASE | 0x0005)
#define CTRL_VALUE_REG (CTRL_REG_BASE | 0x0006)

#define NO_DATA_AVAILABLE 0xFFFFFFFEU

/** True if the driver has read this register since boot. */
bool pivariety_emul_was_read(uint16_t reg);

/** Last value the driver wrote to this register, or NO_DATA_AVAILABLE. */
uint32_t pivariety_emul_last_write(uint16_t reg);

#endif /* PIVARIETY_EMUL_H_ */

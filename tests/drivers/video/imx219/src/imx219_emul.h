/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IMX219_EMUL_H_
#define IMX219_EMUL_H_

#include <stdint.h>

#define IMX219_EMUL_MODE_SELECT       0x0100
#define IMX219_EMUL_SOFTWARE_RESET    0x0103
#define IMX219_EMUL_EXCK_FREQ         0x012a
#define IMX219_EMUL_FRM_LENGTH_A      0x0160
#define IMX219_EMUL_LINE_LENGTH_A     0x0162
#define IMX219_EMUL_X_ADD_STA_A       0x0164
#define IMX219_EMUL_X_ADD_END_A       0x0166
#define IMX219_EMUL_Y_ADD_STA_A       0x0168
#define IMX219_EMUL_Y_ADD_END_A       0x016a
#define IMX219_EMUL_X_OUTPUT_SIZE     0x016c
#define IMX219_EMUL_Y_OUTPUT_SIZE     0x016e
#define IMX219_EMUL_BINNING_MODE_H    0x0174
#define IMX219_EMUL_BINNING_MODE_V    0x0175
#define IMX219_EMUL_CSI_DATA_FORMAT_A 0x018c
#define IMX219_EMUL_OPPXCK_DIV        0x0309
#define IMX219_EMUL_PLL_OP_MPY        0x030c

/** Current value of an 8-bit register. */
uint8_t imx219_emul_reg8(uint16_t addr);

/** Current value of a 16-bit register pair, high byte at @p addr as on the sensor. */
uint16_t imx219_emul_reg16(uint16_t addr);

#endif /* IMX219_EMUL_H_ */

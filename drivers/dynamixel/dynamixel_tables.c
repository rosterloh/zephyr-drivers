/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/util.h>

#include "dynamixel_internal.h"

/* Single source of truth for the X-series control table.
 * Addresses and lengths from the XL330 / XC330 e-Manual.
 */
static const struct dxl_control_info table[] = {
	[MODEL_NUMBER]            = { 0,   2 },
	[MODEL_INFORMATION]       = { 2,   4 },
	[FIRMWARE_VERSION]        = { 6,   1 },
	[ID]                      = { 7,   1 },
	[BAUD_RATE]               = { 8,   1 },
	[RETURN_DELAY_TIME]       = { 9,   1 },
	[DRIVE_MODE]              = { 10,  1 },
	[OPERATING_MODE]          = { 11,  1 },
	[SECONDARY_ID]            = { 12,  1 },
	[PROTOCOL_VERSION]        = { 13,  1 },
	[HOMING_OFFSET]           = { 20,  4 },
	[MOVING_THRESHOLD]        = { 24,  4 },
	[TEMPERATURE_LIMIT]       = { 31,  1 },
	[MAX_VOLTAGE_LIMIT]       = { 32,  2 },
	[MIN_VOLTAGE_LIMIT]       = { 34,  2 },
	[PWM_LIMIT]               = { 36,  2 },
	[CURRENT_LIMIT]           = { 38,  2 },
	[VELOCITY_LIMIT]          = { 44,  4 },
	[MAX_POSITION_LIMIT]      = { 48,  4 },
	[MIN_POSITION_LIMIT]      = { 52,  4 },
	[STARTUP_CONFIGURATION]   = { 60,  1 },
	[PWM_SLOPE]               = { 62,  1 },
	[SHUTDOWN]                = { 63,  1 },

	[TORQUE_ENABLE]           = { 64,  1 },
	[LED]                     = { 65,  1 },
	[STATUS_RETURN_LEVEL]     = { 68,  1 },
	[REGISTERED_INSTRUCTION]  = { 69,  1 },
	[HARDWARE_ERROR_STATUS]   = { 70,  1 },
	[VELOCITY_I_GAIN]         = { 76,  2 },
	[VELOCITY_P_GAIN]         = { 78,  2 },
	[POSITION_D_GAIN]         = { 80,  2 },
	[POSITION_I_GAIN]         = { 82,  2 },
	[POSITION_P_GAIN]         = { 84,  2 },
	[FEEDFORWARD_2ND_GAIN]    = { 88,  2 },
	[FEEDFORWARD_1ST_GAIN]    = { 90,  2 },
	[BUS_WATCHDOG]            = { 98,  1 },
	[GOAL_PWM]                = { 100, 2 },
	[GOAL_CURRENT]            = { 102, 2 },
	[GOAL_VELOCITY]           = { 104, 4 },
	[PROFILE_ACCELERATION]    = { 108, 4 },
	[PROFILE_VELOCITY]        = { 112, 4 },
	[GOAL_POSITION]           = { 116, 4 },
	[REALTIME_TICK]           = { 120, 2 },
	[MOVING]                  = { 122, 1 },
	[MOVING_STATUS]           = { 123, 1 },
	[PRESENT_PWM]             = { 124, 2 },
	[PRESENT_CURRENT]         = { 126, 2 },
	[PRESENT_VELOCITY]        = { 128, 4 },
	[PRESENT_POSITION]        = { 132, 4 },
	[VELOCITY_TRAJECTORY]     = { 136, 4 },
	[POSITION_TRAJECTORY]     = { 140, 4 },
	[PRESENT_INPUT_VOLTAGE]   = { 144, 2 },
	[PRESENT_TEMPERATURE]     = { 146, 1 },
};

int dxl_table_lookup(enum dxl_control item, uint16_t *addr, uint8_t *length)
{
	if ((unsigned)item >= ARRAY_SIZE(table)) {
		return -EINVAL;
	}
	if (table[item].length == 0) {
		return -EINVAL;
	}
	if (addr) {
		*addr = table[item].address;
	}
	if (length) {
		*length = table[item].length;
	}
	return 0;
}

const struct dxl_model_info dxl_info_x330 = {
	0.229f, 0, 2048, 4096, -3.14159265f, 3.14159265f
};

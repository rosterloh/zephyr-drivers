/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C emulator for an Arducam Pivariety module.
 *
 * Models the index/attribute enumeration protocol rather than a flat register
 * file: writing PIXFORMAT_INDEX_REG or RESOLUTION_INDEX_REG changes what the
 * corresponding attribute registers read back, and out-of-range indices return
 * NO_DATA_AVAILABLE. That behaviour is the thing worth testing; a flat register
 * file would let a broken enumeration loop pass.
 *
 * Advertised module: one greyscale RAW10 format (the shape the ToF camera is
 * expected to report) at two resolutions, plus one Bayer RAW8 format, so the
 * fourcc mapping is exercised on both paths.
 */

#define DT_DRV_COMPAT arducam_pivariety

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include "pivariety_emul.h"

#define EMUL_MAX_TRACKED_REG 0x0500

struct piv_emul_fmt {
	uint32_t data_type;
	uint32_t order;
	uint32_t lanes;
	uint32_t res[2][2]; /* {width, height} */
	uint32_t num_res;
};

static const struct piv_emul_fmt emul_fmts[] = {
	{.data_type = 0x2b, .order = 4, .lanes = 2, .res = {{240, 180}, {120, 90}}, .num_res = 2},
	{.data_type = 0x2a, .order = 0, .lanes = 2, .res = {{640, 480}, {0, 0}}, .num_res = 1},
};

static uint32_t emul_pixfmt_index;
static uint32_t emul_res_index;
static uint32_t emul_mode_select;
static bool emul_read_seen[EMUL_MAX_TRACKED_REG];
static uint32_t emul_last_write[EMUL_MAX_TRACKED_REG];

/* One mappable control (exposure) and one with no Zephyr equivalent, so the
 * skip path is exercised too. 0x00980911 is V4L2_CID_EXPOSURE.
 */
struct piv_emul_ctrl {
	uint32_t id, min, max, step, def;
};

static const struct piv_emul_ctrl emul_ctrls[] = {
	{.id = 0x00980911, .min = 0, .max = 65535, .step = 1, .def = 100},
	{.id = 0x009a0999, .min = 0, .max = 1, .step = 1, .def = 0},
};

static uint32_t emul_ctrl_index;
static uint32_t emul_ctrl_value;

bool pivariety_emul_was_read(uint16_t reg)
{
	return reg < EMUL_MAX_TRACKED_REG && emul_read_seen[reg];
}

uint32_t pivariety_emul_last_write(uint16_t reg)
{
	return reg < EMUL_MAX_TRACKED_REG ? emul_last_write[reg] : NO_DATA_AVAILABLE;
}

static uint32_t emul_read_reg(uint16_t reg)
{
	const struct piv_emul_fmt *fmt = NULL;

	if (reg < EMUL_MAX_TRACKED_REG) {
		emul_read_seen[reg] = true;
	}

	if (emul_pixfmt_index < ARRAY_SIZE(emul_fmts)) {
		fmt = &emul_fmts[emul_pixfmt_index];
	}

	switch (reg) {
	case SYSTEM_IDLE_REG:
		/* 0 == free. Confirmed against real hardware, which reports 0 at rest,
		 * and against wait_for_free() in the kernel driver.
		 */
		return 0;
	case DEVICE_VERSION_REG:
		return 0x0001;
	case SENSOR_ID_REG:
		return 0x1234;
	case DEVICE_ID_REG:
		return 0x0001;
	case MODE_SELECT_REG:
		return emul_mode_select;
	case PIXFORMAT_TYPE_REG:
		return fmt ? fmt->data_type : NO_DATA_AVAILABLE;
	case PIXFORMAT_ORDER_REG:
		return fmt ? fmt->order : NO_DATA_AVAILABLE;
	case MIPI_LANES_REG:
		return fmt ? fmt->lanes : NO_DATA_AVAILABLE;
	case FORMAT_WIDTH_REG:
		if (fmt == NULL || emul_res_index >= fmt->num_res) {
			return NO_DATA_AVAILABLE;
		}
		return fmt->res[emul_res_index][0];
	case FORMAT_HEIGHT_REG:
		if (fmt == NULL || emul_res_index >= fmt->num_res) {
			return NO_DATA_AVAILABLE;
		}
		return fmt->res[emul_res_index][1];
	case CTRL_ID_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls) ? emul_ctrls[emul_ctrl_index].id
								: NO_DATA_AVAILABLE;
	case CTRL_MIN_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls) ? emul_ctrls[emul_ctrl_index].min
								: NO_DATA_AVAILABLE;
	case CTRL_MAX_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls) ? emul_ctrls[emul_ctrl_index].max
								: NO_DATA_AVAILABLE;
	case CTRL_STEP_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls) ? emul_ctrls[emul_ctrl_index].step
								: NO_DATA_AVAILABLE;
	case CTRL_DEF_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls) ? emul_ctrls[emul_ctrl_index].def
								: NO_DATA_AVAILABLE;
	case CTRL_VALUE_REG:
		return emul_ctrl_value;
	default:
		return NO_DATA_AVAILABLE;
	}
}

static void emul_write_reg(uint16_t reg, uint32_t val)
{
	if (reg < EMUL_MAX_TRACKED_REG) {
		emul_last_write[reg] = val;
	}

	switch (reg) {
	case PIXFORMAT_INDEX_REG:
		emul_pixfmt_index = val;
		break;
	case RESOLUTION_INDEX_REG:
		emul_res_index = val;
		break;
	case MODE_SELECT_REG:
		emul_mode_select = val;
		break;
	case CTRL_INDEX_REG:
		emul_ctrl_index = val;
		break;
	case CTRL_VALUE_REG:
		emul_ctrl_value = val;
		break;
	default:
		break;
	}
}

static int piv_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
			     int addr)
{
	ARG_UNUSED(target);
	ARG_UNUSED(addr);

	if (num_msgs == 1 && msgs[0].len == 6 && !(msgs[0].flags & I2C_MSG_READ)) {
		/* Write: 2-byte BE address followed by 4-byte BE value. */
		emul_write_reg(sys_get_be16(&msgs[0].buf[0]), sys_get_be32(&msgs[0].buf[2]));
		return 0;
	}

	if (num_msgs == 2 && msgs[0].len == 2 && !(msgs[0].flags & I2C_MSG_READ) &&
	    msgs[1].len == 4 && (msgs[1].flags & I2C_MSG_READ)) {
		sys_put_be32(emul_read_reg(sys_get_be16(&msgs[0].buf[0])), msgs[1].buf);
		return 0;
	}

	return -EIO;
}

static const struct i2c_emul_api piv_emul_api = {
	.transfer = piv_emul_transfer,
};

static int piv_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);
	return 0;
}

#define PIV_EMUL_DEFINE(n) EMUL_DT_INST_DEFINE(n, piv_emul_init, NULL, NULL, &piv_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(PIV_EMUL_DEFINE)

/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C emulator for a TI INA3221: 16-bit big-endian registers behind 8-bit
 * addresses. Writing a measurement mode to CONFIG counts a conversion and
 * raises the conversion-ready flag, which reading MASK_ENABLE clears, as on
 * the device. Measurement registers hold whatever the test put there.
 */

#define DT_DRV_COMPAT ti_ina3221

#include <string.h>

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include "ina3221.h"
#include "ina3221_emul.h"

#define INA3221_CONFIG_POR 0x7127

static uint16_t emul_regs[256];
static uint32_t emul_conversions;

void ina3221_emul_set_reg(uint8_t reg, uint16_t val)
{
	emul_regs[reg] = val;
}

uint32_t ina3221_emul_conversions(void)
{
	return emul_conversions;
}

static void emul_write(uint8_t reg, uint16_t val)
{
	if (reg == INA3221_CONFIG) {
		if (val & INA3221_CONFIG_RST) {
			emul_regs[INA3221_CONFIG] = INA3221_CONFIG_POR;
			return;
		}
		if (val & (INA3221_CONFIG_BUS | INA3221_CONFIG_SHUNT)) {
			emul_conversions++;
			emul_regs[INA3221_MASK_ENABLE] |= INA3221_MASK_ENABLE_CONVERSION_READY;
		}
	}
	emul_regs[reg] = val;
}

static uint16_t emul_read(uint8_t reg)
{
	uint16_t val = emul_regs[reg];

	if (reg == INA3221_MASK_ENABLE) {
		emul_regs[reg] &= ~INA3221_MASK_ENABLE_CONVERSION_READY;
	}

	return val;
}

static int ina3221_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				 int addr)
{
	ARG_UNUSED(target);
	ARG_UNUSED(addr);

	if (num_msgs == 1 && !(msgs[0].flags & I2C_MSG_READ) && msgs[0].len == 3) {
		emul_write(msgs[0].buf[0], sys_get_be16(&msgs[0].buf[1]));
		return 0;
	}

	if (num_msgs == 2 && !(msgs[0].flags & I2C_MSG_READ) && msgs[0].len == 1 &&
	    (msgs[1].flags & I2C_MSG_READ) && msgs[1].len == 2) {
		sys_put_be16(emul_read(msgs[0].buf[0]), msgs[1].buf);
		return 0;
	}

	return -EIO;
}

static const struct i2c_emul_api ina3221_emul_api = {
	.transfer = ina3221_emul_transfer,
};

static int ina3221_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	memset(emul_regs, 0, sizeof(emul_regs));
	emul_regs[INA3221_CONFIG] = INA3221_CONFIG_POR;
	emul_regs[INA3221_MANUF_ID] = INA3221_MANUF_ID_VALUE;
	emul_regs[INA3221_CHIP_ID] = INA3221_CHIP_ID_VALUE;

	return 0;
}

#define INA3221_EMUL_DEFINE(n)                                                                     \
	EMUL_DT_INST_DEFINE(n, ina3221_emul_init, NULL, NULL, &ina3221_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(INA3221_EMUL_DEFINE)

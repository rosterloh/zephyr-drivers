/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C emulator for a Sony IMX219.
 *
 * A flat byte-wide register file behind 16-bit CCI addresses. The driver goes
 * through video_write_cci_reg(), which splits every 16-bit register into two
 * single-byte writes at consecutive addresses, so storing bytes and reading
 * pairs back big-endian is exactly what the sensor does. Burst transfers
 * auto-increment the address the same way.
 *
 * The only behaviour beyond storage is CHIP_ID, which reads 0x0219 so probe
 * passes, and SOFTWARE_RESET, which clears everything else.
 */

#define DT_DRV_COMPAT sony_imx219

#include <string.h>

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include "imx219_emul.h"

#define IMX219_EMUL_ADDR_SIZE 2

static uint8_t emul_regs[UINT16_MAX + 1];

static void emul_reset(void)
{
	memset(emul_regs, 0, sizeof(emul_regs));
	sys_put_be16(0x0219, &emul_regs[0x0000]);
}

uint8_t imx219_emul_reg8(uint16_t addr)
{
	return emul_regs[addr];
}

uint16_t imx219_emul_reg16(uint16_t addr)
{
	return ((uint16_t)emul_regs[addr] << 8) | emul_regs[(uint16_t)(addr + 1)];
}

static void emul_write(uint16_t addr, const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++, addr++) {
		if (addr == IMX219_EMUL_SOFTWARE_RESET && (buf[i] & BIT(0))) {
			emul_reset();
			continue;
		}
		emul_regs[addr] = buf[i];
	}
}

static int imx219_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				int addr)
{
	ARG_UNUSED(target);
	ARG_UNUSED(addr);

	if (num_msgs < 1 || (msgs[0].flags & I2C_MSG_READ) || msgs[0].len < IMX219_EMUL_ADDR_SIZE) {
		return -EIO;
	}

	uint16_t reg = sys_get_be16(msgs[0].buf);

	if (num_msgs == 1) {
		emul_write(reg, &msgs[0].buf[IMX219_EMUL_ADDR_SIZE],
			   msgs[0].len - IMX219_EMUL_ADDR_SIZE);
		return 0;
	}

	if (num_msgs == 2 && msgs[0].len == IMX219_EMUL_ADDR_SIZE &&
	    (msgs[1].flags & I2C_MSG_READ)) {
		for (size_t i = 0; i < msgs[1].len; i++) {
			msgs[1].buf[i] = emul_regs[(uint16_t)(reg + i)];
		}
		return 0;
	}

	return -EIO;
}

static const struct i2c_emul_api imx219_emul_api = {
	.transfer = imx219_emul_transfer,
};

static int imx219_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	emul_reset();

	return 0;
}

#define IMX219_EMUL_DEFINE(n)                                                                      \
	EMUL_DT_INST_DEFINE(n, imx219_emul_init, NULL, NULL, &imx219_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(IMX219_EMUL_DEFINE)

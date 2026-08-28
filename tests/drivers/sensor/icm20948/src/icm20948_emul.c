/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulator for the TDK InvenSense ICM-20948 I2C IMU.
 *
 * The point of interest is the banked register map: the same address means
 * different things depending on the last write to REG_BANK_SEL, so this keeps
 * a register file per bank and routes every access through the currently
 * selected one. A driver that forgets to select a bank reads plausible
 * garbage on real hardware; here it reads a different bank's zeros, and the
 * test notices.
 */
#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <icm20948.h>
#include <icm20948_emul.h>

LOG_MODULE_REGISTER(ICM20948_EMUL, CONFIG_SENSOR_LOG_LEVEL);

struct icm20948_emul_cfg {
	uint16_t addr;
};

struct icm20948_emul_data {
	uint8_t reg[ICM20948_EMUL_BANKS][ICM20948_EMUL_REGS];
	uint8_t bank;
};

void icm20948_emul_reset(const struct emul *target)
{
	struct icm20948_emul_data *data = target->data;

	memset(data->reg, 0, sizeof(data->reg));
	data->bank = 0;
	data->reg[0][ICM20948_REG_WHO_AM_I] = ICM20948_WHO_AM_I_VAL;
}

void icm20948_emul_set_reg(const struct emul *target, uint8_t bank, uint8_t reg, uint8_t val)
{
	struct icm20948_emul_data *data = target->data;

	__ASSERT_NO_MSG(bank < ICM20948_EMUL_BANKS && reg < ICM20948_EMUL_REGS);
	data->reg[bank][reg] = val;
}

uint8_t icm20948_emul_get_reg(const struct emul *target, uint8_t bank, uint8_t reg)
{
	struct icm20948_emul_data *data = target->data;

	__ASSERT_NO_MSG(bank < ICM20948_EMUL_BANKS && reg < ICM20948_EMUL_REGS);
	return data->reg[bank][reg];
}

void icm20948_emul_set_sample(const struct emul *target, uint8_t reg, int16_t val)
{
	struct icm20948_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg + 1 < ICM20948_EMUL_REGS);
	sys_put_be16((uint16_t)val, &data->reg[0][reg]);
}

uint8_t icm20948_emul_current_bank(const struct emul *target)
{
	struct icm20948_emul_data *data = target->data;

	return data->bank;
}

static void icm20948_emul_write(const struct emul *target, uint8_t reg, uint8_t val)
{
	struct icm20948_emul_data *data = target->data;

	if (reg == ICM20948_REG_BANK_SEL) {
		data->bank = val >> ICM20948_BANK_SHIFT;
		return;
	}

	data->reg[data->bank][reg] = val;

	/* A device reset clears the register file and drops the bank selection
	 * back to 0, which is why the driver re-selects it afterwards rather
	 * than assuming its earlier write survived. */
	if (reg == ICM20948_REG_PWR_MGMT_1 && (val & ICM20948_PWR_MGMT_1_RESET)) {
		icm20948_emul_reset(target);
	}
}

static int icm20948_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				      int addr)
{
	struct icm20948_emul_data *data = target->data;

	i2c_dump_msgs_rw(target->dev, msgs, num_msgs, addr, false);

	if (num_msgs < 1 || msgs[0].len < 1 || (msgs[0].flags & I2C_MSG_READ)) {
		LOG_ERR("expected a leading register write, got %d msg(s)", num_msgs);
		return -EIO;
	}

	uint8_t reg = msgs[0].buf[0];

	if (num_msgs == 1) {
		/* Register write: [reg, val...] in one message. */
		for (uint32_t i = 1; i < msgs[0].len; i++) {
			if (reg + i - 1 >= ICM20948_EMUL_REGS) {
				return -EIO;
			}
			icm20948_emul_write(target, reg + i - 1, msgs[0].buf[i]);
		}
		return 0;
	}

	if (!(msgs[1].flags & I2C_MSG_READ)) {
		LOG_ERR("expected msg1 to be a read");
		return -EIO;
	}

	/* Register read, possibly a burst. */
	for (uint32_t i = 0; i < msgs[1].len; i++) {
		if (reg + i >= ICM20948_EMUL_REGS) {
			return -EIO;
		}
		msgs[1].buf[i] = data->reg[data->bank][reg + i];
	}

	return 0;
}

static int icm20948_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);
	icm20948_emul_reset(target);

	return 0;
}

static const struct i2c_emul_api icm20948_emul_api_i2c = {
	.transfer = icm20948_emul_transfer_i2c,
};

#define ICM20948_EMUL(n)                                                                           \
	static const struct icm20948_emul_cfg icm20948_emul_cfg_##n = {                            \
		.addr = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	static struct icm20948_emul_data icm20948_emul_data_##n;                                   \
	EMUL_DT_INST_DEFINE(n, icm20948_emul_init, &icm20948_emul_data_##n,                        \
			    &icm20948_emul_cfg_##n, &icm20948_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(ICM20948_EMUL)

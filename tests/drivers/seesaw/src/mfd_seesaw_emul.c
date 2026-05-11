/*
 * Emulator for the Adafruit SeeSaw MFD parent device.
 *
 * The MFD driver (mfd_seesaw.c) performs I2C reads as two separate
 * transactions: a write (register address) followed by a read (data).
 * This emul therefore stores the last-addressed register and serves
 * the value on the subsequent standalone read, unlike the legacy
 * seesaw_emul.c which expects a combined write+read (num_msgs == 2).
 */
#define DT_DRV_COMPAT adafruit_seesaw_mfd

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "mfd_seesaw_regs.h"
#include "seesaw_emul.h"

LOG_MODULE_REGISTER(mfd_seesaw_emul, CONFIG_MFD_ADAFRUIT_SEESAW_LOG_LEVEL);

/* Register ID, size, and value table */
struct mfd_seesaw_reg {
	uint8_t module;
	uint8_t function;
	uint8_t bytes;
	uint32_t value;
};

/* Emulator configuration passed into driver instance */
struct mfd_seesaw_emul_cfg {
	uint16_t addr;
};

struct mfd_seesaw_emul_data {
	struct mfd_seesaw_reg regs[SEESAW_REGISTER_COUNT];
	/* Last register address written (mod<<8 | func), for split write+read */
	int last_reg;
};

static struct mfd_seesaw_reg *mfd_get_register(struct mfd_seesaw_emul_data *data, int reg)
{
	uint8_t mod = reg >> 8;
	uint8_t func = reg & 0xFF;

	for (int i = 0; i < SEESAW_REGISTER_COUNT; i++) {
		if ((data->regs[i].module == mod) && (data->regs[i].function == func)) {
			return &data->regs[i];
		}
	}
	return NULL;
}

int mfd_seesaw_mock_set_register(void *data_ptr, int reg, uint32_t value)
{
	struct mfd_seesaw_reg *reg_ptr = mfd_get_register(data_ptr, reg);

	if (reg_ptr == NULL) {
		return -EINVAL;
	}

	reg_ptr->value = value;
	return 0;
}

int mfd_seesaw_mock_get_register(void *data_ptr, int reg, uint32_t *value_ptr)
{
	struct mfd_seesaw_reg *reg_ptr = mfd_get_register(data_ptr, reg);

	if (reg_ptr == NULL || value_ptr == NULL) {
		return -EINVAL;
	}

	*value_ptr = reg_ptr->value;
	return 0;
}

static int mfd_seesaw_emul_transfer_i2c(const struct emul *target, struct i2c_msg msgs[],
					int num_msgs, int addr)
{
	struct mfd_seesaw_emul_data *data = (struct mfd_seesaw_emul_data *)target->data;

	ARG_UNUSED(addr);

	if (num_msgs == 1 && !(msgs[0].flags & I2C_MSG_READ)) {
		/* Write-only: either a register address setup or a data write. */
		if (msgs[0].len < 2) {
			LOG_ERR("Expected at least 2 bytes (module + function)");
			return -EIO;
		}

		int reg = ((int)msgs[0].buf[0] << 8) | msgs[0].buf[1];
		uint8_t payload_len = msgs[0].len - 2;

		if (payload_len == 0) {
			/* Pure address write — record last_reg for the upcoming read. */
			data->last_reg = reg;
			LOG_DBG("Address write reg %04x", reg);
			return 0;
		}

		/* Data write */
		struct mfd_seesaw_reg *reg_ptr = mfd_get_register(data, reg);

		if (!reg_ptr) {
			LOG_ERR("Invalid register: %04x", reg);
			return -EIO;
		}
		switch (payload_len) {
		case 1:
			reg_ptr->value = (uint32_t)msgs[0].buf[2];
			break;
		case 2:
			reg_ptr->value = (uint32_t)sys_get_be16(&msgs[0].buf[2]);
			break;
		case 3:
			/* 3-byte payload: buf[2] is a sub-channel selector (e.g. pin number
			 * for TIMER_PWM / TIMER_FREQ), buf[3..4] is the 16-bit value.
			 */
			reg_ptr->value = (uint32_t)sys_get_be16(&msgs[0].buf[3]);
			break;
		case 4:
			reg_ptr->value = sys_get_be32(&msgs[0].buf[2]);
			break;
		default:
			LOG_WRN("Unhandled write length %d for reg %04x", payload_len, reg);
			break;
		}
		LOG_DBG("Write reg %04x: %08x", reg, reg_ptr->value);
		return 0;
	}

	if (num_msgs == 1 && (msgs[0].flags & I2C_MSG_READ)) {
		/* Standalone read — serve from last_reg. */
		struct mfd_seesaw_reg *reg_ptr = mfd_get_register(data, data->last_reg);

		if (!reg_ptr) {
			LOG_ERR("Read without prior address write, or invalid reg %04x",
				data->last_reg);
			return -EIO;
		}

		if (msgs[0].len == 1) {
			msgs[0].buf[0] = reg_ptr->value & 0xFF;
			LOG_DBG("Read8 reg %04x: %02x", data->last_reg, reg_ptr->value);
		} else if (msgs[0].len == 2) {
			sys_put_be16(reg_ptr->value, msgs[0].buf);
			LOG_DBG("Read16 reg %04x: %04x", data->last_reg, reg_ptr->value);
		} else if (msgs[0].len == 4) {
			sys_put_be32(reg_ptr->value, msgs[0].buf);
			LOG_DBG("Read32 reg %04x: %08x", data->last_reg, reg_ptr->value);
		} else {
			/* Arbitrary-length read — fill byte-by-byte from the 32-bit value. */
			uint32_t v = reg_ptr->value;

			for (int i = 0; i < msgs[0].len && i < 4; i++) {
				msgs[0].buf[i] = (v >> (8 * (msgs[0].len - 1 - i))) & 0xFF;
			}
			LOG_DBG("Read%d reg %04x: %08x", msgs[0].len, data->last_reg, v);
		}
		return 0;
	}

	LOG_ERR("Unexpected num_msgs=%d", num_msgs);
	return -EIO;
}

static int mfd_seesaw_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	return 0;
}

static const struct i2c_emul_api mfd_seesaw_emul_api_i2c = {
	.transfer = mfd_seesaw_emul_transfer_i2c,
};

#define MFD_SEESAW_EMUL(n)                                                                         \
	static const struct mfd_seesaw_emul_cfg mfd_seesaw_emul_cfg_##n = {                        \
		.addr = DT_INST_REG_ADDR(n)};                                                      \
	static struct mfd_seesaw_emul_data mfd_seesaw_emul_data_##n = {                            \
		.regs =                                                                            \
			{                                                                          \
				{SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, 1,                       \
				 SEESAW_HW_ID_CODE_SAMD09},                                        \
				{SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION, 4, 0x270FD10F},        \
				{SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS, 4, 0xFFF07},           \
				{SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 1, 0},                   \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, 4, 0},                 \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, 4, 0},                 \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, 4, 0},                        \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, 4, 0},                    \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, 4, 0},                    \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_TOGGLE, 4, 0},                 \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_INTENSET, 4, 0},                    \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_INTENCLR, 4, 0},                    \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_INTFLAG, 4, 0},                     \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, 4, 0},                   \
				{SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENCLR, 4, 0},                   \
				/* ADC: SAMD09 logical channel 3 -> pin 5 */                        \
				{SEESAW_ADC_BASE, SEESAW_ADC_CHANNEL_OFFSET + 5, 2, 0},             \
				/* PWM timer: freq and duty per-pin writes */                        \
				{SEESAW_TIMER_BASE, SEESAW_TIMER_FREQ, 3, 0},                       \
				{SEESAW_TIMER_BASE, SEESAW_TIMER_PWM, 3, 0},                        \
				/* NeoPixel init registers */                                        \
				{SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SPEED, 1, 0},                \
				{SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN, 1, 0},                  \
				{SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH, 2, 0},           \
				{SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF, 2, 0},                  \
				{SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, 4, 0},               \
			},                                                                         \
		.last_reg = -1,                                                                    \
	};                                                                                         \
	EMUL_DT_INST_DEFINE(n, mfd_seesaw_emul_init, &mfd_seesaw_emul_data_##n,                    \
			    &mfd_seesaw_emul_cfg_##n, &mfd_seesaw_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(MFD_SEESAW_EMUL)

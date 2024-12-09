/*
 * Emulator for the AMS AS7341 I2C spectral sensor
 */
#define DT_DRV_COMPAT ams_as7341

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/emul_sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <as7341.h>
#include <as7341_emul.h>

LOG_MODULE_REGISTER(AS7341_EMUL, CONFIG_SENSOR_LOG_LEVEL);

#define NUM_REGS AS7341_REG_FDATA

/* Emulator configuration passed into driver instance */
struct as7341_emul_cfg {
	uint16_t addr;
};

struct as7341_emul_data {
	uint8_t reg[NUM_REGS];
};

void as7341_emul_set_reg(const struct emul *target, uint8_t reg_addr, const uint8_t *val,
			 size_t count)
{
	struct as7341_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg_addr + count < NUM_REGS);
	memcpy(data->reg + reg_addr, val, count);
}

void as7341_emul_get_reg(const struct emul *target, uint8_t reg_addr, uint8_t *val, size_t count)
{
	struct as7341_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg_addr + count < NUM_REGS);
	memcpy(val, data->reg + reg_addr, count);
}

void as7341_emul_reset(const struct emul *target)
{
	struct as7341_emul_data *data = target->data;

	memset(data->reg, 0, NUM_REGS);
	data->reg[AS7341_REG_ID] = 0x24;
}

static int as7341_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				    int addr)
{
	struct as7341_emul_data *data = (struct as7341_emul_data *)target->data;

	i2c_dump_msgs_rw(target->dev, msgs, num_msgs, addr, false);

	if (num_msgs < 1) {
		LOG_ERR("Invalid number of messages: %d", num_msgs);
		return -EIO;
	}
	if (FIELD_GET(I2C_MSG_READ, msgs->flags)) {
		LOG_ERR("Unexpected read");
		return -EIO;
	}
	if (msgs->len < 1) {
		LOG_ERR("Unexpected msg0 length %d", msgs->len);
		return -EIO;
	}

	uint8_t regn = msgs->buf[0];
	bool is_read = FIELD_GET(I2C_MSG_READ, msgs->flags) == 1;
	bool is_stop = FIELD_GET(I2C_MSG_STOP, msgs->flags) == 1;

	if (!is_stop && !is_read) {
		/* First message was a write with the register number, check next message */
		msgs++;
		is_read = FIELD_GET(I2C_MSG_READ, msgs->flags) == 1;
		is_stop = FIELD_GET(I2C_MSG_STOP, msgs->flags) == 1;
	}
	if (is_read) {
		/* Read data */
		for (int i = 0; i < msgs->len; ++i) {
			msgs->buf[i] = data->reg[regn + i];
		}
	} else {
		/* Write data */
		data->reg[regn] = msgs->buf[1];
	}

	return 0;
}

static int as7341_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);
	as7341_emul_reset(target);

	return 0;
}

static const struct i2c_emul_api as7341_emul_api_i2c = {
	.transfer = as7341_emul_transfer_i2c,
};

#define AS7341_EMUL(n)                                                                             \
	static const struct as7341_emul_cfg as7341_emul_cfg_##n = {.addr = DT_INST_REG_ADDR(n)};   \
	static struct as7341_emul_data as7341_emul_data_##n;                                       \
	EMUL_DT_INST_DEFINE(n, as7341_emul_init, &as7341_emul_data_##n, &as7341_emul_cfg_##n,      \
			    &as7341_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(AS7341_EMUL)
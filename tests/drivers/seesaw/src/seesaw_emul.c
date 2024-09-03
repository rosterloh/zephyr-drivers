/*
 * Emulator for the Adafruit SeeSaw device
 */
#define DT_DRV_COMPAT adafruit_seesaw

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <seesaw.h>
#include <seesaw_emul.h>

LOG_MODULE_REGISTER(seesaw_emul, CONFIG_SEESAW_LOG_LEVEL);

// https://github.com/adafruit/seesaw/blob/master/include/RegisterMap.h
/* Register ID, size, and value table */
struct seesaw_reg {
	uint8_t module;
	uint8_t function;
	uint8_t bytes;
	uint32_t value;
};

/* Emulator configuration passed into driver instance */
struct seesaw_emul_cfg {
	uint16_t addr;
};

struct seesaw_emul_data {
	struct seesaw_reg seesaw_regs[SEESAW_REGISTER_COUNT];
};

static struct seesaw_reg *get_register(struct seesaw_emul_data *data, int reg)
{
	uint8_t mod = reg >> 8;
	uint8_t func = reg & 0xFF;
	for (int i = 0; i < SEESAW_REGISTER_COUNT; i++) {
		if ((data->seesaw_regs[i].module == mod) &&
		    (data->seesaw_regs[i].function == func)) {
			return &data->seesaw_regs[i];
		}
	}
	return NULL;
}

int seesaw_mock_set_register(void *data_ptr, int reg, uint32_t value)
{
	struct seesaw_reg *reg_ptr = get_register(data_ptr, reg);

	if (reg_ptr == NULL) {
		return -EINVAL;
	}

	reg_ptr->value = value;
	return 0;
}

int seesaw_mock_get_register(void *data_ptr, int reg, uint32_t *value_ptr)
{
	struct seesaw_reg *reg_ptr = get_register(data_ptr, reg);

	if (reg_ptr == NULL || value_ptr == NULL) {
		return -EINVAL;
	}

	*value_ptr = reg_ptr->value;
	return 0;
}

static int seesaw_emul_transfer_i2c(const struct emul *target, struct i2c_msg msgs[], int num_msgs,
				    int addr)
{
	struct seesaw_emul_data *data = (struct seesaw_emul_data *)target->data;

	if (msgs[0].flags & I2C_MSG_READ) {
		LOG_ERR("Expected write");
		return -EIO;
	}

	if (num_msgs == 1) {
		/* seesaw_write */
		if (msgs[0].len < 3) {
			LOG_ERR("Expected at least 3 bytes");
			return -EIO;
		}

		int reg = sys_get_be16(&msgs[0].buf[0]);
		uint8_t len = msgs[0].len - 2;

		struct seesaw_reg *reg_ptr = get_register(data, reg);

		if (!reg_ptr) {
			LOG_ERR("Invalid register: %04x", reg);
			return -EIO;
		}

		if (len != reg_ptr->bytes) {
			LOG_ERR("Unexpected number of bytes to write: %02x", len);
			return -EIO;
		}

		switch (len) {
		case 1:
			reg_ptr->value = (uint32_t)msgs[0].buf[2];
			break;
		case 2:
			reg_ptr->value = (uint32_t)sys_get_be16(&msgs[0].buf[2]);
			break;
		case 4:
			reg_ptr->value = sys_get_be32(&msgs[0].buf[2]);
			break;
		default:
			break;
		}

		LOG_DBG("Write reg %02x: %04x", reg, reg_ptr->value);
	} else {
		/* seesaw_write */
		if ((msgs[1].flags & I2C_MSG_READ) == I2C_MSG_WRITE) {
			LOG_ERR("Expected read");
			return -EIO;
		}
		uint8_t reg = sys_get_be16(&msgs[0].buf[0]);

		struct seesaw_reg *reg_ptr = get_register(data, reg);

		if (!reg_ptr) {
			LOG_ERR("Invalid register: %04x", reg);
			return -EIO;
		}

		if (msgs[1].len == 1) {
			msgs[1].buf[0] = reg_ptr->value & 0x0F;
			LOG_DBG("Read8 reg %04x: %02x", reg, reg_ptr->value);
		} else if (msgs[1].len == 2) {
			sys_put_be16(reg_ptr->value, msgs[1].buf);
			LOG_DBG("Read16 reg %04x: %04x", reg, reg_ptr->value);
		} else if (msgs[1].len == 4) {
			sys_put_be32(reg_ptr->value, msgs[1].buf);
			LOG_DBG("Read32 reg %04x: %08x", reg, reg_ptr->value);
		} else {
			LOG_ERR("Invalid read length: %d", msgs[1].len);
			return -EIO;
		}
	}

	return 0;
}

static int seesaw_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	return 0;
}

static const struct i2c_emul_api seesaw_emul_api_i2c = {
	.transfer = seesaw_emul_transfer_i2c,
};

#define SEESAW_EMUL(n)                                                                             \
	static const struct seesaw_emul_cfg seesaw_emul_cfg_##n = {.addr = DT_INST_REG_ADDR(n)};   \
	static struct seesaw_emul_data seesaw_emul_data_##n = {                                    \
		.seesaw_regs = {                                                                   \
			{SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, 1, SEESAW_HW_ID_CODE_SAMD09},    \
			{SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION, 4, 0x270FD10F},                \
			{SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS, 4, 0xFFF07},                   \
			{SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 1, 0},                           \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, 4, 0},                         \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, 4, 0},                         \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, 4, 0},                                \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, 4, 0},                            \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, 4, 0},                            \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_TOGGLE, 4, 0},                         \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_INTENSET, 4, 0},                            \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_INTENCLR, 4, 0},                            \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_INTFLAG, 4, 0},                             \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, 4, 0},                           \
			{SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENCLR, 4, 0},                           \
		}};                                                                                \
	EMUL_DT_INST_DEFINE(n, seesaw_emul_init, &seesaw_emul_data_##n, &seesaw_emul_cfg_##n,      \
			    &seesaw_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(SEESAW_EMUL)

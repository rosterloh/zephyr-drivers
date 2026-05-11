#define DT_DRV_COMPAT adafruit_seesaw_mfd

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>

#include "mfd_seesaw_regs.h"

LOG_MODULE_REGISTER(mfd_seesaw, CONFIG_MFD_ADAFRUIT_SEESAW_LOG_LEVEL);

struct mfd_seesaw_config {
	struct i2c_dt_spec i2c;
	uint32_t read_delay_us;
#ifdef CONFIG_MFD_ADAFRUIT_SEESAW_INT
	struct gpio_dt_spec int_gpio;
#endif
};

struct mfd_seesaw_data {
	struct k_mutex lock;
	uint8_t hw_id;
	uint16_t pid;
	uint32_t options;
#ifdef CONFIG_MFD_ADAFRUIT_SEESAW_INT
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_work work;
	sys_slist_t int_callbacks;
#endif
};

int mfd_seesaw_read(const struct device *mfd, uint8_t mod, uint8_t reg, uint8_t *buf, size_t len,
		    uint32_t pre_read_delay_us)
{
	const struct mfd_seesaw_config *cfg = mfd->config;
	struct mfd_seesaw_data *data = mfd->data;
	uint8_t addr[2] = {mod, reg};
	int ret;

	(void)k_mutex_lock(&data->lock, K_FOREVER);
	ret = i2c_write_dt(&cfg->i2c, addr, sizeof(addr));
	if (ret == 0) {
		if (pre_read_delay_us == 0) {
			pre_read_delay_us = cfg->read_delay_us;
		}
		k_busy_wait(pre_read_delay_us);
		ret = i2c_read_dt(&cfg->i2c, buf, len);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int mfd_seesaw_write(const struct device *mfd, uint8_t mod, uint8_t reg, const uint8_t *buf,
		     size_t len)
{
	const struct mfd_seesaw_config *cfg = mfd->config;
	struct mfd_seesaw_data *data = mfd->data;
	/* Largest known write: NeoPixel BUF chunk = 2 (offset) + 30 (payload) = 32. */
	uint8_t tx[2 + 32];
	int ret;

	if (len > sizeof(tx) - 2) {
		return -EINVAL;
	}
	if (len > 0 && buf == NULL) {
		return -EINVAL;
	}

	tx[0] = mod;
	tx[1] = reg;
	if (len > 0) {
		memcpy(&tx[2], buf, len);
	}

	(void)k_mutex_lock(&data->lock, K_FOREVER);
	ret = i2c_write_dt(&cfg->i2c, tx, len + 2);
	k_mutex_unlock(&data->lock);
	return ret;
}

uint32_t mfd_seesaw_options(const struct device *mfd)
{
	const struct mfd_seesaw_data *data = mfd->data;
	return data->options;
}

uint8_t mfd_seesaw_hw_id(const struct device *mfd)
{
	const struct mfd_seesaw_data *data = mfd->data;
	return data->hw_id;
}

uint16_t mfd_seesaw_pid(const struct device *mfd)
{
	const struct mfd_seesaw_data *data = mfd->data;
	return data->pid;
}

static int seesaw_sw_reset(const struct device *mfd)
{
	uint8_t rst = 0xFFU;
	return mfd_seesaw_write(mfd, SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, &rst, sizeof(rst));
}

static int mfd_seesaw_probe(const struct device *mfd)
{
	struct mfd_seesaw_data *data = mfd->data;
	uint8_t buf[4];
	int ret;

	k_busy_wait(SEESAW_WAIT_STARTUP_US);
	ret = seesaw_sw_reset(mfd);
	if (ret < 0) {
		return ret;
	}
	k_busy_wait(SEESAW_WAIT_INITIAL_RESET_US);

	ret = mfd_seesaw_read(mfd, SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, &data->hw_id, 1, 0);
	if (ret < 0) {
		return ret;
	}

	switch (data->hw_id) {
	case SEESAW_HW_ID_CODE_SAMD09:
	case SEESAW_HW_ID_CODE_TINY806:
	case SEESAW_HW_ID_CODE_TINY807:
	case SEESAW_HW_ID_CODE_TINY816:
	case SEESAW_HW_ID_CODE_TINY817:
	case SEESAW_HW_ID_CODE_TINY1616:
	case SEESAW_HW_ID_CODE_TINY1617:
		break;
	default:
		LOG_ERR("Unknown HW ID 0x%02x", data->hw_id);
		return -ENODEV;
	}

	ret = mfd_seesaw_read(mfd, SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION, buf, sizeof(buf), 0);
	if (ret < 0) {
		return ret;
	}
	data->pid = sys_get_be32(buf) >> 16;

	ret = mfd_seesaw_read(mfd, SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS, buf, sizeof(buf), 0);
	if (ret < 0) {
		return ret;
	}
	data->options = sys_get_be32(buf);

	LOG_DBG("seesaw hw=0x%02x pid=%u options=0x%08x", data->hw_id, data->pid, data->options);
	return 0;
}

#ifdef CONFIG_MFD_ADAFRUIT_SEESAW_INT
extern int mfd_seesaw_init_interrupt(const struct device *mfd);
#endif

static int mfd_seesaw_init(const struct device *mfd)
{
	const struct mfd_seesaw_config *cfg = mfd->config;
	struct mfd_seesaw_data *data = mfd->data;
	int ret;

	k_mutex_init(&data->lock);

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	ret = mfd_seesaw_probe(mfd);
	if (ret < 0) {
		LOG_ERR("seesaw probe failed: %d", ret);
		return ret;
	}

#ifdef CONFIG_MFD_ADAFRUIT_SEESAW_INT
	if (cfg->int_gpio.port != NULL) {
		ret = mfd_seesaw_init_interrupt(mfd);
		if (ret < 0) {
			LOG_ERR("seesaw INT init failed: %d", ret);
			return ret;
		}
	}
#endif
	return 0;
}

#ifdef CONFIG_MFD_ADAFRUIT_SEESAW_INT
#define SEESAW_INT_GPIO_INIT(inst) .int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),
#else
#define SEESAW_INT_GPIO_INIT(inst)
#endif

#define MFD_SEESAW_DEFINE(inst)                                                                    \
	static struct mfd_seesaw_data mfd_seesaw_data_##inst;                                      \
	static const struct mfd_seesaw_config mfd_seesaw_cfg_##inst = {                            \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.read_delay_us = DT_INST_PROP(inst, read_delay_us),                                \
		SEESAW_INT_GPIO_INIT(inst)};                                                       \
	DEVICE_DT_INST_DEFINE(inst, mfd_seesaw_init, NULL, &mfd_seesaw_data_##inst,                \
			      &mfd_seesaw_cfg_##inst, POST_KERNEL,                                 \
			      CONFIG_MFD_ADAFRUIT_SEESAW_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MFD_SEESAW_DEFINE)

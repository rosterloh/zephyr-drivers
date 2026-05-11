#define DT_DRV_COMPAT adafruit_seesaw_encoder

#include <zephyr/device.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(seesaw_encoder, CONFIG_ADAFRUIT_SEESAW_ENCODER_LOG_LEVEL);

/* Local mirror of MFD register IDs. */
#define SEESAW_ENCODER_BASE     0x11U
#define SEESAW_ENCODER_POSITION 0x04U

struct seesaw_encoder_config {
	const struct device *mfd;
};

struct seesaw_encoder_data {
	int32_t position;
};

static int seesaw_encoder_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct seesaw_encoder_config *cfg = dev->config;
	struct seesaw_encoder_data *data = dev->data;
	uint8_t buf[4];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	ret = mfd_seesaw_read(cfg->mfd, SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, buf,
			      sizeof(buf), 0);
	if (ret < 0) {
		return ret;
	}

	data->position = (int32_t)sys_get_be32(buf);
	return 0;
}

static int seesaw_encoder_channel_get(const struct device *dev, enum sensor_channel chan,
				      struct sensor_value *val)
{
	const struct seesaw_encoder_data *data = dev->data;

	if (chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	val->val1 = data->position;
	val->val2 = 0;
	return 0;
}

static DEVICE_API(sensor, seesaw_encoder_api) = {
	.sample_fetch = seesaw_encoder_sample_fetch,
	.channel_get = seesaw_encoder_channel_get,
};

static int seesaw_encoder_init(const struct device *dev)
{
	const struct seesaw_encoder_config *cfg = dev->config;

	if (!device_is_ready(cfg->mfd)) {
		return -ENODEV;
	}
	return 0;
}

#define SEESAW_ENCODER_DEFINE(inst)                                                                \
	static struct seesaw_encoder_data seesaw_encoder_data_##inst;                              \
	static const struct seesaw_encoder_config seesaw_encoder_cfg_##inst = {                    \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, seesaw_encoder_init, NULL, &seesaw_encoder_data_##inst,        \
			      &seesaw_encoder_cfg_##inst, POST_KERNEL,                             \
			      CONFIG_SENSOR_INIT_PRIORITY, &seesaw_encoder_api);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_ENCODER_DEFINE)

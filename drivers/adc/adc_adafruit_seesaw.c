#define DT_DRV_COMPAT adafruit_seesaw_adc

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_seesaw, CONFIG_ADC_ADAFRUIT_SEESAW_LOG_LEVEL);

/* Local mirror of MFD register IDs. */
#define SEESAW_ADC_BASE           0x09U
#define SEESAW_ADC_CHANNEL_OFFSET 0x07U
#define SEESAW_HW_ID_CODE_SAMD09  0x55U

/* SAMD09 maps logical ADC inputs 0..3 to chip GPIO pins 2,3,4,5. */
static const uint8_t samd09_adc_pin_map[4] = {2U, 3U, 4U, 5U};

#define ADC_SEESAW_MAX_CHANNELS 32U

struct adc_seesaw_config {
	const struct device *mfd;
};

struct adc_seesaw_data {
	struct adc_context ctx;
	const struct device *self;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	struct k_thread thread;
	struct k_sem sem;

	K_KERNEL_STACK_MEMBER(stack, 512);
};

static int adc_seesaw_channel_setup(const struct device *dev, const struct adc_channel_cfg *cfg)
{
	ARG_UNUSED(dev);

	if (cfg->channel_id >= ADC_SEESAW_MAX_CHANNELS) {
		return -EINVAL;
	}
	if (cfg->differential) {
		return -ENOTSUP;
	}
	/* Seesaw firmware fixes gain, reference, and acquisition time. Accept
	 * the caller's values without applying them. */
	return 0;
}

static int adc_seesaw_read_one(const struct device *dev, uint8_t logical_chan, uint16_t *out)
{
	const struct adc_seesaw_config *cfg = dev->config;
	uint8_t pin;
	uint8_t buf[2];
	int ret;

	if (mfd_seesaw_hw_id(cfg->mfd) == SEESAW_HW_ID_CODE_SAMD09) {
		if (logical_chan >= ARRAY_SIZE(samd09_adc_pin_map)) {
			return -EINVAL;
		}
		pin = samd09_adc_pin_map[logical_chan];
	} else {
		pin = logical_chan;
	}

	/* Seesaw ADC conversion needs ~500 us after the channel-select write
	 * before the result register is valid.
	 */
	ret = mfd_seesaw_read(cfg->mfd, SEESAW_ADC_BASE, SEESAW_ADC_CHANNEL_OFFSET + pin, buf,
			      sizeof(buf), 500);
	if (ret == 0) {
		*out = sys_get_be16(buf);
	}
	return ret;
}

static int adc_seesaw_validate_sequence(const struct adc_sequence *seq)
{
	if (seq->resolution != 10) {
		return -EINVAL;
	}
	if (seq->channels == 0) {
		return -EINVAL;
	}
	if (seq->buffer == NULL) {
		return -EINVAL;
	}

	size_t needed = POPCOUNT(seq->channels) * sizeof(uint16_t);

	if (seq->buffer_size < needed) {
		return -ENOMEM;
	}
	return 0;
}

static int adc_seesaw_read(const struct device *dev, const struct adc_sequence *seq)
{
	struct adc_seesaw_data *data = dev->data;
	int ret;

	ret = adc_seesaw_validate_sequence(seq);
	if (ret < 0) {
		return ret;
	}

	adc_context_lock(&data->ctx, false, NULL);
	data->buffer = seq->buffer;
	data->channels = seq->channels;
	adc_context_start_read(&data->ctx, seq);
	ret = adc_context_wait_for_completion(&data->ctx);
	adc_context_release(&data->ctx, ret);
	return ret;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_seesaw_data *data = CONTAINER_OF(ctx, struct adc_seesaw_data, ctx);

	data->repeat_buffer = data->buffer;
	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_seesaw_data *data = CONTAINER_OF(ctx, struct adc_seesaw_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_seesaw_thread(void *p1, void *p2, void *p3)
{
	struct adc_seesaw_data *data = p1;
	const struct device *dev = data->self;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);

		uint32_t chans = data->channels;
		uint16_t *out = data->buffer;
		int ret = 0;

		for (uint8_t i = 0; i < ADC_SEESAW_MAX_CHANNELS && chans; i++) {
			if (!(chans & BIT(i))) {
				continue;
			}
			chans &= ~BIT(i);
			ret = adc_seesaw_read_one(dev, i, out);
			if (ret < 0) {
				LOG_ERR("read channel %u failed: %d", i, ret);
				break;
			}
			out++;
		}

		if (ret < 0) {
			adc_context_complete(&data->ctx, ret);
		} else {
			adc_context_on_sampling_done(&data->ctx, dev);
		}
	}
}

static int adc_seesaw_init(const struct device *dev)
{
	const struct adc_seesaw_config *cfg = dev->config;
	struct adc_seesaw_data *data = dev->data;

	if (!device_is_ready(cfg->mfd)) {
		return -ENODEV;
	}

	data->self = dev;
	k_sem_init(&data->sem, 0, 1);
	(void)k_thread_create(&data->thread, data->stack, K_KERNEL_STACK_SIZEOF(data->stack),
			      adc_seesaw_thread, data, NULL, NULL,
			      K_PRIO_COOP(CONFIG_ADC_ADAFRUIT_SEESAW_ACQUISITION_THREAD_PRIO), 0,
			      K_NO_WAIT);

	adc_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(adc, adc_seesaw_api) = {
	.channel_setup = adc_seesaw_channel_setup,
	.read = adc_seesaw_read,
	.ref_internal = 3300, /* seesaw firmware uses VCC ref ~3.3V */
};

#define ADC_SEESAW_DEFINE(inst)                                                                    \
	static struct adc_seesaw_data adc_seesaw_data_##inst = {                                   \
		ADC_CONTEXT_INIT_TIMER(adc_seesaw_data_##inst, ctx),                               \
		ADC_CONTEXT_INIT_LOCK(adc_seesaw_data_##inst, ctx),                                \
		ADC_CONTEXT_INIT_SYNC(adc_seesaw_data_##inst, ctx),                                \
	};                                                                                         \
	static const struct adc_seesaw_config adc_seesaw_cfg_##inst = {                            \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, adc_seesaw_init, NULL, &adc_seesaw_data_##inst,                \
			      &adc_seesaw_cfg_##inst, POST_KERNEL,                                 \
			      CONFIG_ADC_ADAFRUIT_SEESAW_INIT_PRIORITY, &adc_seesaw_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_SEESAW_DEFINE)

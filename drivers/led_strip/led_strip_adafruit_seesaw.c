#define DT_DRV_COMPAT adafruit_seesaw_neopixel

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(led_seesaw, CONFIG_LED_STRIP_ADAFRUIT_SEESAW_LOG_LEVEL);

/* Local mirror of MFD register IDs. */
#define SEESAW_NEOPIXEL_BASE       0x0EU
#define SEESAW_NEOPIXEL_PIN        0x01U
#define SEESAW_NEOPIXEL_SPEED      0x02U
#define SEESAW_NEOPIXEL_BUF_LENGTH 0x03U
#define SEESAW_NEOPIXEL_BUF        0x04U
#define SEESAW_NEOPIXEL_SHOW       0x05U

/* MFD I2C tx buffer is 2 (addr) + 32 (payload). For NeoPixel BUF writes we
 * need 2 (offset) + N (pixel bytes) within that 32-byte payload window,
 * giving 30 pixel bytes per chunk.
 */
#define NEOPIXEL_CHUNK_PAYLOAD 30U

struct led_strip_seesaw_config {
	const struct device *mfd;
	uint32_t chain_length;
	uint32_t frequency;
	uint8_t seesaw_pin;
	const uint8_t *color_mapping;
	uint8_t num_colors;
	uint8_t *pixel_buf;
	size_t pixel_buf_size;
};

static int led_strip_seesaw_update_rgb(const struct device *dev, struct led_rgb *pixels,
				       size_t count)
{
	const struct led_strip_seesaw_config *cfg = dev->config;
	const uint8_t bpp = cfg->num_colors;
	const size_t total = MIN(count, cfg->chain_length) * bpp;
	uint8_t txbuf[2 + NEOPIXEL_CHUNK_PAYLOAD];
	size_t off;
	int ret;

	if (total > cfg->pixel_buf_size) {
		return -EINVAL;
	}

	for (size_t i = 0; i < MIN(count, cfg->chain_length); i++) {
		uint8_t *out = &cfg->pixel_buf[i * bpp];

		for (uint8_t c = 0; c < bpp; c++) {
			switch (cfg->color_mapping[c]) {
			case LED_COLOR_ID_RED:
				out[c] = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				out[c] = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				out[c] = pixels[i].b;
				break;
			default:
				out[c] = 0;
				break;
			}
		}
	}

	for (off = 0; off < total; off += NEOPIXEL_CHUNK_PAYLOAD) {
		size_t chunk = MIN(NEOPIXEL_CHUNK_PAYLOAD, total - off);

		sys_put_be16((uint16_t)off, txbuf);
		memcpy(&txbuf[2], &cfg->pixel_buf[off], chunk);
		ret = mfd_seesaw_write(cfg->mfd, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF, txbuf,
				       2U + chunk);
		if (ret < 0) {
			return ret;
		}
	}

	return mfd_seesaw_write(cfg->mfd, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SHOW, NULL, 0U);
}

static size_t led_strip_seesaw_length(const struct device *dev)
{
	const struct led_strip_seesaw_config *cfg = dev->config;

	return cfg->chain_length;
}

static DEVICE_API(led_strip, led_strip_seesaw_api) = {
	.update_rgb = led_strip_seesaw_update_rgb,
	.length = led_strip_seesaw_length,
};

static int led_strip_seesaw_init(const struct device *dev)
{
	const struct led_strip_seesaw_config *cfg = dev->config;
	uint8_t speed = (cfg->frequency >= 800000U) ? 1U : 0U;
	uint8_t lenbuf[2];
	int ret;

	if (!device_is_ready(cfg->mfd)) {
		return -ENODEV;
	}

	ret = mfd_seesaw_write(cfg->mfd, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SPEED, &speed,
			       sizeof(speed));
	if (ret < 0) {
		return ret;
	}

	sys_put_be16((uint16_t)(cfg->chain_length * cfg->num_colors), lenbuf);
	ret = mfd_seesaw_write(cfg->mfd, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH, lenbuf,
			       sizeof(lenbuf));
	if (ret < 0) {
		return ret;
	}

	return mfd_seesaw_write(cfg->mfd, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN,
				&cfg->seesaw_pin, sizeof(cfg->seesaw_pin));
}

#define LED_STRIP_SEESAW_DEFINE(inst)                                                              \
	static const uint8_t led_strip_seesaw_color_mapping_##inst[] =                             \
		DT_INST_PROP(inst, color_mapping);                                                 \
	static uint8_t led_strip_seesaw_pixbuf_##inst[DT_INST_PROP(inst, pixel_buffer_size)];      \
	static const struct led_strip_seesaw_config led_strip_seesaw_cfg_##inst = {                \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
		.chain_length = DT_INST_PROP(inst, chain_length),                                  \
		.frequency = DT_INST_PROP(inst, frequency),                                        \
		.seesaw_pin = DT_INST_PROP(inst, seesaw_pin),                                      \
		.color_mapping = led_strip_seesaw_color_mapping_##inst,                            \
		.num_colors = ARRAY_SIZE(led_strip_seesaw_color_mapping_##inst),                   \
		.pixel_buf = led_strip_seesaw_pixbuf_##inst,                                       \
		.pixel_buf_size = sizeof(led_strip_seesaw_pixbuf_##inst),                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, led_strip_seesaw_init, NULL, NULL,                             \
			      &led_strip_seesaw_cfg_##inst, POST_KERNEL,                           \
			      CONFIG_LED_STRIP_ADAFRUIT_SEESAW_INIT_PRIORITY,                      \
			      &led_strip_seesaw_api);

DT_INST_FOREACH_STATUS_OKAY(LED_STRIP_SEESAW_DEFINE)

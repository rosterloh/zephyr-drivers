/*
 * Display driver that decodes each write back into black/white pixels.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vnd_od_capture_display

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/ztest.h>

#include "capture_display.h"

struct capture_state capture;

static bool pixel(const struct display_buffer_descriptor *desc, const uint8_t *buf, uint16_t x,
		  uint16_t y)
{
	bool msb = (capture.screen_info & SCREEN_INFO_MONO_MSB_FIRST) != 0U;
	bool inv = capture.fmt == PIXEL_FORMAT_MONO10;
	uint8_t byte;
	uint8_t bit;

	if (capture.fmt != PIXEL_FORMAT_MONO01 && capture.fmt != PIXEL_FORMAT_MONO10) {
		size_t n = DISPLAY_BITS_PER_PIXEL(capture.fmt) / 8U;
		const uint8_t *p = &buf[((size_t)y * desc->pitch + x) * n];

		for (size_t i = 1; i < n; i++) {
			zassert_equal(p[i], p[0], "pixel bytes differ");
		}
		zassert_true(p[0] == 0x00 || p[0] == 0xFF);
		return p[0] == 0xFF;
	}

	if ((capture.screen_info & SCREEN_INFO_MONO_VTILED) != 0U) {
		byte = buf[(y / 8U) * desc->pitch + x];
		bit = msb ? 7U - (y % 8U) : y % 8U;
	} else {
		byte = buf[y * (desc->pitch / 8U) + x / 8U];
		bit = msb ? 7U - (x % 8U) : x % 8U;
	}

	return (((byte >> bit) & 1U) != 0U) != inv;
}

static int cap_write(const struct device *dev, const uint16_t x, const uint16_t y,
		     const struct display_buffer_descriptor *desc, const void *buf)
{
	ARG_UNUSED(dev);

	zassert_true(x + desc->width <= CAP_W);
	zassert_true(y + desc->height <= CAP_H);
	if ((capture.screen_info & SCREEN_INFO_MONO_VTILED) != 0U) {
		zassert_equal(y % 8U, 0);
		zassert_equal(desc->height % 8U, 0);
	}

	for (uint16_t r = 0; r < desc->height; r++) {
		for (uint16_t c = 0; c < desc->width; c++) {
			capture.white[y + r][x + c] = pixel(desc, buf, c, r);
		}
	}
	capture.writes++;
	capture.last_incomplete = desc->frame_incomplete;

	return 0;
}

static int cap_blanking_on(const struct device *dev)
{
	ARG_UNUSED(dev);
	capture.blanked = true;
	return 0;
}

static int cap_blanking_off(const struct device *dev)
{
	ARG_UNUSED(dev);
	capture.blanked = false;
	capture.refreshes++;
	return capture.blanking_off_ret;
}

static void cap_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	ARG_UNUSED(dev);
	memset(caps, 0, sizeof(*caps));
	caps->x_resolution = CAP_W;
	caps->y_resolution = CAP_H;
	caps->supported_pixel_formats = capture.fmt;
	caps->current_pixel_format = capture.fmt;
	caps->screen_info = capture.screen_info;
}

static DEVICE_API(display, cap_api) = {
	.write = cap_write,
	.blanking_on = cap_blanking_on,
	.blanking_off = cap_blanking_off,
	.get_capabilities = cap_get_capabilities,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,
		      &cap_api);

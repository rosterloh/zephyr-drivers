/*
 * OpenDisplay direct write: an uncompressed 1 bpp image (rows of
 * ceil(width / 8) bytes, MSB = leftmost pixel, 1 = white) streamed into the
 * zephyr,display device a band at a time, so no framebuffer is kept.
 *
 * Mono panels get 8-row bands in their native layout (horizontal or
 * vertically tiled, either bit order, either polarity). Displays with 8 bits
 * per pixel or more get each row as black/white pixels, written in segments.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/sys/util.h>

#include "od_internal.h"

LOG_MODULE_DECLARE(opendisplay, CONFIG_OPENDISPLAY_LOG_LEVEL);

#define OD_DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define OD_MAX_WIDTH    DT_PROP(OD_DISPLAY_NODE, width)
#define OD_MAX_STRIDE   DIV_ROUND_UP(OD_MAX_WIDTH, 8)
#define OD_BAND_ROWS    8U
#define OD_OUT_SIZE     MAX(OD_MAX_WIDTH, 256)

static const struct device *const od_display = DEVICE_DT_GET(OD_DISPLAY_NODE);

static const struct opendisplay_callbacks *od_cb;

static struct {
	const struct od_transport *owner;
	struct display_capabilities caps;
	uint32_t total;
	uint32_t received;
	uint16_t stride;
	uint16_t row;      /* first row of the band being filled */
	uint16_t band;     /* rows per band */
	uint16_t fill;     /* bytes in in[] */
	uint8_t bpp_bytes; /* 0 for mono */
	bool invert;
	bool active;
	uint8_t in[OD_BAND_ROWS * OD_MAX_STRIDE];
	uint8_t out[OD_OUT_SIZE];
} dw;

void opendisplay_set_callbacks(const struct opendisplay_callbacks *cb)
{
	od_cb = cb;
}

static const uint8_t rev4[16] = {0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
				 0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF};

static bool host_white(const uint8_t *row, uint16_t x)
{
	return (row[x / 8U] & BIT(7U - (x % 8U))) != 0U;
}

static int write_mono(uint16_t rows, bool last)
{
	const struct display_capabilities *caps = &dw.caps;
	struct display_buffer_descriptor desc = {
		.width = caps->x_resolution,
		.height = rows,
		.frame_incomplete = !last,
	};
	const uint8_t *buf = dw.in;

	if ((caps->screen_info & SCREEN_INFO_MONO_VTILED) != 0U) {
		bool msb = (caps->screen_info & SCREEN_INFO_MONO_MSB_FIRST) != 0U;

		for (uint16_t x = 0; x < caps->x_resolution; x++) {
			uint8_t v = 0U;

			for (uint16_t r = 0; r < rows; r++) {
				if (host_white(&dw.in[r * dw.stride], x) != dw.invert) {
					v |= BIT(msb ? 7U - r : r);
				}
			}
			dw.out[x] = v;
		}
		desc.pitch = caps->x_resolution;
		desc.buf_size = caps->x_resolution;
		buf = dw.out;
	} else {
		bool lsb = (caps->screen_info & SCREEN_INFO_MONO_MSB_FIRST) == 0U;

		for (size_t i = 0; i < (size_t)rows * dw.stride; i++) {
			uint8_t v = dw.in[i];

			if (lsb) {
				v = (uint8_t)((rev4[v & 0x0FU] << 4) | rev4[v >> 4]);
			}
			dw.in[i] = dw.invert ? (uint8_t)~v : v;
		}
		desc.pitch = dw.stride * 8U;
		desc.buf_size = (uint32_t)rows * dw.stride;
	}

	return display_write(od_display, 0, dw.row, &desc, buf);
}

static int write_pixels(bool last)
{
	const struct display_capabilities *caps = &dw.caps;
	uint16_t seg = sizeof(dw.out) / dw.bpp_bytes;

	for (uint16_t x = 0; x < caps->x_resolution; x += seg) {
		uint16_t n = MIN(seg, caps->x_resolution - x);
		struct display_buffer_descriptor desc = {
			.width = n,
			.height = 1,
			.pitch = n,
			.buf_size = (uint32_t)n * dw.bpp_bytes,
			.frame_incomplete = !(last && x + n == caps->x_resolution),
		};
		int err;

		for (uint16_t i = 0; i < n; i++) {
			memset(&dw.out[i * dw.bpp_bytes], host_white(dw.in, x + i) ? 0xFF : 0x00,
			       dw.bpp_bytes);
		}
		err = display_write(od_display, x, dw.row, &desc, dw.out);
		if (err != 0) {
			return err;
		}
	}

	return 0;
}

static int flush_band(void)
{
	uint16_t rows = dw.fill / dw.stride;
	bool last = dw.received == dw.total;
	int err;

	err = dw.bpp_bytes == 0U ? write_mono(rows, last) : write_pixels(last);
	if (err != 0) {
		LOG_ERR("display_write row %u: %d", dw.row, err);
		return err;
	}
	dw.row += rows;
	dw.fill = 0U;

	return 0;
}

static void abort_transfer(void)
{
	dw.active = false;
	dw.owner = NULL;
}

static int start(const struct od_transport *tp, size_t plen)
{
	struct display_capabilities *caps = &dw.caps;
	enum display_pixel_format fmt;
	uint8_t bits;

	abort_transfer();

	if (plen != 0U) {
		/* Compressed START: [size:4][zlib...]. Hosts fall back to plain. */
		LOG_DBG("compressed direct write not supported");
		return -ENOTSUP;
	}

	display_get_capabilities(od_display, caps);
	fmt = caps->current_pixel_format;
	bits = DISPLAY_BITS_PER_PIXEL(fmt);

	if (caps->x_resolution > OD_MAX_WIDTH) {
		return -EINVAL;
	}
	if (fmt == PIXEL_FORMAT_MONO01 || fmt == PIXEL_FORMAT_MONO10) {
		dw.bpp_bytes = 0U;
		dw.invert = fmt == PIXEL_FORMAT_MONO10;
		dw.band = OD_BAND_ROWS;
		if ((caps->screen_info & SCREEN_INFO_MONO_VTILED) != 0U &&
		    caps->y_resolution % OD_BAND_ROWS != 0U) {
			return -ENOTSUP;
		}
	} else if (bits >= 8U && bits % 8U == 0U && fmt != PIXEL_FORMAT_YUYV) {
		dw.bpp_bytes = bits / 8U;
		dw.band = 1U;
		if ((caps->screen_info & SCREEN_INFO_X_ALIGNMENT_WIDTH) != 0U &&
		    (size_t)caps->x_resolution * dw.bpp_bytes > sizeof(dw.out)) {
			return -ENOTSUP;
		}
	} else {
		LOG_WRN("pixel format 0x%x not supported", fmt);
		return -ENOTSUP;
	}

	dw.stride = DIV_ROUND_UP(caps->x_resolution, 8U);
	dw.total = (uint32_t)dw.stride * caps->y_resolution;
	dw.received = 0U;
	dw.row = 0U;
	dw.fill = 0U;

	/* E-paper: hold the refresh until END. */
	if ((caps->screen_info & SCREEN_INFO_EPD) != 0U) {
		(void)display_blanking_on(od_display);
	}

	dw.owner = tp;
	dw.active = true;
	LOG_INF("direct write %ux%u, %u bytes", caps->x_resolution, caps->y_resolution, dw.total);

	return 0;
}

static int data(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	if (!dw.active || dw.owner != tp) {
		return -EINVAL;
	}
	if (dw.received + len > dw.total) {
		LOG_WRN("direct write overflow");
		abort_transfer();
		return -EFBIG;
	}

	while (len > 0U) {
		size_t room = (size_t)dw.band * dw.stride - dw.fill;
		size_t n = MIN(room, len);

		memcpy(&dw.in[dw.fill], buf, n);
		dw.fill += n;
		dw.received += n;
		buf += n;
		len -= n;

		if (dw.fill == (size_t)dw.band * dw.stride || dw.received == dw.total) {
			int err = flush_band();

			if (err != 0) {
				abort_transfer();
				return err;
			}
		}
	}

	return 0;
}

static void end(const struct od_transport *tp)
{
	int err;

	if (!dw.active || dw.owner != tp || dw.received != dw.total) {
		LOG_WRN("direct write ended at %u of %u bytes", dw.received, dw.total);
		abort_transfer();
		(void)od_reply(tp, OD_NACK, OD_CMD_DW_END, NULL, 0);
		return;
	}
	abort_transfer();
	(void)od_reply(tp, OD_ACK, OD_CMD_DW_END, NULL, 0);

	if (od_cb != NULL && od_cb->image_received != NULL) {
		od_cb->image_received();
	}

	/* Refreshes the panel; blocks until an e-paper refresh completes. */
	err = display_blanking_off(od_display);
	if (err == -ENOSYS) {
		err = 0;
	}
	if (err != 0) {
		LOG_ERR("refresh failed: %d", err);
	}
	(void)od_reply(tp, OD_ACK, err == 0 ? OD_EVT_REFRESH_OK : OD_EVT_REFRESH_TIMEOUT, NULL, 0);

	if (od_cb != NULL && od_cb->refresh_done != NULL) {
		od_cb->refresh_done(err);
	}
}

void od_direct(const struct od_transport *tp, uint16_t cmd, const uint8_t *payload, size_t len)
{
	int err;

	switch (cmd) {
	case OD_CMD_DW_START:
		err = start(tp, len);
		(void)od_reply(tp, err == 0 ? OD_ACK : OD_NACK, cmd, NULL, 0);
		break;
	case OD_CMD_DW_DATA:
		err = data(tp, payload, len);
		(void)od_reply(tp, err == 0 ? OD_ACK : OD_NACK, cmd, NULL, 0);
		break;
	default:
		end(tp);
		break;
	}
}

void od_direct_abort(const struct od_transport *tp)
{
	if (dw.owner == tp) {
		abort_transfer();
	}
}

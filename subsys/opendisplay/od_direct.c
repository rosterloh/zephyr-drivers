/*
 * OpenDisplay direct write: an uncompressed 1 bpp image (rows of
 * ceil(width / 8) bytes, MSB = leftmost pixel, 1 = white) streamed into the
 * zephyr,display device a band at a time, so no framebuffer is kept.
 *
 * Mono panels get 8-row bands in their native layout (horizontal or
 * vertically tiled, either bit order, either polarity). Displays with 8 bits
 * per pixel or more get each row as black/white pixels, written in segments.
 *
 * The image arrives through DIRECT_WRITE (0x70-0x72), optionally zlib
 * compressed, or through PIPE_WRITE (0x80-0x82): the same bytes in sequenced
 * frames with one cumulative ACK per N frames. The receiver keeps no
 * out-of-order frames; the host rewinds or re-sends after a gap.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "od_internal.h"

LOG_MODULE_DECLARE(opendisplay, CONFIG_OPENDISPLAY_LOG_LEVEL);

#define OD_DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define OD_MAX_WIDTH    DT_PROP(OD_DISPLAY_NODE, width)
#define OD_MAX_STRIDE   DIV_ROUND_UP(OD_MAX_WIDTH, 8)
#define OD_BAND_ROWS    8U
#define OD_OUT_SIZE     MAX(OD_MAX_WIDTH, 256)

#define PIPE_VERSION       1U
#define PIPE_FLAG_ZLIB     0x01U
#define PIPE_FLAG_PARTIAL  0x02U
#define PIPE_MAX_WINDOW    32U
#define PIPE_MAX_ACK_EVERY 8U
/* Largest BLE notification at the default 247-byte MTU. */
#define PIPE_MAX_FRAME     244U
/* PIPE_WRITE_START NACK reasons */
#define PIPE_ERR_PARAMS    0x01U
#define PIPE_ERR_ZLIB      0x02U
#define PIPE_ERR_SIZE      0x03U
#define PIPE_ERR_STATE     0x04U
#define PIPE_ERR_PARTIAL   0x06U
/* PIPE_WRITE_DATA NACK reasons */
#define PIPE_ERR_INFLATE   0x02U
#define PIPE_ERR_WRITE     0x03U

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
	bool zlib;
	bool pipe;
	uint8_t ack_every;
	uint8_t since_ack;
	uint8_t next_seq;
	bool any_seq;
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

#if defined(CONFIG_OPENDISPLAY_ZLIB)
#define inflate_start()        od_inflate_start()
#define inflate_data(buf, len) od_inflate_data(buf, len)
#define inflate_end()          od_inflate_end()
#else
#define inflate_start()
#define inflate_data(buf, len) (-ENOTSUP)
#define inflate_end()          (-ENOTSUP)
#endif

static void abort_transfer(void)
{
	dw.active = false;
	dw.owner = NULL;
}

static int start(const struct od_transport *tp)
{
	struct display_capabilities *caps = &dw.caps;
	enum display_pixel_format fmt;
	uint8_t bits;

	abort_transfer();
	dw.zlib = false;
	dw.pipe = false;

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

int od_direct_feed(const uint8_t *buf, size_t len)
{
	if (dw.received + len > dw.total) {
		LOG_WRN("direct write overflow");
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
				return err;
			}
		}
	}

	return 0;
}

static int data(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	int err;

	if (!dw.active || dw.owner != tp) {
		return -EINVAL;
	}
	err = dw.zlib ? inflate_data(buf, len) : od_direct_feed(buf, len);
	if (err != 0) {
		abort_transfer();
	}

	return err;
}

/* Finish the transfer and refresh; @p cmd is the END opcode to acknowledge. */
static void end(const struct od_transport *tp, uint16_t cmd)
{
	int err;

	if (!dw.active || dw.owner != tp || (dw.zlib && inflate_end() != 0) ||
	    dw.received != dw.total) {
		LOG_WRN("direct write ended at %u of %u bytes", dw.received, dw.total);
		abort_transfer();
		(void)od_reply(tp, OD_NACK, cmd, NULL, 0);
		return;
	}
	abort_transfer();
	(void)od_reply(tp, OD_ACK, cmd, NULL, 0);

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

/* START payload: none, or [uncompressed size:4 LE][first zlib bytes]. */
static int dw_start(const struct od_transport *tp, const uint8_t *payload, size_t len)
{
	int err;

	if (len != 0U && !IS_ENABLED(CONFIG_OPENDISPLAY_ZLIB)) {
		/* Hosts fall back to an uncompressed upload. */
		return -ENOTSUP;
	}
	if (len != 0U && len < 6U) {
		return -EINVAL;
	}
	err = start(tp);
	if (err == 0 && len != 0U) {
		dw.zlib = true;
		inflate_start();
		/* The zlib header is in here, so a bad window NACKs the START. */
		err = sys_get_le32(payload) != dw.total ? -EINVAL
							: inflate_data(&payload[4], len - 4U);
	}
	if (err != 0) {
		abort_transfer();
	}

	return err;
}

#if defined(CONFIG_OPENDISPLAY_PIPE)
static void pipe_nack(const struct od_transport *tp, uint16_t cmd, uint8_t err)
{
	/* START: [err][0]; DATA: [err][highest seq][mask:4 LE] */
	uint8_t buf[6] = {err, cmd == OD_CMD_PIPE_START ? 0U : (uint8_t)(dw.next_seq - 1U)};

	(void)od_reply(tp, OD_NACK, cmd, buf, cmd == OD_CMD_PIPE_START ? 2U : sizeof(buf));
}

/* [ver][flags][window][ack_every][max_frame:2 LE][total:4 LE] */
static void pipe_start(const struct od_transport *tp, const uint8_t *p, size_t len)
{
	uint8_t ack[6] = {PIPE_VERSION, PIPE_MAX_WINDOW, PIPE_MAX_ACK_EVERY};
	uint8_t err = 0U;
	bool zlib;

	if (len < 10U || p[0] != PIPE_VERSION) {
		err = PIPE_ERR_PARAMS;
	} else if ((p[1] & PIPE_FLAG_PARTIAL) != 0U) {
		err = PIPE_ERR_PARTIAL;
	} else if ((p[1] & PIPE_FLAG_ZLIB) != 0U && !IS_ENABLED(CONFIG_OPENDISPLAY_ZLIB)) {
		err = PIPE_ERR_ZLIB;
	} else if (start(tp) != 0) {
		err = PIPE_ERR_STATE;
	} else if (sys_get_le32(&p[6]) != dw.total) {
		err = PIPE_ERR_SIZE;
	}
	if (err != 0U) {
		abort_transfer();
		pipe_nack(tp, OD_CMD_PIPE_START, err);
		return;
	}

	zlib = (p[1] & PIPE_FLAG_ZLIB) != 0U;
	/* The host computes the same: min of both sides, ack_every <= window. */
	dw.ack_every =
		CLAMP(MIN(p[3], PIPE_MAX_ACK_EVERY), 1U, MAX(MIN(p[2], PIPE_MAX_WINDOW), 1U));
	dw.since_ack = 0U;
	dw.next_seq = 0U;
	dw.any_seq = false;
	dw.pipe = true;
	if (zlib) {
		/* The stream, header included, arrives in DATA frames. */
		dw.zlib = true;
		inflate_start();
	}
	sys_put_le16(PIPE_MAX_FRAME, &ack[3]);
	(void)od_reply(tp, OD_ACK, OD_CMD_PIPE_START, ack, sizeof(ack));
}

/* Cumulative ACK: [highest in-order seq][mask:4 LE, all earlier frames seen]. */
static void pipe_ack(const struct od_transport *tp)
{
	uint8_t buf[5] = {(uint8_t)(dw.next_seq - 1U), 0xFF, 0xFF, 0xFF, 0xFF};

	dw.since_ack = 0U;
	(void)od_reply(tp, OD_ACK, OD_CMD_PIPE_DATA, buf, sizeof(buf));
}

/* [seq][data] */
static void pipe_data(const struct od_transport *tp, const uint8_t *p, size_t len)
{
	int err;

	if (!dw.active || !dw.pipe || dw.owner != tp || len < 1U) {
		pipe_nack(tp, OD_CMD_PIPE_DATA, PIPE_ERR_STATE);
		return;
	}
	if (p[0] != dw.next_seq) {
		/* Duplicate or gap: tell the host where we are. */
		if (dw.any_seq) {
			pipe_ack(tp);
		}
		return;
	}

	err = data(tp, &p[1], len - 1U);
	if (err != 0) {
		pipe_nack(tp, OD_CMD_PIPE_DATA,
			  err == -EBADMSG ? PIPE_ERR_INFLATE : PIPE_ERR_WRITE);
		return;
	}
	dw.next_seq++;
	dw.any_seq = true;
	if (++dw.since_ack >= dw.ack_every || dw.received == dw.total) {
		pipe_ack(tp);
	}
	/* Uncompressed transfers end themselves; the host sends no END. */
	if (!dw.zlib && dw.received == dw.total) {
		end(tp, OD_CMD_PIPE_END);
	}
}
#endif /* CONFIG_OPENDISPLAY_PIPE */

void od_direct(const struct od_transport *tp, uint16_t cmd, const uint8_t *payload, size_t len)
{
	int err;

	switch (cmd) {
	case OD_CMD_DW_START:
		err = dw_start(tp, payload, len);
		(void)od_reply(tp, err == 0 ? OD_ACK : OD_NACK, cmd, NULL, 0);
		break;
	case OD_CMD_DW_DATA:
		err = data(tp, payload, len);
		(void)od_reply(tp, err == 0 ? OD_ACK : OD_NACK, cmd, NULL, 0);
		break;
	case OD_CMD_DW_END:
		end(tp, cmd);
		break;
#if defined(CONFIG_OPENDISPLAY_PIPE)
	case OD_CMD_PIPE_START:
		pipe_start(tp, payload, len);
		break;
	case OD_CMD_PIPE_DATA:
		pipe_data(tp, payload, len);
		break;
#endif
	default:
		end(tp, cmd);
		break;
	}
}

void od_direct_abort(const struct od_transport *tp)
{
	if (dw.owner == tp) {
		abort_transfer();
	}
}

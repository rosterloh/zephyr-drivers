/*
 * OpenDisplay direct write into a capture display in each supported layout.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "capture_display.h"
#include "od_internal.h"

#define STRIDE    DIV_ROUND_UP(CAP_W, 8)
#define IMG_SIZE  (STRIDE * CAP_H)
#define MAX_REPLY 16

static uint8_t image[IMG_SIZE];

static struct {
	uint8_t frame[MAX_REPLY][8];
	size_t len[MAX_REPLY];
	size_t count;
} rep;

static int images;
static int refreshed_err = 1;

static int cap_send(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(tp);
	zassert_true(rep.count < MAX_REPLY);
	zassert_true(len >= 2U && len <= 8U);
	rep.len[rep.count] = len;
	memcpy(rep.frame[rep.count++], buf, len);
	return 0;
}

static const struct od_transport tp = {.send = cap_send};
static const struct od_transport other = {.send = cap_send};

static void on_image(void)
{
	images++;
}

static void on_refresh(int err)
{
	refreshed_err = err;
}

static const struct opendisplay_callbacks cbs = {
	.image_received = on_image,
	.refresh_done = on_refresh,
};

/* Send one command; return its single reply as [status][echo]. */
static uint16_t send(const struct od_transport *t, uint16_t op, const uint8_t *data, size_t len)
{
	uint8_t req[2 + 232];

	zassert_true(len <= sizeof(req) - 2U);
	rep.count = 0;
	sys_put_be16(op, req);
	memcpy(&req[2], data, len);
	od_handle(t, req, len + 2U);
	zassert_true(rep.count >= 1U);
	return sys_get_be16(rep.frame[0]);
}

static void upload(size_t chunk)
{
	zassert_equal(send(&tp, OD_CMD_DW_START, NULL, 0), 0x0070);
	for (size_t off = 0; off < sizeof(image); off += chunk) {
		zassert_equal(
			send(&tp, OD_CMD_DW_DATA, &image[off], MIN(chunk, sizeof(image) - off)),
			0x0071);
	}
	zassert_equal(send(&tp, OD_CMD_DW_END, (const uint8_t[]){0}, 1), 0x0072);
	zassert_equal(rep.count, 2, "END ack then refresh event");
}

static void check_canvas(void)
{
	for (int y = 0; y < CAP_H; y++) {
		for (int x = 0; x < CAP_W; x++) {
			bool white = (image[y * STRIDE + x / 8] & BIT(7 - x % 8)) != 0;

			zassert_equal(capture.white[y][x], white, "pixel %d,%d", x, y);
		}
	}
	zassert_false(capture.last_incomplete, "last write ends the frame");
	zassert_equal(capture.refreshes, 1);
	zassert_equal(images, 1);
	zassert_equal(refreshed_err, 0);
}

static void run(enum display_pixel_format fmt, uint32_t info, size_t chunk)
{
	capture.fmt = fmt;
	capture.screen_info = info;
	upload(chunk);
	zassert_equal(sys_get_be16(rep.frame[1]), 0x0073, "refresh ok");
	check_canvas();
}

static void before(void *f)
{
	ARG_UNUSED(f);
	memset(&capture, 0, sizeof(capture));
	memset(&rep, 0, sizeof(rep));
	images = 0;
	refreshed_err = 1;
	sys_rand_get(image, sizeof(image));
	opendisplay_set_callbacks(&cbs);
}

ZTEST(opendisplay_direct, test_mono01_horizontal_msb)
{
	run(PIXEL_FORMAT_MONO01, SCREEN_INFO_MONO_MSB_FIRST, 230);
}

ZTEST(opendisplay_direct, test_mono10_horizontal_lsb)
{
	run(PIXEL_FORMAT_MONO10, 0, 7);
}

ZTEST(opendisplay_direct, test_mono01_vtiled_msb_epd)
{
	/* ssd16xx in its normal orientation. */
	run(PIXEL_FORMAT_MONO01,
	    SCREEN_INFO_MONO_VTILED | SCREEN_INFO_MONO_MSB_FIRST | SCREEN_INFO_EPD, 230);
	zassert_false(capture.blanked);
}

ZTEST(opendisplay_direct, test_mono10_vtiled_lsb)
{
	run(PIXEL_FORMAT_MONO10, SCREEN_INFO_MONO_VTILED, 100);
}

ZTEST(opendisplay_direct, test_rgb565)
{
	run(PIXEL_FORMAT_RGB_565, 0, 230);
}

ZTEST(opendisplay_direct, test_argb8888_segments)
{
	/* 100 px at 4 bytes per pixel goes out in two segments per row. */
	run(PIXEL_FORMAT_ARGB_8888, 0, 230);
	zassert_equal(capture.writes, 2 * CAP_H);
}

ZTEST(opendisplay_direct, test_epd_blanked_during_upload)
{
	capture.fmt = PIXEL_FORMAT_MONO01;
	capture.screen_info = SCREEN_INFO_MONO_MSB_FIRST | SCREEN_INFO_EPD;
	zassert_equal(send(&tp, OD_CMD_DW_START, NULL, 0), 0x0070);
	zassert_true(capture.blanked, "refresh held until END");
	for (size_t off = 0; off < sizeof(image); off += 200) {
		zassert_equal(send(&tp, OD_CMD_DW_DATA, &image[off], MIN(200, sizeof(image) - off)),
			      0x0071);
		zassert_true(capture.blanked);
	}
	zassert_equal(send(&tp, OD_CMD_DW_END, NULL, 0), 0x0072);
	zassert_false(capture.blanked);
}

ZTEST(opendisplay_direct, test_refresh_failure)
{
	capture.blanking_off_ret = -EIO;
	capture.fmt = PIXEL_FORMAT_MONO01;
	capture.screen_info = SCREEN_INFO_MONO_MSB_FIRST;
	upload(230);
	zassert_equal(sys_get_be16(rep.frame[1]), 0x0074, "refresh timeout event");
	zassert_equal(refreshed_err, -EIO);
}

ZTEST(opendisplay_direct, test_rejects)
{
	capture.fmt = PIXEL_FORMAT_MONO01;
	capture.screen_info = SCREEN_INFO_MONO_MSB_FIRST;

	/* Compressed START: the host falls back to uncompressed. */
	zassert_equal(send(&tp, OD_CMD_DW_START, (const uint8_t[]){0, 1, 0, 0, 0x78}, 5), 0xFF70);

	/* DATA without START. */
	zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 10), 0xFF71);

	/* More data than the image holds aborts the transfer. */
	zassert_equal(send(&tp, OD_CMD_DW_START, NULL, 0), 0x0070);
	for (size_t off = 0; off + 200 <= sizeof(image); off += 200) {
		zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 200), 0x0071);
	}
	zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 200), 0xFF71);
	zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 1), 0xFF71, "aborted");

	/* END before all bytes arrived. */
	zassert_equal(send(&tp, OD_CMD_DW_START, NULL, 0), 0x0070);
	zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 10), 0x0071);
	zassert_equal(send(&tp, OD_CMD_DW_END, NULL, 0), 0xFF72);
	zassert_equal(rep.count, 1, "no refresh event");

	/* DATA from another link, and a disconnect, end the transfer. */
	zassert_equal(send(&tp, OD_CMD_DW_START, NULL, 0), 0x0070);
	zassert_equal(send(&other, OD_CMD_DW_DATA, image, 10), 0xFF71);
	od_disconnected(&tp);
	zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 10), 0xFF71);

	/* Unsupported pixel format. */
	capture.fmt = PIXEL_FORMAT_I_4;
	zassert_equal(send(&tp, OD_CMD_DW_START, NULL, 0), 0xFF70);
	zassert_equal(capture.refreshes, 0);
}

#if defined(CONFIG_OPENDISPLAY_ZLIB)
#include "zlib_vectors.inc"

BUILD_ASSERT(sizeof(zimage) == IMG_SIZE);

static void mono(void)
{
	capture.fmt = PIXEL_FORMAT_MONO01;
	capture.screen_info = SCREEN_INFO_MONO_MSB_FIRST;
	memcpy(image, zimage, sizeof(image));
}

/* Compressed START carries [size:4 LE] and the first stream bytes. */
static uint16_t zstart(const uint8_t *z, size_t first, uint32_t size)
{
	uint8_t p[4 + 194];

	sys_put_le32(size, p);
	memcpy(&p[4], z, first);
	return send(&tp, OD_CMD_DW_START, p, 4 + first);
}

ZTEST(opendisplay_direct, test_zlib)
{
	mono();
	zassert_equal(zstart(z9, 194, IMG_SIZE), 0x0070);
	for (size_t off = 194; off < sizeof(z9); off += 230) {
		zassert_equal(send(&tp, OD_CMD_DW_DATA, &z9[off], MIN(230, sizeof(z9) - off)),
			      0x0071);
	}
	zassert_equal(send(&tp, OD_CMD_DW_END, NULL, 0), 0x0072);
	zassert_equal(sys_get_be16(rep.frame[1]), 0x0073);
	check_canvas();
}

ZTEST(opendisplay_direct, test_zlib_rejects)
{
	static uint8_t bad[sizeof(z9)];

	mono();
	/* Window over 512 bytes: the host falls back to uncompressed. */
	zassert_equal(zstart(z15, 194, IMG_SIZE), 0xFF70);
	zassert_equal(zstart(z9, 194, IMG_SIZE - 1), 0xFF70, "size mismatch");
	zassert_equal(zstart(z9, 1, IMG_SIZE), 0xFF70, "no full zlib header");

	/* Truncated stream. */
	zassert_equal(zstart(z9, 194, IMG_SIZE), 0x0070);
	zassert_equal(send(&tp, OD_CMD_DW_DATA, &z9[194], 200), 0x0071);
	zassert_equal(send(&tp, OD_CMD_DW_END, NULL, 0), 0xFF72);

	/* Bad Adler-32. */
	memcpy(bad, z9, sizeof(bad));
	bad[sizeof(bad) - 1] ^= 1;
	zassert_equal(zstart(bad, 194, IMG_SIZE), 0x0070);
	for (size_t off = 194; off < sizeof(bad); off += 230) {
		zassert_equal(send(&tp, OD_CMD_DW_DATA, &bad[off], MIN(230, sizeof(bad) - off)),
			      0x0071);
	}
	zassert_equal(send(&tp, OD_CMD_DW_END, NULL, 0), 0xFF72);
	zassert_equal(capture.refreshes, 0);
}
#endif /* CONFIG_OPENDISPLAY_ZLIB */

#if defined(CONFIG_OPENDISPLAY_PIPE)
#define PIPE_CHUNK 100

static void pipe_start(uint8_t flags, uint32_t total)
{
	uint8_t p[10] = {1, flags, 8, 4};
	static const uint8_t want[] = {0x00, 0x80, 1, 32, 8, 244, 0, 0};

	sys_put_le16(244, &p[4]);
	sys_put_le32(total, &p[6]);
	send(&tp, OD_CMD_PIPE_START, p, sizeof(p));
	zassert_equal(rep.len[0], sizeof(want));
	zassert_mem_equal(rep.frame[0], want, sizeof(want));
}

/* Send frame @p seq of @p src; returns how many replies came back. */
static size_t pipe_frame(const uint8_t *src, size_t size, uint8_t seq)
{
	uint8_t p[1 + PIPE_CHUNK];
	size_t off = (size_t)seq * PIPE_CHUNK;
	size_t n = MIN(PIPE_CHUNK, size - off);

	p[0] = seq;
	memcpy(&p[1], &src[off], n);
	rep.count = 0;
	{
		uint8_t req[2 + sizeof(p)];

		sys_put_be16(OD_CMD_PIPE_DATA, req);
		memcpy(&req[2], p, n + 1);
		od_handle(&tp, req, n + 3);
	}
	return rep.count;
}

static void expect_ack(size_t i, uint8_t highest)
{
	const uint8_t want[] = {0x00, 0x81, highest, 0xFF, 0xFF, 0xFF, 0xFF};

	zassert_equal(rep.len[i], sizeof(want));
	zassert_mem_equal(rep.frame[i], want, sizeof(want), "ack %u", highest);
}

ZTEST(opendisplay_direct, test_pipe)
{
	const size_t frames = DIV_ROUND_UP(IMG_SIZE, PIPE_CHUNK);

	capture.fmt = PIXEL_FORMAT_MONO01;
	capture.screen_info = SCREEN_INFO_MONO_MSB_FIRST;
	pipe_start(0, IMG_SIZE);

	/* ACK every 4 in-order frames. */
	for (uint8_t s = 0; s < 3; s++) {
		zassert_equal(pipe_frame(image, IMG_SIZE, s), 0);
	}
	zassert_equal(pipe_frame(image, IMG_SIZE, 3), 1);
	expect_ack(0, 3);

	/* A gap and a duplicate are dropped and answered with the current ACK. */
	zassert_equal(pipe_frame(image, IMG_SIZE, 5), 1);
	expect_ack(0, 3);
	zassert_equal(pipe_frame(image, IMG_SIZE, 2), 1);
	expect_ack(0, 3);

	for (uint8_t s = 4; s < frames - 1; s++) {
		pipe_frame(image, IMG_SIZE, s);
	}
	/* Last frame: flush ACK, then the device ends the transfer itself. */
	zassert_equal(pipe_frame(image, IMG_SIZE, frames - 1), 3);
	expect_ack(0, frames - 1);
	zassert_equal(sys_get_be16(rep.frame[1]), 0x0082);
	zassert_equal(sys_get_be16(rep.frame[2]), 0x0073);
	check_canvas();
}

#if defined(CONFIG_OPENDISPLAY_ZLIB)
ZTEST(opendisplay_direct, test_pipe_zlib)
{
	const size_t frames = DIV_ROUND_UP(sizeof(z9), PIPE_CHUNK);

	mono();
	pipe_start(0x01, IMG_SIZE);
	for (uint8_t s = 0; s < frames; s++) {
		pipe_frame(z9, sizeof(z9), s);
	}
	/* The tail earns no cadence ACK; the host probes with a duplicate. */
	zassert_equal(pipe_frame(z9, sizeof(z9), frames - 1), 1);
	expect_ack(0, frames - 1);
	zassert_equal(send(&tp, OD_CMD_PIPE_END, (const uint8_t[]){0}, 1), 0x0082);
	zassert_equal(sys_get_be16(rep.frame[1]), 0x0073);
	check_canvas();
}
#endif

ZTEST(opendisplay_direct, test_pipe_rejects)
{
	uint8_t p[10] = {1, 0, 8, 4};

	capture.fmt = PIXEL_FORMAT_MONO01;
	capture.screen_info = SCREEN_INFO_MONO_MSB_FIRST;
	sys_put_le32(IMG_SIZE, &p[6]);

	p[0] = 2;
	zassert_equal(send(&tp, OD_CMD_PIPE_START, p, sizeof(p)), 0xFF80, "version");
	zassert_equal(rep.frame[0][2], 0x01);
	p[0] = 1;
	p[1] = 0x02;
	zassert_equal(send(&tp, OD_CMD_PIPE_START, p, sizeof(p)), 0xFF80, "partial");
	zassert_equal(rep.frame[0][2], 0x06);
	p[1] = 0;
	sys_put_le32(IMG_SIZE + 1, &p[6]);
	zassert_equal(send(&tp, OD_CMD_PIPE_START, p, sizeof(p)), 0xFF80, "size");
	zassert_equal(rep.frame[0][2], 0x03);

	zassert_equal(send(&tp, OD_CMD_PIPE_DATA, (const uint8_t[]){0, 1}, 2), 0xFF81);
	zassert_equal(rep.len[0], 8);
	zassert_equal(rep.frame[0][2], 0x04, "no transfer");
}
#endif /* CONFIG_OPENDISPLAY_PIPE */

ZTEST_SUITE(opendisplay_direct, NULL, NULL, before, NULL, NULL);

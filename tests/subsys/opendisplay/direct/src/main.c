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
#define MAX_REPLY 8

static uint8_t image[IMG_SIZE];

static struct {
	uint8_t frame[MAX_REPLY][4];
	size_t count;
} rep;

static int images;
static int refreshed_err = 1;

static int cap_send(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(tp);
	zassert_true(rep.count < MAX_REPLY);
	zassert_true(len >= 2U && len <= 4U);
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
	zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 200), 0x0071);
	zassert_equal(send(&tp, OD_CMD_DW_DATA, &image[200], sizeof(image) - 200), 0x0071);
	zassert_true(capture.blanked);
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
	zassert_equal(send(&tp, OD_CMD_DW_DATA, image, 200), 0x0071);
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

ZTEST_SUITE(opendisplay_direct, NULL, NULL, before, NULL, NULL);

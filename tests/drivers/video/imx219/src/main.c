/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register-sequence tests for the IMX219 fork. Each one asserts the state the
 * sensor is left in, which is the part of #37's bring-up that never needed a
 * camera: the register read-back was self-consistent there while no frame
 * arrived, and these tests say nothing about frames either.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/video/formats.h>
#include <zephyr/video/video.h>
#include <zephyr/ztest.h>

#include "imx219_emul.h"

static const struct device *const cam = DEVICE_DT_GET(DT_NODELABEL(imx219));

/* What imx219_init() left behind, captured before any test changes it. */
static struct {
	struct video_format fmt;
	uint16_t x_output_size;
	uint16_t y_output_size;
	uint16_t csi_data_format;
	uint8_t oppxck_div;
} boot;

static void set_fmt(uint32_t pixelformat, uint32_t width, uint32_t height)
{
	struct video_format fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = pixelformat,
		.width = width,
		.height = height,
	};

	zassert_ok(video_set_format(cam, &fmt), "%ux%u rejected", width, height);
}

static void set_fps(uint32_t fps)
{
	struct video_frmival frmival = {.numerator = 1, .denominator = fps};

	zassert_ok(video_set_frmival(cam, &frmival));
	zassert_equal(frmival.denominator, fps);
}

/* Centred crop window of w x h on the 3280x2464 array, as inclusive bounds. */
static void assert_crop(uint16_t w, uint16_t h)
{
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_X_ADD_STA_A), (3280 - w) / 2);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_X_ADD_END_A), (3280 + w) / 2 - 1);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_Y_ADD_STA_A), (2464 - h) / 2);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_Y_ADD_END_A), (2464 + h) / 2 - 1);
}

static void *imx219_setup(void)
{
	zassert_true(device_is_ready(cam), "probe should succeed against the emulator");

	boot.fmt.type = VIDEO_BUF_TYPE_OUTPUT;
	zassert_ok(video_get_format(cam, &boot.fmt));
	boot.x_output_size = imx219_emul_reg16(IMX219_EMUL_X_OUTPUT_SIZE);
	boot.y_output_size = imx219_emul_reg16(IMX219_EMUL_Y_OUTPUT_SIZE);
	boot.csi_data_format = imx219_emul_reg16(IMX219_EMUL_CSI_DATA_FORMAT_A);
	boot.oppxck_div = imx219_emul_reg8(IMX219_EMUL_OPPXCK_DIV);

	return NULL;
}

/* #44: the boot format was the caps minimum, 4x4. */
ZTEST(imx219, test_boot_format_is_a_real_mode)
{
	zassert_equal(boot.fmt.pixelformat, VIDEO_PIX_FMT_SBGGR10P);
	zassert_equal(boot.fmt.width, 1640);
	zassert_equal(boot.fmt.height, 1232);
	zassert_equal(boot.fmt.pitch, 1640 * 10 / 8);

	zassert_equal(boot.x_output_size, 1640);
	zassert_equal(boot.y_output_size, 1232);
	zassert_equal(boot.csi_data_format, 0x0a0a);
	zassert_equal(boot.oppxck_div, 10);
}

ZTEST(imx219, test_probe_programs_clock_and_link)
{
	/* 24 MHz INCK, as MHz << 8. */
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_EXCK_FREQ), 0x1800);
	/* 24 / 3 * 114 = 912 Mbps per lane, matching the advertised link frequency. */
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_PLL_OP_MPY), 114);
}

/*
 * The defect #35 fixed: set_frmival() used to rewrite OPPXCK_DIV to 10 after
 * set_fmt() had chosen RAW8, leaving 8-bit payloads clocked out for 10.
 */
ZTEST(imx219, test_raw8_divider_survives_set_frmival)
{
	set_fmt(VIDEO_PIX_FMT_SBGGR8, 1640, 1232);
	set_fps(30);
	set_fps(15);

	zassert_equal(imx219_emul_reg8(IMX219_EMUL_OPPXCK_DIV), 8);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_CSI_DATA_FORMAT_A), 0x0808);
}

ZTEST(imx219, test_raw10_divider_and_data_format)
{
	set_fmt(VIDEO_PIX_FMT_SBGGR8, 1920, 1080);
	set_fmt(VIDEO_PIX_FMT_SBGGR10P, 1920, 1080);
	set_fps(30);

	zassert_equal(imx219_emul_reg8(IMX219_EMUL_OPPXCK_DIV), 10);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_CSI_DATA_FORMAT_A), 0x0a0a);
}

/* #37: binning was never enabled, so 1640x1232 was a centre crop. */
ZTEST(imx219, test_half_resolution_bins_full_field_of_view)
{
	set_fmt(VIDEO_PIX_FMT_SBGGR10P, 1640, 1232);

	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_H), 0x03);
	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_V), 0x03);
	assert_crop(3280, 2464);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_X_OUTPUT_SIZE), 1640);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_Y_OUTPUT_SIZE), 1232);
	/* Analog binning needs the longer minimum line. */
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_LINE_LENGTH_A), 3560);
}

ZTEST(imx219, test_small_mode_crops_twice_the_output)
{
	set_fmt(VIDEO_PIX_FMT_SBGGR10P, 640, 480);

	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_H), 0x03);
	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_V), 0x03);
	assert_crop(1280, 960);
}

ZTEST(imx219, test_1080p_is_an_unbinned_crop)
{
	set_fmt(VIDEO_PIX_FMT_SBGGR10P, 1920, 1080);

	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_H), 0x00);
	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_V), 0x00);
	assert_crop(1920, 1080);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_LINE_LENGTH_A), 3448);
}

ZTEST(imx219, test_binning_keeps_pixels_square)
{
	/* The height fits twice into the array but the width does not: no binning. */
	set_fmt(VIDEO_PIX_FMT_SBGGR10P, 1920, 1232);

	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_H), 0x00);
	zassert_equal(imx219_emul_reg8(IMX219_EMUL_BINNING_MODE_V), 0x00);
	assert_crop(1920, 1232);
}

/*
 * Frame length comes from the requested rate: 182.4 MHz / (3448 * 30) = 1763,
 * the FLL the Raspberry Pi driver uses for its 30 fps 1080p mode. It used to be
 * height + 20 whatever rate was asked for.
 */
ZTEST(imx219, test_frame_length_follows_frame_rate)
{
	set_fmt(VIDEO_PIX_FMT_SBGGR10P, 1920, 1080);

	set_fps(30);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_FRM_LENGTH_A), 1763);

	set_fps(15);
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_FRM_LENGTH_A), 3526);
}

ZTEST(imx219, test_frame_length_never_shorter_than_output)
{
	set_fps(30);
	set_fmt(VIDEO_PIX_FMT_SBGGR10P, 3280, 2464);

	/* 30 fps is out of reach at full resolution; keep 32 lines of blanking. */
	zassert_equal(imx219_emul_reg16(IMX219_EMUL_FRM_LENGTH_A), 2464 + 32);
}

ZTEST(imx219, test_stream_start_and_stop_drive_mode_select)
{
	zassert_ok(video_stream_start(cam, VIDEO_BUF_TYPE_OUTPUT));
	zassert_equal(imx219_emul_reg8(IMX219_EMUL_MODE_SELECT), 0x01);

	zassert_ok(video_stream_stop(cam, VIDEO_BUF_TYPE_OUTPUT));
	zassert_equal(imx219_emul_reg8(IMX219_EMUL_MODE_SELECT), 0x00);
}

ZTEST(imx219, test_set_fmt_rejects_unsupported_requests)
{
	struct video_format fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_SBGGR10P,
		.width = 1642,
		.height = 1232,
	};

	zassert_equal(video_set_format(cam, &fmt), -EINVAL, "width off the 4-pixel step");

	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	fmt.width = 1640;
	zassert_equal(video_set_format(cam, &fmt), -ENOTSUP);
}

ZTEST_SUITE(imx219, NULL, imx219_setup, NULL, NULL, NULL);

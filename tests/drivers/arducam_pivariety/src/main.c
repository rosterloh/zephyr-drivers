/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/video/formats.h>
#include <zephyr/ztest.h>

#include "pivariety_emul.h"

static const struct device *const cam = DEVICE_DT_GET(DT_NODELABEL(pivariety));

ZTEST(pivariety, test_device_is_ready)
{
	zassert_true(device_is_ready(cam), "probe should succeed against the emulator");
}

ZTEST(pivariety, test_probe_reads_identity_registers)
{
	/* Probe must read all three identity registers, or a wrong/absent module
	 * would bind silently.
	 */
	zassert_true(pivariety_emul_was_read(DEVICE_ID_REG), "DEVICE_ID_REG not read");
	zassert_true(pivariety_emul_was_read(SENSOR_ID_REG), "SENSOR_ID_REG not read");
	zassert_true(pivariety_emul_was_read(DEVICE_VERSION_REG), "DEVICE_VERSION_REG not read");
}

ZTEST(pivariety, test_enumerates_all_format_resolution_pairs)
{
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
	int n = 0;

	zassert_ok(video_get_caps(cam, &caps));

	while (caps.format_caps[n].pixelformat != 0) {
		n++;
	}

	/* The emulator advertises RAW10-grey at 2 resolutions and RAW8-BGGR at 1. */
	zassert_equal(n, 3, "expected 3 (format, resolution) pairs, got %d", n);
}

ZTEST(pivariety, test_maps_greyscale_and_bayer_data_types)
{
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};

	zassert_ok(video_get_caps(cam, &caps));

	/* RAW10 (0x2b) + order 4 (GRAY) -> Y10P, not a Bayer format. */
	zassert_equal(caps.format_caps[0].pixelformat, VIDEO_PIX_FMT_Y10P);
	zassert_equal(caps.format_caps[0].width_min, 240);
	zassert_equal(caps.format_caps[0].height_min, 180);
	zassert_equal(caps.format_caps[1].pixelformat, VIDEO_PIX_FMT_Y10P);
	zassert_equal(caps.format_caps[1].width_min, 120);

	/* RAW8 (0x2a) + order 0 (BGGR) -> SBGGR8. */
	zassert_equal(caps.format_caps[2].pixelformat, VIDEO_PIX_FMT_SBGGR8);
	zassert_equal(caps.format_caps[2].width_min, 640);
}

ZTEST(pivariety, test_set_fmt_selects_module_indices)
{
	struct video_format fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_SBGGR8,
		.width = 640,
		.height = 480,
	};

	zassert_ok(video_set_format(cam, &fmt));

	/* SBGGR8 640x480 is the emulator's format 1, resolution 0. */
	zassert_equal(pivariety_emul_last_write(PIXFORMAT_INDEX_REG), 1);
	zassert_equal(pivariety_emul_last_write(RESOLUTION_INDEX_REG), 0);
}

ZTEST(pivariety, test_set_fmt_rejects_unadvertised_format)
{
	struct video_format fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width = 640,
		.height = 480,
	};

	zassert_equal(video_set_format(cam, &fmt), -ENOTSUP);
}

ZTEST(pivariety, test_get_fmt_returns_what_was_set)
{
	struct video_format set = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_Y10P,
		.width = 240,
		.height = 180,
	};
	struct video_format got = {.type = VIDEO_BUF_TYPE_OUTPUT};

	zassert_ok(video_set_format(cam, &set));
	zassert_ok(video_get_format(cam, &got));

	zassert_equal(got.pixelformat, VIDEO_PIX_FMT_Y10P);
	zassert_equal(got.width, 240);
	zassert_equal(got.height, 180);
}

ZTEST(pivariety, test_stream_start_stop_writes_mode_select)
{
	zassert_ok(video_stream_start(cam, VIDEO_BUF_TYPE_OUTPUT));
	zassert_equal(pivariety_emul_last_write(MODE_SELECT_REG), 1);

	zassert_ok(video_stream_stop(cam, VIDEO_BUF_TYPE_OUTPUT));
	zassert_equal(pivariety_emul_last_write(MODE_SELECT_REG), 0);
}

ZTEST(pivariety, test_link_frequency_comes_from_devicetree)
{
	int64_t freq = video_get_csi_link_freq(cam, 10, 2);

	/* Pivariety has no link-frequency register; it must come from the
	 * endpoint's link-frequencies property.
	 */
	zassert_equal(freq, 493500000, "got %lld", freq);
}

ZTEST(pivariety, test_maps_known_control_and_skips_unknown)
{
	struct video_ctrl_query cq = {.dev = cam, .id = VIDEO_CID_EXPOSURE};

	zassert_ok(video_query_ctrl(&cq), "exposure should be registered");
	zassert_equal(cq.range.min, 0);
	zassert_equal(cq.range.max, 65535);
	zassert_equal(cq.range.def, 100);

	/* The vendor control has no Zephyr CID and must not have been registered. */
	cq.id = 0x009a0999;
	zassert_not_equal(video_query_ctrl(&cq), 0, "vendor control should be skipped");
}

ZTEST(pivariety, test_set_ctrl_writes_id_then_value)
{
	struct video_control ctrl = {.id = VIDEO_CID_EXPOSURE, .val = 500};

	zassert_ok(video_set_ctrl(cam, &ctrl));
	zassert_equal(pivariety_emul_last_write(CTRL_ID_REG), 0x00980911);
	zassert_equal(pivariety_emul_last_write(CTRL_VALUE_REG), 500);
}

ZTEST_SUITE(pivariety, NULL, NULL, NULL, NULL, NULL);

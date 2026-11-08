/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Build check: the board's camera chain (LCD_CAM DVP + OV3660) links and
 * offers JPEG. Needs the board to run.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/ztest.h>

ZTEST(ov3660, test_jpeg_caps)
{
	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};

	zassert_true(device_is_ready(dev));
	zassert_ok(video_get_caps(dev, &caps));
	zassert_equal(caps.format_caps[0].pixelformat, VIDEO_PIX_FMT_JPEG);
}

ZTEST_SUITE(ov3660, NULL, NULL, NULL, NULL, NULL);

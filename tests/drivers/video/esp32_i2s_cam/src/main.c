/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * The ESP32 I2S camera driver's FIFO unpacking and JPEG framing helpers.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "esp32_i2s_cam_jpeg.h"

/* What the I2S FIFO holds for camera bytes b0, b1, ...: "00 bN 00 00" LE words. */
static void pack(uint8_t *dst, const uint8_t *src, size_t n)
{
	memset(dst, 0xAA, n * 4); /* padding must be ignored, whatever it holds */
	for (size_t i = 0; i < n; i++) {
		dst[i * 4 + 2] = src[i];
	}
}

ZTEST(esp32_i2s_cam, test_unpack)
{
	const uint8_t cam[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x01, 0x02, 0x03, 0x04};
	uint8_t fifo[sizeof(cam) * 4];
	uint8_t out[sizeof(cam)];

	pack(fifo, cam, sizeof(cam));
	zassert_equal(esp32_i2s_cam_unpack(out, fifo, sizeof(fifo)), sizeof(cam));
	zassert_mem_equal(out, cam, sizeof(cam));
}

ZTEST(esp32_i2s_cam, test_soi)
{
	const uint8_t good[] = {0xFF, 0xD8, 0xFF};
	const uint8_t bad[] = {0x00, 0xFF, 0xD8};

	zassert_true(esp32_i2s_cam_has_soi(good, sizeof(good)));
	zassert_false(esp32_i2s_cam_has_soi(bad, sizeof(bad)));
	zassert_false(esp32_i2s_cam_has_soi(good, 1));
}

ZTEST(esp32_i2s_cam, test_eoi)
{
	/* New frame ends at index 5; stale bytes from a bigger frame follow. */
	const uint8_t buf[] = {0xFF, 0xD8, 0x12, 0xFF, 0x00, 0xFF, 0xD9, 0x34, 0xFF, 0xD9};

	zassert_equal(esp32_i2s_cam_find_eoi(buf, sizeof(buf), 0), 7);
	/* Search start past the real EOI finds only the stale one. */
	zassert_equal(esp32_i2s_cam_find_eoi(buf, sizeof(buf), 7), 10);
	/* EOI straddling the end of the data is not found. */
	zassert_equal(esp32_i2s_cam_find_eoi(buf, 6, 0), -1);
	/* A stuffed FF 00 is not EOI. */
	zassert_equal(esp32_i2s_cam_find_eoi(buf, 5, 0), -1);
}

ZTEST_SUITE(esp32_i2s_cam, NULL, NULL, NULL, NULL, NULL);

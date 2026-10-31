/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-free helpers for the ESP32 I2S camera driver, kept in a header so
 * they can be unit tested on native_sim.
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_ESP32_I2S_CAM_JPEG_H_
#define ZEPHYR_DRIVERS_VIDEO_ESP32_I2S_CAM_JPEG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/*
 * In the SM_0A00_0B00 sampling mode the I2S FIFO stores each camera byte as
 * byte 2 of a little-endian 32-bit word: "00 s1 00 00, 00 s2 00 00, ...".
 * Returns the number of camera bytes written to dst (len / 4).
 */
static inline size_t esp32_i2s_cam_unpack(uint8_t *dst, const uint8_t *src, size_t len)
{
	size_t n = len / 4;

	for (size_t i = 0; i < n; i++) {
		dst[i] = src[i * 4 + 2];
	}

	return n;
}

static inline bool esp32_i2s_cam_has_soi(const uint8_t *buf, size_t len)
{
	return len >= 2 && buf[0] == 0xFF && buf[1] == 0xD8;
}

/*
 * Find the first EOI (FF D9) at or after `from`. Inside entropy-coded data a
 * 0xFF is always stuffed as FF 00, so the first FF D9 is the real end; a
 * forward search also ignores stale bytes a previous, larger frame left at
 * the end of the ring. Returns the length up to and including EOI, or -1.
 */
static inline ssize_t esp32_i2s_cam_find_eoi(const uint8_t *buf, size_t len, size_t from)
{
	for (size_t i = from; i + 1 < len; i++) {
		if (buf[i] == 0xFF && buf[i + 1] == 0xD9) {
			return (ssize_t)(i + 2);
		}
	}

	return -1;
}

#endif /* ZEPHYR_DRIVERS_VIDEO_ESP32_I2S_CAM_JPEG_H_ */

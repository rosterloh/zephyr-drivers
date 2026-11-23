/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief OpenTherm master over two GPIOs and a line adapter.
 *
 * Frames are 32-bit OpenTherm v2.2 messages: parity (bit 31), message
 * type (bits 30..28), spare (27..24), data ID (23..16) and value (15..0).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_OPENTHERM_H_
#define ZEPHYR_INCLUDE_DRIVERS_OPENTHERM_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** OpenTherm message types. */
enum opentherm_msg_type {
	OPENTHERM_READ_DATA = 0,
	OPENTHERM_WRITE_DATA = 1,
	OPENTHERM_INVALID_DATA = 2,
	OPENTHERM_READ_ACK = 4,
	OPENTHERM_WRITE_ACK = 5,
	OPENTHERM_DATA_INVALID = 6,
	OPENTHERM_UNKNOWN_DATA_ID = 7,
};

/**
 * @brief Build a frame with even parity.
 *
 * @param type Message type.
 * @param id Data ID.
 * @param value Data value.
 *
 * @return The 32-bit frame.
 */
static inline uint32_t opentherm_frame(enum opentherm_msg_type type, uint8_t id, uint16_t value)
{
	uint32_t frame = ((uint32_t)type << 28) | ((uint32_t)id << 16) | value;

	return frame | ((uint32_t)(__builtin_popcount(frame) & 1) << 31);
}

/** @brief Message type of a frame. */
static inline enum opentherm_msg_type opentherm_frame_type(uint32_t frame)
{
	return (enum opentherm_msg_type)((frame >> 28) & 0x7U);
}

/** @brief Data ID of a frame. */
static inline uint8_t opentherm_frame_id(uint32_t frame)
{
	return (uint8_t)(frame >> 16);
}

/** @brief Data value of a frame. */
static inline uint16_t opentherm_frame_value(uint32_t frame)
{
	return (uint16_t)frame;
}

/** @cond INTERNAL_HIDDEN */
__subsystem struct opentherm_driver_api {
	int (*transfer)(const struct device *dev, uint32_t request, uint32_t *response);
};
/** @endcond */

/**
 * @brief Send a request and wait for the slave's response.
 *
 * Blocks for the frame time plus the slave's reply time (up to about 1 s),
 * and keeps the 100 ms gap the spec requires between messages. The
 * response's message type is not checked: DATA-INVALID and UNKNOWN-DATAID
 * are valid replies the caller has to handle.
 *
 * @param dev OpenTherm device.
 * @param request Frame to send, normally from opentherm_frame().
 * @param response Received frame.
 *
 * @retval 0 Response received with valid parity.
 * @retval -ETIMEDOUT No complete response.
 * @retval -EBADMSG Response has bad parity or broken Manchester timing.
 * @retval -EIO Line driver error.
 */
static inline int opentherm_transfer(const struct device *dev, uint32_t request, uint32_t *response)
{
	return DEVICE_API_GET(opentherm, dev)->transfer(dev, request, response);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_OPENTHERM_H_ */

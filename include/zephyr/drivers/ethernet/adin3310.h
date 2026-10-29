/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ADIN3310_H_
#define ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ADIN3310_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADIN3310_PORT_COUNT 3U

int adin3310_get_port_count(const struct device *dev, uint8_t *port_count);
int adin3310_reset(const struct device *dev);
int adin3310_transfer(const struct device *dev, const uint8_t *tx, uint8_t *rx, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETHERNET_ADIN3310_H_ */

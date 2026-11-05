/*
 * OpenDisplay protocol 2.2 wire constants and the core/transport interface.
 *
 * Written from the published protocol description; values only.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_OPENDISPLAY_OD_INTERNAL_H_
#define ZEPHYR_SUBSYS_OPENDISPLAY_OD_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Response status byte (frame byte 0). */
#define OD_ACK           0x00U
#define OD_NACK          0xFFU
#define OD_AUTH_REQUIRED 0xFEU

/* Command opcodes, sent big-endian; responses echo the low byte. */
#define OD_CMD_REBOOT          0x000FU
#define OD_CMD_CONFIG_READ     0x0040U
#define OD_CMD_CONFIG_WRITE    0x0041U
#define OD_CMD_CONFIG_CHUNK    0x0042U
#define OD_CMD_FW_VERSION      0x0043U
#define OD_CMD_READ_MSD        0x0044U
#define OD_CMD_CONFIG_CLEAR    0x0045U
#define OD_CMD_AUTHENTICATE    0x0050U
#define OD_CMD_DW_START        0x0070U
#define OD_CMD_DW_DATA         0x0071U
#define OD_CMD_DW_END          0x0072U
/* Device-to-host notifications after DW_END. */
#define OD_EVT_REFRESH_OK      0x0073U
#define OD_EVT_REFRESH_TIMEOUT 0x0074U

#define OD_COMPANY_ID 0x2446U

/* Config container: [len:2 LE][version:1][packets...][crc:2 LE]. */
#define OD_CFG_HDR_LEN    3U
#define OD_CFG_CRC_LEN    2U
#define OD_CFG_VERSION    1U
/* Largest config data per CONFIG_WRITE/CONFIG_CHUNK frame. */
#define OD_CFG_CHUNK      200U
/* Largest CONFIG_READ notification, header included. */
#define OD_CFG_READ_FRAME 100U

/* Config packet types: [number:1][type:1][fixed-size body]. */
#define OD_PKT_SYSTEM        0x01U
#define OD_PKT_MANUFACTURER  0x02U
#define OD_PKT_POWER         0x04U
#define OD_PKT_DISPLAY       0x20U
#define OD_PKT_LED           0x21U
#define OD_PKT_SENSOR        0x23U
#define OD_PKT_DATA_BUS      0x24U
#define OD_PKT_BINARY_INPUTS 0x25U
#define OD_PKT_WIFI          0x26U
#define OD_PKT_SECURITY      0x27U
#define OD_PKT_TOUCH         0x28U
#define OD_PKT_BUZZER        0x29U
#define OD_PKT_NFC           0x2AU
#define OD_PKT_FLASH         0x2BU
#define OD_PKT_DATA_EXTENDED 0x2CU

#define OD_SYSTEM_LEN        22U
#define OD_MANUFACTURER_LEN  22U
#define OD_POWER_LEN         30U
#define OD_DISPLAY_LEN       46U
#define OD_WIFI_LEN          160U
#define OD_SECURITY_LEN      64U
#define OD_DATA_EXTENDED_LEN 288U

/* SystemConfig.communication_modes */
#define OD_COMM_BLE  0x01U
#define OD_COMM_WIFI 0x04U

/* DisplayConfig.display_technology */
#define OD_TECH_EPAPER 1U
#define OD_TECH_LCD    2U

/* DisplayConfig.color_scheme */
#define OD_COLOR_MONO 0U

/* DisplayConfig.transmission_modes */
#define OD_TX_STREAMING_ZLIB 0x01U
#define OD_TX_ZIP            0x02U
#define OD_TX_DIRECT_WRITE   0x08U
#define OD_TX_PIPE           0x10U

/** A link the core answers on. */
struct od_transport {
	/** Send one response or notification frame. */
	int (*send)(const struct od_transport *tp, const uint8_t *buf, size_t len);
};

/** Handle one request frame; replies go out through @p tp before it returns. */
void od_handle(const struct od_transport *tp, const uint8_t *req, size_t len);

/** Send [status][cmd low byte][data]. */
int od_reply(const struct od_transport *tp, uint8_t status, uint16_t cmd, const uint8_t *data,
	     size_t len);

/* Config (od_config.c); called with the core lock held. */
void od_config_read(const struct od_transport *tp);
void od_config_write(const struct od_transport *tp, uint16_t cmd, const uint8_t *data, size_t len);
void od_config_clear(const struct od_transport *tp);
/** Host-written packet body of @p type, or NULL. */
const uint8_t *od_config_packet(uint8_t type);

#endif /* ZEPHYR_SUBSYS_OPENDISPLAY_OD_INTERNAL_H_ */

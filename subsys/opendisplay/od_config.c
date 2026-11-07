/*
 * OpenDisplay config container.
 *
 * READ_CONFIG is generated: the hardware packets come from the zephyr,display
 * device and Kconfig, never from what a host wrote. WRITE_CONFIG keeps only the
 * host-owned packets (WiFi, security, extended identity) and stores them in
 * settings; everything else in a written config is validated and dropped.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "od_internal.h"

LOG_MODULE_DECLARE(opendisplay, CONFIG_OPENDISPLAY_LOG_LEVEL);

#define OD_PIN_NONE 0xFFU

static const struct device *const od_display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

/* Host-owned packets, in the order they are returned. */
static const struct {
	uint8_t type;
	uint16_t len;
	uint16_t offset;
} soft_pkts[] = {
	{OD_PKT_WIFI, OD_WIFI_LEN, 0},
	{OD_PKT_SECURITY, OD_SECURITY_LEN, OD_WIFI_LEN},
	{OD_PKT_DATA_EXTENDED, OD_DATA_EXTENDED_LEN, OD_WIFI_LEN + OD_SECURITY_LEN},
};

#define OD_SOFT_SIZE (OD_WIFI_LEN + OD_SECURITY_LEN + OD_DATA_EXTENDED_LEN)

static struct {
	uint8_t body[OD_SOFT_SIZE];
	uint8_t present; /* bit i: soft_pkts[i] */
} soft;

#define OD_GEN_SIZE                                                                                \
	(OD_CFG_HDR_LEN + 2 + OD_SYSTEM_LEN + 2 + OD_MANUFACTURER_LEN + 2 + OD_POWER_LEN + 2 +     \
	 OD_DISPLAY_LEN + 2 * ARRAY_SIZE(soft_pkts) + OD_SOFT_SIZE + OD_CFG_CRC_LEN)

BUILD_ASSERT(CONFIG_OPENDISPLAY_CONFIG_BUF_SIZE >= OD_GEN_SIZE,
	     "OPENDISPLAY_CONFIG_BUF_SIZE too small for READ_CONFIG");

/* Shared by READ_CONFIG and a chunked WRITE_CONFIG in progress. */
static uint8_t cfg_buf[CONFIG_OPENDISPLAY_CONFIG_BUF_SIZE];

static struct {
	size_t total;
	size_t received;
	bool active;
} wr;

static int pkt_len(uint8_t type)
{
	switch (type) {
	case OD_PKT_SYSTEM:
	case OD_PKT_MANUFACTURER:
	case OD_PKT_LED:
		return 22;
	case OD_PKT_POWER:
	case OD_PKT_SENSOR:
	case OD_PKT_DATA_BUS:
	case OD_PKT_BINARY_INPUTS:
		return 30;
	case OD_PKT_DISPLAY:
		return OD_DISPLAY_LEN;
	case OD_PKT_WIFI:
		return OD_WIFI_LEN;
	case OD_PKT_SECURITY:
		return OD_SECURITY_LEN;
	case OD_PKT_TOUCH:
	case OD_PKT_BUZZER:
	case OD_PKT_NFC:
	case OD_PKT_FLASH:
		return 32;
	case OD_PKT_DATA_EXTENDED:
		return OD_DATA_EXTENDED_LEN;
	default:
		return -ENOENT;
	}
}

/* CRC-16/CCITT-FALSE over the container with the length field taken as zero. */
static uint16_t cfg_crc(const uint8_t *blob, size_t len)
{
	static const uint8_t zero[2];
	uint16_t crc = crc16_itu_t(0xFFFFU, zero, sizeof(zero));

	return crc16_itu_t(crc, &blob[2], len - 2U - OD_CFG_CRC_LEN);
}

static uint8_t *put_hdr(uint8_t *p, uint8_t type, size_t len)
{
	p[0] = 0U;
	p[1] = type;
	memset(&p[2], 0, len);
	return &p[2];
}

uint8_t od_comm_modes(void)
{
	return (IS_ENABLED(CONFIG_OPENDISPLAY_BLE) ? OD_COMM_BLE : 0U) |
	       (IS_ENABLED(CONFIG_OPENDISPLAY_LAN) ? OD_COMM_WIFI : 0U);
}

static size_t build_config(uint8_t *buf)
{
	struct display_capabilities caps;
	uint8_t *p = &buf[OD_CFG_HDR_LEN];
	uint8_t *b;

	display_get_capabilities(od_display, &caps);

	buf[2] = OD_CFG_VERSION;

	b = put_hdr(p, OD_PKT_SYSTEM, OD_SYSTEM_LEN);
	sys_put_le16(CONFIG_OPENDISPLAY_IC_TYPE, &b[0]);
	b[2] = od_comm_modes();
	b[4] = OD_PIN_NONE;
	b[20] = OD_PIN_NONE;
	b[21] = OD_PIN_NONE;
	p = b + OD_SYSTEM_LEN;

	b = put_hdr(p, OD_PKT_MANUFACTURER, OD_MANUFACTURER_LEN);
	sys_put_le16(CONFIG_OPENDISPLAY_MANUFACTURER_ID, &b[0]);
	b[2] = CONFIG_OPENDISPLAY_BOARD_TYPE;
	p = b + OD_MANUFACTURER_LEN;

	b = put_hdr(p, OD_PKT_POWER, OD_POWER_LEN);
	b[0] = CONFIG_OPENDISPLAY_POWER_MODE;
	b[8] = OD_PIN_NONE;  /* battery sense */
	b[9] = OD_PIN_NONE;  /* battery sense enable */
	b[20] = OD_PIN_NONE; /* charge enable */
	b[21] = OD_PIN_NONE; /* charge state */
	p = b + OD_POWER_LEN;

	b = put_hdr(p, OD_PKT_DISPLAY, OD_DISPLAY_LEN);
	b[1] = (caps.screen_info & SCREEN_INFO_EPD) != 0U ? OD_TECH_EPAPER : OD_TECH_LCD;
	sys_put_le16(caps.x_resolution, &b[4]);
	sys_put_le16(caps.y_resolution, &b[6]);
	memset(&b[15], OD_PIN_NONE, 5); /* reset, busy, dc, cs, data */
	b[21] = OD_COLOR_MONO;
	b[22] = OD_TX_DIRECT_WRITE;
	memset(&b[23], OD_PIN_NONE, 8); /* clk + 7 reserved pins */
	p = b + OD_DISPLAY_LEN;

	for (size_t i = 0; i < ARRAY_SIZE(soft_pkts); i++) {
		if ((soft.present & BIT(i)) == 0U) {
			continue;
		}
		b = put_hdr(p, soft_pkts[i].type, soft_pkts[i].len);
		memcpy(b, &soft.body[soft_pkts[i].offset], soft_pkts[i].len);
		p = b + soft_pkts[i].len;
	}

	size_t len = (size_t)(p - buf) + OD_CFG_CRC_LEN;

	sys_put_le16((uint16_t)len, &buf[0]);
	sys_put_le16(cfg_crc(buf, len), p);

	return len;
}

void od_config_read(const struct od_transport *tp)
{
	uint8_t frame[OD_CFG_READ_FRAME];
	size_t len;
	size_t off = 0;
	uint16_t chunk = 0;

	/* The buffer is shared; a read abandons a half-written config. */
	wr.active = false;
	len = build_config(cfg_buf);

	while (off < len) {
		size_t hdr = 4U;
		size_t n;

		frame[0] = OD_ACK;
		frame[1] = (uint8_t)OD_CMD_CONFIG_READ;
		sys_put_le16(chunk, &frame[2]);
		if (chunk == 0U) {
			sys_put_le16((uint16_t)len, &frame[4]);
			hdr = 6U;
		}
		n = MIN(len - off, sizeof(frame) - hdr);
		memcpy(&frame[hdr], &cfg_buf[off], n);
		if (od_send(tp, frame, hdr + n) < 0) {
			return;
		}
		off += n;
		chunk++;
	}
}

static int soft_save(void)
{
	char key[sizeof("opendisplay/xx")];
	int err = 0;

	if (!IS_ENABLED(CONFIG_SETTINGS)) {
		return 0;
	}

	for (size_t i = 0; i < ARRAY_SIZE(soft_pkts); i++) {
		int ret;

		snprintk(key, sizeof(key), "opendisplay/%02x", soft_pkts[i].type);
		if ((soft.present & BIT(i)) != 0U) {
			ret = settings_save_one(key, &soft.body[soft_pkts[i].offset],
						soft_pkts[i].len);
		} else {
			ret = settings_delete(key);
		}
		if (ret < 0) {
			err = ret;
		}
	}

	return err;
}

/* Validate a whole container, then keep its host-owned packets. */
static int apply(const uint8_t *blob, size_t len)
{
	size_t end;
	size_t off;
	uint8_t present = 0U;

	if (len < OD_CFG_HDR_LEN + OD_CFG_CRC_LEN) {
		return -EINVAL;
	}
	if (sys_get_le16(&blob[len - OD_CFG_CRC_LEN]) != cfg_crc(blob, len)) {
		LOG_WRN("config CRC mismatch");
		return -EBADMSG;
	}

	end = len - OD_CFG_CRC_LEN;
	for (int pass = 0; pass < 2; pass++) {
		for (off = OD_CFG_HDR_LEN; off + 2U <= end;) {
			uint8_t type = blob[off + 1U];
			int plen = pkt_len(type);

			if (plen < 0) {
				/* Same as hosts: an unknown type ends the parse. */
				LOG_DBG("unknown packet 0x%02x, rest ignored", type);
				break;
			}
			off += 2U;
			if (off + (size_t)plen > end) {
				return -EINVAL;
			}
			for (size_t i = 0; pass == 1 && i < ARRAY_SIZE(soft_pkts); i++) {
				if (soft_pkts[i].type == type) {
					memcpy(&soft.body[soft_pkts[i].offset], &blob[off],
					       (size_t)plen);
					present |= BIT(i);
				}
			}
			off += (size_t)plen;
		}
	}

	soft.present = present;
	if (soft_save() < 0) {
		LOG_WRN("config not persisted");
	}
	if (IS_ENABLED(CONFIG_OPENDISPLAY_LAN)) {
		od_lan_config_changed();
	}
	LOG_INF("config written (%zu bytes)", len);

	return 0;
}

static void reply(const struct od_transport *tp, uint16_t cmd, int err)
{
	(void)od_reply(tp, err == 0 ? OD_ACK : OD_NACK, cmd, NULL, 0);
}

void od_config_write(const struct od_transport *tp, uint16_t cmd, const uint8_t *data, size_t len)
{
	if (cmd == OD_CMD_CONFIG_WRITE) {
		wr.active = false;
		if (len <= OD_CFG_CHUNK) {
			reply(tp, cmd, apply(data, len));
			return;
		}
		/* Chunked: [total:2 LE][first data bytes] */
		wr.total = sys_get_le16(data);
		wr.received = len - 2U;
		if (wr.total > sizeof(cfg_buf) || wr.received > wr.total) {
			LOG_WRN("config of %zu bytes rejected", wr.total);
			reply(tp, cmd, -ENOMEM);
			return;
		}
		memcpy(cfg_buf, &data[2], wr.received);
		wr.active = true;
		reply(tp, cmd, 0);
		return;
	}

	if (!wr.active || len > OD_CFG_CHUNK || wr.received + len > wr.total) {
		wr.active = false;
		reply(tp, cmd, -EINVAL);
		return;
	}
	memcpy(&cfg_buf[wr.received], data, len);
	wr.received += len;
	if (wr.received < wr.total) {
		reply(tp, cmd, 0);
		return;
	}
	wr.active = false;
	reply(tp, cmd, apply(cfg_buf, wr.total));
}

void od_config_abort(void)
{
	wr.active = false;
}

void od_config_clear(const struct od_transport *tp)
{
	wr.active = false;
	soft.present = 0U;
	reply(tp, OD_CMD_CONFIG_CLEAR, soft_save());
	if (IS_ENABLED(CONFIG_OPENDISPLAY_LAN)) {
		od_lan_config_changed();
	}
}

uint16_t od_config_server_port(void)
{
	const uint8_t *wifi = od_config_packet(OD_PKT_WIFI);

	return wifi != NULL ? sys_get_be16(&wifi[OD_WIFI_PORT_OFFSET]) : 0U;
}

const uint8_t *od_config_packet(uint8_t type)
{
	for (size_t i = 0; i < ARRAY_SIZE(soft_pkts); i++) {
		if (soft_pkts[i].type == type && (soft.present & BIT(i)) != 0U) {
			return &soft.body[soft_pkts[i].offset];
		}
	}

	return NULL;
}

#if defined(CONFIG_SETTINGS)
static int od_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	char key[3];

	for (size_t i = 0; i < ARRAY_SIZE(soft_pkts); i++) {
		snprintk(key, sizeof(key), "%02x", soft_pkts[i].type);
		if (!settings_name_steq(name, key, NULL)) {
			continue;
		}
		if (len != soft_pkts[i].len) {
			return -EINVAL;
		}
		if (read_cb(cb_arg, &soft.body[soft_pkts[i].offset], len) != (ssize_t)len) {
			return -EIO;
		}
		soft.present |= BIT(i);
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(opendisplay, "opendisplay", NULL, od_settings_set, NULL, NULL);
#endif

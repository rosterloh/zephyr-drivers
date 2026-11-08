/*
 * OpenDisplay command dispatcher: [cmd:2 BE][payload] in,
 * [status][cmd low byte][data] out. Transports call od_handle() from their own
 * thread; one lock serialises commands from all of them.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <zephyr/version.h>

#include "od_internal.h"

LOG_MODULE_REGISTER(opendisplay, CONFIG_OPENDISPLAY_LOG_LEVEL);

/* Longest reply built here: firmware version with a 40-character SHA. */
#define OD_REPLY_MAX 104U
#define OD_SHA_MAX   40U

K_MUTEX_DEFINE(od_lock);

static struct {
	struct opendisplay_status status;
	uint8_t loop;
} od_msd;

int od_reply(const struct od_transport *tp, uint8_t status, uint16_t cmd, const uint8_t *data,
	     size_t len)
{
	uint8_t buf[2 + OD_REPLY_MAX];

	if (len > OD_REPLY_MAX) {
		return -EMSGSIZE;
	}

	buf[0] = status;
	buf[1] = (uint8_t)cmd;
	if (len > 0U) {
		memcpy(&buf[2], data, len);
	}

	return od_send(tp, buf, len + 2U);
}

static void fw_version(const struct od_transport *tp)
{
	static const char sha[] = STRINGIFY(BUILD_VERSION);
	uint8_t buf[3 + OD_SHA_MAX + 1];
	size_t sha_len = MIN(strlen(sha), OD_SHA_MAX);

	buf[0] = CONFIG_OPENDISPLAY_FW_VERSION_MAJOR;
	buf[1] = CONFIG_OPENDISPLAY_FW_VERSION_MINOR;
	/* Hosts reject an empty SHA. */
	if (sha_len == 0U) {
		buf[2] = 1U;
		buf[3] = '0';
		sha_len = 1U;
	} else {
		buf[2] = (uint8_t)sha_len;
		memcpy(&buf[3], sha, sha_len);
	}
	buf[3 + sha_len] = CONFIG_OPENDISPLAY_FW_VERSION_PATCH;

	(void)od_reply(tp, OD_ACK, OD_CMD_FW_VERSION, buf, 4U + sha_len);
}

void opendisplay_get_msd(uint8_t msd[OPENDISPLAY_MSD_LEN])
{
	uint16_t batt;
	int32_t temp;

	k_mutex_lock(&od_lock, K_FOREVER);

	batt = MIN(od_msd.status.battery_mv / 10U, 0x1FFU);
	temp = CLAMP((od_msd.status.temperature_c + 40) * 2, 0, UINT8_MAX);

	sys_put_le16(OD_COMPANY_ID, msd);
	memcpy(&msd[2], od_msd.status.dynamic, OPENDISPLAY_MSD_DYNAMIC_LEN);
	msd[13] = (uint8_t)temp;
	msd[14] = (uint8_t)batt;
	msd[15] = (uint8_t)((batt >> 8) | (od_msd.loop << 4));

	k_mutex_unlock(&od_lock);
}

void opendisplay_set_status(const struct opendisplay_status *status)
{
	k_mutex_lock(&od_lock, K_FOREVER);
	od_msd.status = *status;
	od_msd.loop = (od_msd.loop + 1U) & 0x0FU;
	k_mutex_unlock(&od_lock);

	if (IS_ENABLED(CONFIG_OPENDISPLAY_BLE)) {
		od_ble_msd_changed();
	}
}

void od_disconnected(const struct od_transport *tp)
{
	k_mutex_lock(&od_lock, K_FOREVER);
	od_config_abort();
	od_direct_abort(tp);
	if (IS_ENABLED(CONFIG_OPENDISPLAY_SECURITY)) {
		od_sec_reset(tp);
	}
	k_mutex_unlock(&od_lock);
}

static void reboot_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (IS_ENABLED(CONFIG_REBOOT)) {
		sys_reboot(SYS_REBOOT_COLD);
	}
}

static K_WORK_DELAYABLE_DEFINE(reboot_work, reboot_handler);

/* [0x00][echo][marker], always plain. */
static void sec_reject(const struct od_transport *tp, uint16_t cmd, uint8_t marker)
{
	uint8_t buf[3] = {OD_ACK, (uint8_t)cmd, marker};

	(void)tp->send(tp, buf, sizeof(buf));
}

static void dispatch(const struct od_transport *tp, uint16_t cmd, const uint8_t *payload,
		     size_t plen)
{
	uint8_t msd[OPENDISPLAY_MSD_LEN];

	switch (cmd) {
	case OD_CMD_FW_VERSION:
		fw_version(tp);
		break;
	case OD_CMD_READ_MSD:
		opendisplay_get_msd(msd);
		(void)od_reply(tp, OD_ACK, cmd, msd, sizeof(msd));
		break;
	case OD_CMD_CONFIG_READ:
		od_config_read(tp);
		break;
	case OD_CMD_CONFIG_WRITE:
	case OD_CMD_CONFIG_CHUNK:
		od_config_write(tp, cmd, payload, plen);
		break;
	case OD_CMD_DW_START:
	case OD_CMD_DW_DATA:
	case OD_CMD_DW_END:
		od_direct(tp, cmd, payload, plen);
		break;
	case OD_CMD_CONFIG_CLEAR:
		od_config_clear(tp);
		break;
	case OD_CMD_REBOOT:
		if (IS_ENABLED(CONFIG_REBOOT)) {
			/* No reply: hosts treat the dropped link as success. */
			LOG_INF("reboot requested");
			(void)k_work_schedule(&reboot_work, K_MSEC(100));
		} else {
			(void)od_reply(tp, OD_NACK, cmd, NULL, 0);
		}
		break;
	default:
		LOG_DBG("unsupported cmd 0x%04x", cmd);
		(void)od_reply(tp, OD_NACK, cmd, NULL, 0);
		break;
	}
}

#if defined(CONFIG_OPENDISPLAY_SECURITY)
/* Encryption on: only the handshake is plain; everything else is enveloped. */
static void handle_secured(const struct od_transport *tp, uint16_t cmd, const uint8_t *req,
			   size_t len)
{
	const uint8_t *payload;
	size_t plen;
	int err;

	if (cmd == OD_CMD_AUTHENTICATE) {
		od_sec_authenticate(tp, req + 2, len - 2U);
		return;
	}
	if (cmd == OD_CMD_FW_VERSION) {
		fw_version(tp);
		return;
	}

	err = od_sec_open(tp, req, len, &payload, &plen);
	if (err != 0) {
		LOG_WRN("cmd 0x%04x rejected: %d", cmd, err);
		sec_reject(tp, cmd, err == -EACCES ? OD_AUTH_REQUIRED : OD_NACK);
		return;
	}
	dispatch(tp, cmd, payload, plen);
}
#endif

void od_handle(const struct od_transport *tp, const uint8_t *req, size_t len)
{
	uint16_t cmd;

	if (len < 2U) {
		return;
	}

	cmd = sys_get_be16(req);
	LOG_DBG("cmd 0x%04x len %zu", cmd, len - 2U);

	k_mutex_lock(&od_lock, K_FOREVER);
#if defined(CONFIG_OPENDISPLAY_SECURITY)
	if (!tp->secure && od_sec_enabled()) {
		handle_secured(tp, cmd, req, len);
		k_mutex_unlock(&od_lock);
		return;
	}
#endif
	if (cmd == OD_CMD_AUTHENTICATE) {
		/* Status 0x03: encryption not configured. */
		sec_reject(tp, cmd, 0x03U);
	} else {
		dispatch(tp, cmd, req + 2, len - 2U);
	}
	k_mutex_unlock(&od_lock);
}

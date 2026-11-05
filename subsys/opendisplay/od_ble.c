/*
 * OpenDisplay BLE transport: GATT service 0x2446 with one characteristic
 * (write, write without response, notify). Writes are queued from the BT RX
 * thread and handled on this transport's own thread, so a slow display write
 * never stalls the host stack. One central at a time.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/byteorder.h>

#include "od_internal.h"

LOG_MODULE_DECLARE(opendisplay, CONFIG_OPENDISPLAY_LOG_LEVEL);

/* Largest single GATT write value: ATT MTU 256 minus opcode and handle. */
#define OD_BLE_MAX_WRITE 253U

struct od_ble_frame {
	uint16_t len;
	uint8_t data[OD_BLE_MAX_WRITE];
};

K_MSGQ_DEFINE(od_ble_rxq, sizeof(struct od_ble_frame), CONFIG_OPENDISPLAY_BLE_RX_QUEUE, 4);

static struct bt_uuid_16 od_uuid = BT_UUID_INIT_16(OD_COMPANY_ID);

static struct k_spinlock conn_lock;
static struct bt_conn *od_conn;
static bool od_started;

static ssize_t od_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			uint16_t len, uint16_t offset, uint8_t flags);

BT_GATT_SERVICE_DEFINE(od_svc, BT_GATT_PRIMARY_SERVICE(&od_uuid),
		       BT_GATT_CHARACTERISTIC(&od_uuid.uuid,
					      BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP |
						      BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_WRITE, NULL, od_write, NULL),
		       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

static struct bt_conn *conn_get(void)
{
	k_spinlock_key_t key = k_spin_lock(&conn_lock);
	struct bt_conn *conn = od_conn != NULL ? bt_conn_ref(od_conn) : NULL;

	k_spin_unlock(&conn_lock, key);

	return conn;
}

static ssize_t od_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			uint16_t len, uint16_t offset, uint8_t flags)
{
	struct od_ble_frame frame;
	struct bt_conn *cur = conn_get();
	bool ours = cur == conn;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (cur != NULL) {
		bt_conn_unref(cur);
	}
	if (!ours) {
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
	}
	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len > sizeof(frame.data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	frame.len = len;
	memcpy(frame.data, buf, len);
	if (k_msgq_put(&od_ble_rxq, &frame, K_NO_WAIT) != 0) {
		LOG_WRN("rx queue full, write dropped");
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	}

	return len;
}

static int od_ble_send(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	struct bt_conn *conn = conn_get();
	int err;

	ARG_UNUSED(tp);

	if (conn == NULL) {
		return -ENOTCONN;
	}
	if (len > bt_gatt_get_mtu(conn) - 3U) {
		LOG_WRN("%zu-byte reply exceeds MTU %u", len, bt_gatt_get_mtu(conn));
		bt_conn_unref(conn);
		return -EMSGSIZE;
	}

	err = bt_gatt_notify(conn, &od_svc.attrs[1], buf, len);
	bt_conn_unref(conn);

	return err;
}

static const struct od_transport od_ble_tp = {.send = od_ble_send};

static int adv_start(void)
{
	uint8_t msd[OPENDISPLAY_MSD_LEN];
	const char *name = bt_get_name();
	struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
		BT_DATA(BT_DATA_MANUFACTURER_DATA, msd, sizeof(msd)),
	};
	struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, name, strlen(name)),
	};
	int err;

	opendisplay_get_msd(msd);
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err == -EALREADY) {
		err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	}
	if (err != 0) {
		LOG_ERR("advertising failed (%d)", err);
	}

	return err;
}

static void adv_work_handler(struct k_work *work)
{
	struct bt_conn *conn = conn_get();

	ARG_UNUSED(work);

	/* Advertising resumes from recycled() once the central is gone. */
	if (conn != NULL) {
		bt_conn_unref(conn);
		return;
	}
	(void)adv_start();
}

static K_WORK_DEFINE(adv_work, adv_work_handler);

void od_ble_msd_changed(void)
{
	if (od_started) {
		(void)k_work_submit(&adv_work);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	k_spinlock_key_t key;
	bool busy;

	if (err != 0U) {
		return;
	}

	key = k_spin_lock(&conn_lock);
	busy = od_conn != NULL;
	if (!busy) {
		od_conn = bt_conn_ref(conn);
	}
	k_spin_unlock(&conn_lock, key);

	if (busy) {
		LOG_INF("second central refused");
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	LOG_INF("connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	k_spinlock_key_t key = k_spin_lock(&conn_lock);
	struct bt_conn *old = NULL;

	if (od_conn == conn) {
		old = od_conn;
		od_conn = NULL;
	}
	k_spin_unlock(&conn_lock, key);

	if (old != NULL) {
		LOG_INF("disconnected (0x%02x)", reason);
		bt_conn_unref(old);
		od_disconnected(&od_ble_tp);
	}
}

static void recycled(void)
{
	/* Connectable advertising stops on connect; restart once the slot is free. */
	od_ble_msd_changed();
}

BT_CONN_CB_DEFINE(od_conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
};

int opendisplay_ble_start(void)
{
	int err;

	if (!bt_is_ready()) {
		return -EAGAIN;
	}

	if (IS_ENABLED(CONFIG_OPENDISPLAY_LAN)) {
		bt_addr_le_t id[CONFIG_BT_ID_MAX];
		size_t count = ARRAY_SIZE(id);
		uint8_t mac[6];

		/* mDNS "mac" is the advertised address, most significant byte first. */
		bt_id_get(id, &count);
		if (count > 0U) {
			sys_memcpy_swap(mac, id[0].a.val, sizeof(mac));
			od_lan_set_mac(mac);
		}
	}

	err = adv_start();
	if (err == 0) {
		od_started = true;
	}

	return err;
}

static void od_ble_thread(void *p1, void *p2, void *p3)
{
	static struct od_ble_frame frame;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		(void)k_msgq_get(&od_ble_rxq, &frame, K_FOREVER);
		od_handle(&od_ble_tp, frame.data, frame.len);
	}
}

K_THREAD_DEFINE(od_ble_tid, CONFIG_OPENDISPLAY_BLE_STACK_SIZE, od_ble_thread, NULL, NULL, NULL,
		CONFIG_OPENDISPLAY_BLE_THREAD_PRIORITY, 0, 0);

/*
 * OpenDisplay receiver: Home Assistant or py-opendisplay push images to the
 * zephyr,display device over BLE and, when networking is on, over LAN.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>

#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static void image_received(void)
{
	LOG_INF("image received, refreshing");
}

static void refresh_done(int err)
{
	LOG_INF("refresh done: %d", err);
}

static const struct opendisplay_callbacks callbacks = {
	.image_received = image_received,
	.refresh_done = refresh_done,
};

static void lan_start(void)
{
	struct net_if *iface = net_if_get_default();

	if (iface == NULL) {
		return;
	}
	if (IS_ENABLED(CONFIG_NET_DHCPV4)) {
		net_dhcpv4_start(iface);
	}
#if defined(CONFIG_WIFI_CREDENTIALS_CONNECT_STORED)
	if (net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, NULL, 0) != 0) {
		LOG_ERR("Wi-Fi connect failed");
	}
#endif
}

int main(void)
{
	int err;

	opendisplay_set_callbacks(&callbacks);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_subsys_init();
		if (err != 0) {
			LOG_ERR("settings: %d", err);
		}
	}

	if (IS_ENABLED(CONFIG_OPENDISPLAY_BLE)) {
		err = bt_enable(NULL);
		if (err != 0) {
			LOG_ERR("bt_enable: %d", err);
			return 0;
		}
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			(void)settings_load();
		}
		err = opendisplay_ble_start();
		if (err != 0) {
			LOG_ERR("advertising: %d", err);
		}
	} else if (IS_ENABLED(CONFIG_SETTINGS)) {
		(void)settings_load();
	}

	/* The LAN server starts by itself once the interface is up. */
	if (IS_ENABLED(CONFIG_OPENDISPLAY_LAN)) {
		lan_start();
	}

	return 0;
}

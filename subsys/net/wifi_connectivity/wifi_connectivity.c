/*
 * conn_mgr connectivity implementation for WiFi drivers that bind
 * CONNECTIVITY_WIFI_MGMT (the CONNECTIVITY_WIFI_MGMT_APPLICATION choice).
 *
 * Association parameters come from the wifi_credentials subsystem via
 * NET_REQUEST_WIFI_CONNECT_STORED. Persistence (CONN_MGR_IF_PERSISTENT)
 * is honoured by retrying failed attempts and unsolicited disconnects on
 * a delayable work item; a connect issued before any credentials are
 * stored keeps polling so the interface comes up once credentials are
 * provisioned (e.g. via `wifi cred add`).
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_connectivity, CONFIG_WIFI_CONNECTIVITY_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <zephyr/net/conn_mgr_connectivity_impl.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_credentials.h>
#include <zephyr/net/wifi_mgmt.h>

#define RECONNECT_DELAY K_SECONDS(CONFIG_WIFI_CONNECTIVITY_RECONNECT_DELAY_SECONDS)

static struct net_if *wifi_iface;
static struct net_mgmt_event_callback wifi_mgmt_cb;
static struct k_work_delayable reconnect_work;
static bool disconnect_requested;

static bool creds_available(void)
{
	return IS_ENABLED(CONFIG_WIFI_CREDENTIALS_STATIC) || !wifi_credentials_is_empty();
}

static void schedule_reconnect(struct net_if *iface)
{
	if (!conn_mgr_if_get_flag(iface, CONN_MGR_IF_PERSISTENT)) {
		return;
	}

	if (!net_if_is_admin_up(iface)) {
		return;
	}

	k_work_reschedule(&reconnect_work, RECONNECT_DELAY);
}

static void reconnect_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	/* conn_mgr_if_connect() brings the interface admin-up as needed, so we
	 * do not gate on admin state here: the deferred initial attempt runs
	 * before conn_mgr has brought the interface up.
	 */
	err = conn_mgr_if_connect(wifi_iface);
	if (err && err != -EALREADY) {
		LOG_DBG("Connect attempt failed: %d", err);
		k_work_reschedule(&reconnect_work, RECONNECT_DELAY);
	}
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				    struct net_if *iface)
{
	const struct wifi_status *status = cb->info;

	if (iface != wifi_iface) {
		return;
	}

	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		if (status->status) {
			LOG_WRN("Connection attempt failed (%d)", status->status);
			schedule_reconnect(iface);
		} else {
			LOG_INF("Connected");
			if (IS_ENABLED(CONFIG_WIFI_CONNECTIVITY_DISABLE_POWER_SAVE)) {
				struct wifi_ps_params ps = {
					.enabled = WIFI_PS_DISABLED,
					.type = WIFI_PS_PARAM_STATE,
				};
				int perr = net_mgmt(NET_REQUEST_WIFI_PS, iface, &ps, sizeof(ps));

				if (perr) {
					LOG_WRN("Failed to disable power save: %d", perr);
				}
			}
			if (IS_ENABLED(CONFIG_NET_DHCPV4)) {
				net_dhcpv4_start(iface);
			}
		}
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		if (disconnect_requested) {
			disconnect_requested = false;
		} else {
			LOG_INF("Disconnected (%d)", status->status);
			schedule_reconnect(iface);
		}
		break;
	default:
		break;
	}
}

static bool wc_has_config(struct conn_mgr_conn_binding *const binding)
{
	ARG_UNUSED(binding);

	return creds_available();
}

static int wc_connect(struct conn_mgr_conn_binding *const binding)
{
	int err;

	disconnect_requested = false;

	if (!creds_available()) {
		/* NET_REQUEST_WIFI_CONNECT_STORED silently does nothing without
		 * stored credentials; poll until some are provisioned.
		 */
		LOG_INF("No WiFi credentials stored yet");
		schedule_reconnect(binding->iface);
		return 0;
	}

	err = net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, binding->iface, NULL, 0);
	if (err == -EALREADY) {
		return 0;
	}

	return err;
}

static int wc_disconnect(struct conn_mgr_conn_binding *const binding)
{
	disconnect_requested = true;
	k_work_cancel_delayable(&reconnect_work);

	return net_mgmt(NET_REQUEST_WIFI_DISCONNECT, binding->iface, NULL, 0);
}

static void wc_init(struct conn_mgr_conn_binding *const binding)
{
	__ASSERT(wifi_iface == NULL, "only one WiFi interface is supported");
	wifi_iface = binding->iface;

	if (IS_ENABLED(CONFIG_WIFI_CONNECTIVITY_PERSISTENCE)) {
		binding->flags |= BIT(CONN_MGR_IF_PERSISTENT);
	}

	/* Drive the connect from our work queue instead of conn_mgr's automatic
	 * connect on iface-up: offloaded WiFi drivers (e.g. ESP32) can wedge if
	 * asked to associate before their stack has finished coming up at boot.
	 * Deferring the first attempt avoids that boot race.
	 */
	binding->flags |= BIT(CONN_MGR_IF_NO_AUTO_CONNECT);

	k_work_init_delayable(&reconnect_work, reconnect_handler);
	k_work_reschedule(&reconnect_work, RECONNECT_DELAY);

	net_mgmt_init_event_callback(&wifi_mgmt_cb, wifi_mgmt_event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT |
					     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_mgmt_cb);
}

static struct conn_mgr_conn_api wifi_connectivity_api = {
	.connect = wc_connect,
	.disconnect = wc_disconnect,
	.has_connection_config = wc_has_config,
	.init = wc_init,
};

CONN_MGR_CONN_DEFINE(CONNECTIVITY_WIFI_MGMT, &wifi_connectivity_api);

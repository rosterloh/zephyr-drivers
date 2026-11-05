/*
 * OpenDisplay LAN transport: [len:2 LE][frame] over TCP, one client at a time
 * (a new connection evicts the old one), dropped after an idle timeout.
 * Registers _opendisplay._tcp with DNS-SD while listening. With encryption
 * on it serves only TLS 1.2 PSK, on server_port + 1.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/hostname.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "od_internal.h"

LOG_MODULE_DECLARE(opendisplay, CONFIG_OPENDISPLAY_LOG_LEVEL);

#define OD_LAN_MAX_FRAME   4096U
#define OD_LAN_MAX_PAYLOAD (OD_LAN_MAX_FRAME - 2U)
#define OD_LAN_TX_MAX      128U
#define IDLE_MS            (CONFIG_OPENDISPLAY_LAN_IDLE_TIMEOUT * MSEC_PER_SEC)
/* Bounds the poll so a port change from WRITE_CONFIG is picked up. */
#define POLL_MS            1000

static struct {
	int fd;
	int64_t last_rx;
	size_t len;
	uint8_t rx[OD_LAN_MAX_FRAME];
} cl; /* fd set to -1 when the thread starts; kept out of .data */

static atomic_t reconfigure;

static int od_lan_send(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	uint8_t frame[2 + OD_LAN_TX_MAX];
	size_t off = 0;

	ARG_UNUSED(tp);

	if (cl.fd < 0) {
		return -ENOTCONN;
	}
	if (len > OD_LAN_TX_MAX) {
		return -EMSGSIZE;
	}

	sys_put_le16((uint16_t)len, frame);
	memcpy(&frame[2], buf, len);
	len += 2U;
	while (off < len) {
		ssize_t n = zsock_send(cl.fd, &frame[off], len - off, 0);

		if (n < 0) {
			return -errno;
		}
		off += (size_t)n;
	}

	return 0;
}

/* Plain TCP only while encryption is off, so the link never needs the envelope. */
static const struct od_transport od_lan_tp = {.send = od_lan_send, .secure = true};

#if defined(CONFIG_OPENDISPLAY_LAN_MDNS)
#define TXT_FW                                                                                     \
	"fw=" STRINGIFY(CONFIG_OPENDISPLAY_FW_VERSION_MAJOR) "." STRINGIFY(CONFIG_OPENDISPLAY_FW_VERSION_MINOR)
#define TXT_MAC    "mac=00:00:00:00:00:00"
#define TXT_TLS    "tls=0"
#define TXT_CM     "cm=00"
#define TXT_PV     "pv=2.2"
#define TXT_LEN(s) (sizeof(s))

/* Length-prefixed strings; mac and cm are rewritten in place. */
static char mdns_txt[TXT_LEN(TXT_MAC) + TXT_LEN(TXT_TLS) + TXT_LEN(TXT_FW) + TXT_LEN(TXT_CM) +
		     TXT_LEN(TXT_PV) + 1];
static char mdns_instance[32];
/* Network order; 0 hides the record while nothing listens. */
static uint16_t mdns_port;

DNS_SD_REGISTER_SERVICE(od_lan_sd, mdns_instance, "_opendisplay", "_tcp", "local", mdns_txt,
			&mdns_port);

static size_t txt_add(size_t pos, const char *kv)
{
	size_t len = strlen(kv);

	mdns_txt[pos] = (char)len;
	memcpy(&mdns_txt[pos + 1U], kv, len);

	return pos + 1U + len;
}

void od_lan_set_mac(const uint8_t mac[6])
{
	/* mac= is the first string; overwrite the hex in place. */
	char kv[sizeof(TXT_MAC)];

	snprintk(kv, sizeof(kv), "mac=%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2],
		 mac[3], mac[4], mac[5]);
	memcpy(&mdns_txt[1], kv, sizeof(TXT_MAC) - 1U);
}

static void mdns_init(void)
{
	char cm[sizeof(TXT_CM)];
	struct net_if *iface = net_if_get_default();
	struct net_linkaddr *ll = iface != NULL ? net_if_get_link_addr(iface) : NULL;
	size_t pos = 0;

	snprintk(cm, sizeof(cm), "cm=%02x", od_comm_modes());
	pos = txt_add(pos, TXT_MAC);
	pos = txt_add(pos, TXT_TLS);
	pos = txt_add(pos, TXT_FW);
	pos = txt_add(pos, cm);
	pos = txt_add(pos, TXT_PV);
	__ASSERT_NO_MSG(pos == sizeof(mdns_txt) - 1U);

	/* The BLE address is the identity; until BLE starts use the interface's. */
	if (!IS_ENABLED(CONFIG_OPENDISPLAY_BLE) && ll != NULL && ll->len == 6U) {
		od_lan_set_mac(ll->addr);
	}

	strncpy(mdns_instance,
		IS_ENABLED(CONFIG_NET_HOSTNAME_ENABLE) ? net_hostname_get() : "opendisplay",
		sizeof(mdns_instance) - 1U);
}

static void mdns_set_port(uint16_t port, bool tls)
{
	/* tls=x is the second string. */
	mdns_txt[sizeof(TXT_MAC) + sizeof(TXT_TLS) - 1U] = tls ? '1' : '0';
	mdns_port = sys_cpu_to_be16(port);
}
#else
void od_lan_set_mac(const uint8_t mac[6])
{
	ARG_UNUSED(mac);
}

static void mdns_init(void)
{
}

static void mdns_set_port(uint16_t port, bool tls)
{
	ARG_UNUSED(port);
	ARG_UNUSED(tls);
}
#endif /* CONFIG_OPENDISPLAY_LAN_MDNS */

void od_lan_config_changed(void)
{
	atomic_set(&reconfigure, 1);
}

/* Port to serve; with @p psk filled in when TLS is required. */
static uint16_t lan_port(uint8_t psk[16], bool *tls)
{
	uint16_t port;

	k_mutex_lock(&od_lock, K_FOREVER);
	port = od_config_server_port();
	if (port == 0U) {
		port = CONFIG_OPENDISPLAY_LAN_PORT;
	}
#if defined(CONFIG_OPENDISPLAY_SECURITY)
	*tls = od_sec_tls_psk(psk) == 0;
#else
	*tls = false;
#endif
	k_mutex_unlock(&od_lock);

	return *tls ? port + 1U : port;
}

#if defined(CONFIG_OPENDISPLAY_SECURITY)
static int set_psk(int fd, const uint8_t key[16])
{
	/* Credentials are kept by reference. */
	static uint8_t psk[16];
	static const char id[] = "opendisplay";
	static const sec_tag_t tags[] = {CONFIG_OPENDISPLAY_LAN_TLS_SEC_TAG};
	int err;

	(void)tls_credential_delete(tags[0], TLS_CREDENTIAL_PSK);
	(void)tls_credential_delete(tags[0], TLS_CREDENTIAL_PSK_ID);
	memcpy(psk, key, sizeof(psk));
	err = tls_credential_add(tags[0], TLS_CREDENTIAL_PSK, psk, sizeof(psk));
	if (err == 0) {
		err = tls_credential_add(tags[0], TLS_CREDENTIAL_PSK_ID, id, sizeof(id) - 1U);
	}
	if (err == 0 &&
	    zsock_setsockopt(fd, ZSOCK_SOL_TLS, ZSOCK_TLS_SEC_TAG_LIST, tags, sizeof(tags)) < 0) {
		err = -errno;
	}

	return err;
}
#endif

static int open_listener(uint16_t port, bool tls, const uint8_t psk[16])
{
	int one = 1;
	int fd;
#if defined(CONFIG_NET_IPV4)
#define LISTEN_FAMILY NET_AF_INET
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(port),
	};
#else
#define LISTEN_FAMILY NET_AF_INET6
	struct net_sockaddr_in6 addr = {
		.sin6_family = NET_AF_INET6,
		.sin6_port = net_htons(port),
	};
#endif

	fd = zsock_socket(LISTEN_FAMILY, NET_SOCK_STREAM,
			  tls ? NET_IPPROTO_TLS_1_2 : NET_IPPROTO_TCP);
	if (fd < 0) {
		return -errno;
	}
#if defined(CONFIG_OPENDISPLAY_SECURITY)
	if (tls) {
		int err = set_psk(fd, psk);

		if (err != 0) {
			zsock_close(fd);
			return err;
		}
	}
#else
	ARG_UNUSED(psk);
#endif
	(void)zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR, &one, sizeof(one));
	if (zsock_bind(fd, (struct net_sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(fd, 1) < 0) {
		int err = -errno;

		zsock_close(fd);
		return err;
	}

	return fd;
}

static void close_client(void)
{
	if (cl.fd < 0) {
		return;
	}
	zsock_close(cl.fd);
	cl.fd = -1;
	cl.len = 0U;
	od_disconnected(&od_lan_tp);
}

/* Handle every complete frame in rx[]; a bad length drops the client. */
static int process_rx(void)
{
	size_t pos = 0;

	while (cl.len - pos >= 2U) {
		uint16_t len = sys_get_le16(&cl.rx[pos]);

		if (len == 0U || len > OD_LAN_MAX_PAYLOAD) {
			LOG_WRN("bad frame length %u", len);
			return -EBADMSG;
		}
		if (cl.len - pos < 2U + len) {
			break;
		}
		od_handle(&od_lan_tp, &cl.rx[pos + 2U], len);
		if (cl.fd < 0) {
			return 0;
		}
		pos += 2U + len;
	}

	memmove(cl.rx, &cl.rx[pos], cl.len - pos);
	cl.len -= pos;

	return 0;
}

static void handle_rx(void)
{
	ssize_t n = zsock_recv(cl.fd, &cl.rx[cl.len], sizeof(cl.rx) - cl.len, 0);

	if (n <= 0) {
		LOG_DBG("client closed (%d)", n < 0 ? -errno : 0);
		close_client();
		return;
	}
	cl.len += (size_t)n;
	cl.last_rx = k_uptime_get();
	if (process_rx() < 0) {
		close_client();
	}
}

static void handle_accept(int listen_fd)
{
	int one = 1;
	/* shortcut: a TLS handshake blocks this thread, fine for one host at a time. */
	int fd = zsock_accept(listen_fd, NULL, NULL);

	if (fd < 0) {
		LOG_WRN("accept: %d", errno);
		return;
	}
	if (cl.fd >= 0) {
		LOG_INF("new client evicts the old one");
		close_client();
	}
	(void)zsock_setsockopt(fd, NET_IPPROTO_TCP, ZSOCK_TCP_NODELAY, &one, sizeof(one));
	cl.fd = fd;
	cl.len = 0U;
	cl.last_rx = k_uptime_get();
	LOG_DBG("client connected");
}

static void od_lan_thread(void *p1, void *p2, void *p3)
{
	int listen_fd = -1;
	uint16_t port = 0;
	bool tls = false;
	uint8_t psk[16] = {0};

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	cl.fd = -1;
	mdns_init();

	for (;;) {
		struct zsock_pollfd fds[2];
		int nfds = 0;
		int timeout = POLL_MS;

		if (atomic_cas(&reconfigure, 1, 0) || listen_fd < 0) {
			uint8_t want_psk[16] = {0};
			bool want_tls;
			uint16_t want = lan_port(want_psk, &want_tls);

			if (listen_fd >= 0 && (want != port || want_tls != tls ||
					       (tls && memcmp(want_psk, psk, sizeof(psk)) != 0))) {
				/* The client was let in under the old rules. */
				close_client();
				zsock_close(listen_fd);
				listen_fd = -1;
			}
			if (listen_fd < 0) {
				/* Binding the wildcard address works before an interface is up. */
				listen_fd = open_listener(want, want_tls, want_psk);
				if (listen_fd < 0) {
					LOG_ERR("listen on port %u: %d", want, listen_fd);
					mdns_set_port(0, false);
					k_sleep(K_SECONDS(1));
					continue;
				}
				port = want;
				tls = want_tls;
				memcpy(psk, want_psk, sizeof(psk));
				mdns_set_port(port, tls);
				LOG_INF("listening on port %u%s", port, tls ? " (TLS)" : "");
			}
		}

		fds[nfds++] = (struct zsock_pollfd){.fd = listen_fd, .events = ZSOCK_POLLIN};
		if (cl.fd >= 0) {
			int64_t left = cl.last_rx + IDLE_MS - k_uptime_get();

			if (left <= 0) {
				LOG_INF("client idle, closing");
				close_client();
				continue;
			}
			timeout = MIN(timeout, (int)left);
			fds[nfds++] = (struct zsock_pollfd){.fd = cl.fd, .events = ZSOCK_POLLIN};
		}

		if (zsock_poll(fds, nfds, timeout) < 0) {
			LOG_ERR("poll: %d", errno);
			k_sleep(K_MSEC(100));
			continue;
		}
		if (nfds > 1 && fds[1].revents != 0) {
			handle_rx();
		}
		if (fds[0].revents != 0) {
			handle_accept(listen_fd);
		}
	}
}

K_THREAD_DEFINE(od_lan_tid, CONFIG_OPENDISPLAY_LAN_STACK_SIZE, od_lan_thread, NULL, NULL, NULL,
		CONFIG_OPENDISPLAY_LAN_THREAD_PRIORITY, 0, 0);

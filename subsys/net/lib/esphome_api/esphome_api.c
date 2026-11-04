/*
 * ESPHome native API server (plaintext framing).
 *
 * One thread accepts clients and handles their requests. State updates from
 * esphome_api_publish_*() are sent from the caller's thread. A mutex guards the
 * client table, the shared encode buffers and entity state.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/esphome_api.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/version.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <api.pb.h>

#if defined(CONFIG_ESPHOME_API_NOISE)
#include <zephyr/settings/settings.h>
#include <zephyr/sys/base64.h>
#include <zephyr/sys/byteorder.h>
#include "noise.h"
#endif

LOG_MODULE_REGISTER(esphome_api, CONFIG_ESPHOME_API_LOG_LEVEL);

#if defined(CONFIG_NET_HOSTNAME_ENABLE)
#include <zephyr/net/hostname.h>
/* ESPHome's device name is the hostname; mDNS advertises it as <name>.local. */
#define DEVICE_NAME net_hostname_get()
#else
BUILD_ASSERT(sizeof(CONFIG_ESPHOME_API_DEVICE_NAME) <= sizeof(((HelloResponse *)0)->name),
	     "CONFIG_ESPHOME_API_DEVICE_NAME is limited to 31 characters");
#define DEVICE_NAME CONFIG_ESPHOME_API_DEVICE_NAME
#endif

#define VERSION_STR "Zephyr " KERNEL_VERSION_STRING

#define FRAME_HDR_MAX          (1 + 5 + 5)
/* Noise frame: 0x01, u16 length, then encrypted u16 type, u16 length, body, tag. */
#define NOISE_HDR              (3 + 4)
#define NOISE_HANDSHAKE_REJECT "Handshake MAC failure"
#define IDLE_TIMEOUT_MS        (CONFIG_ESPHOME_API_IDLE_TIMEOUT * MSEC_PER_SEC)

struct client {
	int fd;
	bool hello;
	bool subscribed;
	int64_t last_rx;
	size_t len;
	uint8_t rx[CONFIG_ESPHOME_API_RX_BUF_SIZE];
#if defined(CONFIG_ESPHOME_API_NOISE)
	enum {
		NOISE_HELLO,
		NOISE_HANDSHAKE,
		NOISE_READY
	} noise;
	struct noise_cs rx_cs;
	struct noise_cs tx_cs;
#endif
};

static struct client clients[CONFIG_ESPHOME_API_MAX_CLIENTS] = {
	[0 ... CONFIG_ESPHOME_API_MAX_CLIENTS - 1] = {.fd = -1},
};
static K_MUTEX_DEFINE(lock);

/* Encode buffers; only used with lock held. */
static uint8_t tx[FRAME_HDR_MAX + PROTO_API_PB_H_MAX_SIZE +
		  COND_CODE_1(CONFIG_ESPHOME_API_NOISE, (NOISE_TAG_LEN), (0))];
static union {
	HelloRequest hello_req;
	HelloResponse hello;
	DeviceInfoResponse info;
	ListEntitiesSensorResponse sensor;
	ListEntitiesBinarySensorResponse binary_sensor;
	ListEntitiesSwitchResponse sw;
	ListEntitiesButtonResponse button;
	SensorStateResponse sensor_state;
	BinarySensorStateResponse binary_state;
	SwitchStateResponse switch_state;
	SwitchCommandRequest switch_cmd;
	ButtonCommandRequest button_cmd;
} msg;

/* ESPHome's entity key: 32-bit FNV-1 of the object_id. */
static uint32_t entity_key(const struct esphome_entity *e)
{
	uint32_t hash = 2166136261U;

	for (const char *c = e->object_id; *c != '\0'; c++) {
		hash *= 16777619U;
		hash ^= (uint8_t)*c;
	}

	return hash;
}

static void set_str(char *dst, size_t size, const char *src)
{
	if (src == NULL) {
		dst[0] = '\0';
		return;
	}
	strncpy(dst, src, size - 1);
	dst[size - 1] = '\0';
}

#define SET_STR(dst, src) set_str(dst, sizeof(dst), src)

static size_t put_varint(uint8_t *buf, uint32_t val)
{
	size_t n = 0;

	do {
		buf[n] = (val & 0x7f) | (val > 0x7f ? 0x80 : 0);
		val >>= 7;
		n++;
	} while (val != 0);

	return n;
}

/* Returns -EAGAIN when more bytes are needed, -EBADMSG for an over-long varint. */
static int get_varint(const uint8_t *buf, size_t len, size_t *pos, uint32_t *val)
{
	uint32_t v = 0;

	for (int i = 0; i < 5; i++) {
		if (*pos >= len) {
			return -EAGAIN;
		}
		uint8_t b = buf[(*pos)++];

		v |= (uint32_t)(b & 0x7f) << (7 * i);
		if ((b & 0x80) == 0) {
			*val = v;
			return 0;
		}
	}

	return -EBADMSG;
}

/* Encode one frame for @p c into tx; returns its length or a negative errno. */
static int encode_frame(struct client *c, uint16_t type, const pb_msgdesc_t *fields,
			const void *body)
{
	size_t len = 0;
	size_t hdr = 0;
	pb_ostream_t os;

	if (fields != NULL && !pb_get_encoded_size(&len, fields, body)) {
		return -EINVAL;
	}

#if defined(CONFIG_ESPHOME_API_NOISE)
	ARG_UNUSED(put_varint);
	ARG_UNUSED(get_varint);
	hdr = NOISE_HDR;
#else
	ARG_UNUSED(c);
	tx[hdr++] = 0x00;
	hdr += put_varint(&tx[hdr], len);
	hdr += put_varint(&tx[hdr], type);
#endif

	if (fields != NULL) {
		os = pb_ostream_from_buffer(&tx[hdr], sizeof(tx) - hdr);
		if (!pb_encode(&os, fields, body)) {
			LOG_ERR("encode %u: %s", type, PB_GET_ERROR(&os));
			return -EINVAL;
		}
	}

#if defined(CONFIG_ESPHOME_API_NOISE)
	int ret;

	tx[0] = 0x01;
	sys_put_be16(4 + len + NOISE_TAG_LEN, &tx[1]);
	sys_put_be16(type, &tx[3]);
	sys_put_be16(len, &tx[5]);
	ret = noise_encrypt(&c->tx_cs, &tx[3], 4 + len);
	if (ret < 0) {
		return ret;
	}
	len += NOISE_TAG_LEN;
#endif

	return hdr + len;
}

/* shortcut: blocking send, so a stalled client stalls publishers; add SO_SNDTIMEO if seen. */
static int send_all(int fd, size_t len)
{
	size_t off = 0;

	while (off < len) {
		ssize_t n = zsock_send(fd, &tx[off], len - off, 0);

		if (n < 0) {
			return -errno;
		}
		off += n;
	}

	return 0;
}

static int send_msg(struct client *c, uint16_t type, const pb_msgdesc_t *fields, const void *body)
{
	int len = encode_frame(c, type, fields, body);

	return len < 0 ? len : send_all(c->fd, len);
}

static void close_client(struct client *c)
{
	if (c->fd < 0) {
		return;
	}
	LOG_DBG("client %d closed", c->fd);
	zsock_close(c->fd);
	c->fd = -1;
#if defined(CONFIG_ESPHOME_API_NOISE)
	noise_cs_destroy(&c->rx_cs);
	noise_cs_destroy(&c->tx_cs);
#endif
}

static void fill_state(struct esphome_entity *e, uint16_t *type, const pb_msgdesc_t **fields)
{
	uint32_t key = entity_key(e);

	switch (e->type) {
	case ESPHOME_SENSOR:
		msg.sensor_state = (SensorStateResponse){
			.key = key, .state = e->state.f, .missing_state = !e->has_state};
		*type = SensorStateResponse_msgid;
		*fields = SensorStateResponse_fields;
		break;
	case ESPHOME_BINARY_SENSOR:
		msg.binary_state = (BinarySensorStateResponse){
			.key = key, .state = e->state.b, .missing_state = !e->has_state};
		*type = BinarySensorStateResponse_msgid;
		*fields = BinarySensorStateResponse_fields;
		break;
	case ESPHOME_SWITCH:
		msg.switch_state = (SwitchStateResponse){
			.key = key, .state = e->state.b, .missing_state = !e->has_state};
		*type = SwitchStateResponse_msgid;
		*fields = SwitchStateResponse_fields;
		break;
	default:
		*fields = NULL;
		break;
	}
}

static int send_entity(struct client *c, struct esphome_entity *e)
{
	const char *name = e->name != NULL ? e->name : e->object_id;
	uint32_t key = entity_key(e);

	switch (e->type) {
	case ESPHOME_SENSOR:
		msg.sensor = (ListEntitiesSensorResponse){
			.key = key,
			.accuracy_decimals = e->accuracy_decimals,
			.state_class = e->state_class,
			.entity_category = e->entity_category,
		};
		SET_STR(msg.sensor.object_id, e->object_id);
		SET_STR(msg.sensor.name, name);
		SET_STR(msg.sensor.icon, e->icon);
		SET_STR(msg.sensor.unit_of_measurement, e->unit);
		SET_STR(msg.sensor.device_class, e->device_class);
		return send_msg(c, ListEntitiesSensorResponse_msgid,
				ListEntitiesSensorResponse_fields, &msg.sensor);
	case ESPHOME_BINARY_SENSOR:
		msg.binary_sensor = (ListEntitiesBinarySensorResponse){
			.key = key, .entity_category = e->entity_category};
		SET_STR(msg.binary_sensor.object_id, e->object_id);
		SET_STR(msg.binary_sensor.name, name);
		SET_STR(msg.binary_sensor.icon, e->icon);
		SET_STR(msg.binary_sensor.device_class, e->device_class);
		return send_msg(c, ListEntitiesBinarySensorResponse_msgid,
				ListEntitiesBinarySensorResponse_fields, &msg.binary_sensor);
	case ESPHOME_SWITCH:
		msg.sw = (ListEntitiesSwitchResponse){.key = key,
						      .entity_category = e->entity_category};
		SET_STR(msg.sw.object_id, e->object_id);
		SET_STR(msg.sw.name, name);
		SET_STR(msg.sw.icon, e->icon);
		SET_STR(msg.sw.device_class, e->device_class);
		return send_msg(c, ListEntitiesSwitchResponse_msgid,
				ListEntitiesSwitchResponse_fields, &msg.sw);
	case ESPHOME_BUTTON:
		msg.button = (ListEntitiesButtonResponse){.key = key,
							  .entity_category = e->entity_category};
		SET_STR(msg.button.object_id, e->object_id);
		SET_STR(msg.button.name, name);
		SET_STR(msg.button.icon, e->icon);
		SET_STR(msg.button.device_class, e->device_class);
		return send_msg(c, ListEntitiesButtonResponse_msgid,
				ListEntitiesButtonResponse_fields, &msg.button);
	}

	return -EINVAL;
}

static struct esphome_entity *find_entity(uint32_t key, enum esphome_entity_type type)
{
	STRUCT_SECTION_FOREACH(esphome_entity, e) {
		if (e->type == type && entity_key(e) == key) {
			return e;
		}
	}

	return NULL;
}

/* MAC of the default interface; zeros when it has none (loopback, offloaded sockets). */
static void get_mac(uint8_t mac[6])
{
	struct net_if *iface = net_if_get_default();
	struct net_linkaddr *ll = iface != NULL ? net_if_get_link_addr(iface) : NULL;

	memset(mac, 0, 6);
	if (ll != NULL && ll->len == 6) {
		memcpy(mac, ll->addr, 6);
	}
}

static int send_device_info(struct client *c)
{
	uint8_t mac[6];

	get_mac(mac);
	msg.info = (DeviceInfoResponse){0};
	SET_STR(msg.info.name, DEVICE_NAME);
	SET_STR(msg.info.friendly_name, CONFIG_ESPHOME_API_FRIENDLY_NAME);
	SET_STR(msg.info.manufacturer, CONFIG_ESPHOME_API_MANUFACTURER);
	SET_STR(msg.info.model, CONFIG_BOARD);
	SET_STR(msg.info.esphome_version, VERSION_STR);
	snprintk(msg.info.mac_address, sizeof(msg.info.mac_address),
		 "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	return send_msg(c, DeviceInfoResponse_msgid, DeviceInfoResponse_fields, &msg.info);
}

static bool decode(const uint8_t *buf, size_t len, const pb_msgdesc_t *fields, void *body)
{
	pb_istream_t is = pb_istream_from_buffer(buf, len);

	memset(body, 0, sizeof(msg));
	return pb_decode(&is, fields, body);
}

/* Handle one request; a negative return closes the connection. */
static int dispatch(struct client *c, uint32_t type, const uint8_t *buf, size_t len)
{
	struct esphome_entity *e;
	int ret;

	LOG_DBG("client %d: message %u, %zu bytes", c->fd, type, len);
	if (!c->hello && type != HelloRequest_msgid && type != PingRequest_msgid &&
	    type != DisconnectRequest_msgid) {
		LOG_WRN("message %u before hello", type);
		return -EPROTO;
	}

	switch (type) {
	case HelloRequest_msgid:
		if (!decode(buf, len, HelloRequest_fields, &msg.hello_req)) {
			return -EBADMSG;
		}
		LOG_INF("hello from \"%s\" (API %u.%u)", msg.hello_req.client_info,
			msg.hello_req.api_version_major, msg.hello_req.api_version_minor);
		c->hello = true;
		msg.hello = (HelloResponse){.api_version_major = ESPHOME_API_VERSION_MAJOR,
					    .api_version_minor = ESPHOME_API_VERSION_MINOR};
		SET_STR(msg.hello.server_info, VERSION_STR);
		SET_STR(msg.hello.name, DEVICE_NAME);
		return send_msg(c, HelloResponse_msgid, HelloResponse_fields, &msg.hello);
	case DisconnectRequest_msgid:
		(void)send_msg(c, DisconnectResponse_msgid, NULL, NULL);
		return -ECONNRESET;
	case DisconnectResponse_msgid:
		return -ECONNRESET;
	case PingRequest_msgid:
		return send_msg(c, PingResponse_msgid, NULL, NULL);
	case DeviceInfoRequest_msgid:
		return send_device_info(c);
	case DeviceCapabilitiesRequest_msgid:
		return send_msg(c, DeviceCapabilitiesResponse_msgid, NULL, NULL);
	case ListEntitiesRequest_msgid:
		STRUCT_SECTION_FOREACH(esphome_entity, ent) {
			ret = send_entity(c, ent);
			if (ret < 0) {
				return ret;
			}
		}
		return send_msg(c, ListEntitiesDoneResponse_msgid, NULL, NULL);
	case SubscribeStatesRequest_msgid:
		c->subscribed = true;
		STRUCT_SECTION_FOREACH(esphome_entity, ent) {
			uint16_t t;
			const pb_msgdesc_t *f;

			fill_state(ent, &t, &f);
			if (f == NULL) {
				continue;
			}
			ret = send_msg(c, t, f, &msg);
			if (ret < 0) {
				return ret;
			}
		}
		return 0;
	case SwitchCommandRequest_msgid:
		if (!decode(buf, len, SwitchCommandRequest_fields, &msg.switch_cmd)) {
			return -EBADMSG;
		}
		e = find_entity(msg.switch_cmd.key, ESPHOME_SWITCH);
		if (e != NULL && e->on_switch != NULL) {
			e->on_switch(e, msg.switch_cmd.state);
		}
		return 0;
	case ButtonCommandRequest_msgid:
		if (!decode(buf, len, ButtonCommandRequest_fields, &msg.button_cmd)) {
			return -EBADMSG;
		}
		e = find_entity(msg.button_cmd.key, ESPHOME_BUTTON);
		if (e != NULL && e->on_press != NULL) {
			e->on_press(e);
		}
		return 0;
	default:
		/* Subscriptions this server has nothing for (logs, HA states, ...). */
		LOG_DBG("ignoring message %u", type);
		return 0;
	}
}

#if defined(CONFIG_ESPHOME_API_NOISE)
static uint8_t psk[NOISE_KEY_LEN];
static bool psk_set;

/* An unencrypted 0x01 frame; used for the hello and handshake. */
static int send_raw(struct client *c, const void *a, size_t a_len, const void *b, size_t b_len)
{
	tx[0] = 0x01;
	sys_put_be16(a_len + b_len, &tx[1]);
	memcpy(&tx[3], a, a_len);
	memcpy(&tx[3 + a_len], b, b_len);

	return send_all(c->fd, 3 + a_len + b_len);
}

/* Handshake failure the client shows to the user; aioesphomeapi maps the MAC one to a bad key. */
static int send_reject(struct client *c, const char *reason)
{
	LOG_WRN("client %d: %s", c->fd, reason);
	(void)send_raw(c, "\x01", 1, reason, strlen(reason));

	return -EPROTO;
}

static int send_server_hello(struct client *c)
{
	/* Chosen protocol 0x01, then name and lowercase MAC, each NUL terminated. */
	char buf[1 + sizeof(((HelloResponse *)0)->name) + 13];
	uint8_t mac[6];
	int n;

	get_mac(mac);
	n = snprintk(buf, sizeof(buf), "\x01%.31s%c%02x%02x%02x%02x%02x%02x", DEVICE_NAME, 0,
		     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	return send_raw(c, buf, MIN(n, sizeof(buf) - 1) + 1, "", 0);
}

static int noise_frame(struct client *c, uint8_t *buf, size_t len)
{
	uint8_t msg2[1 + NOISE_HS_LEN] = {0x00};
	psa_key_id_t e;
	uint32_t type;
	size_t mlen;
	int ret;

	switch (c->noise) {
	case NOISE_HELLO:
		/* ClientHello carries nothing this server uses. */
		c->noise = NOISE_HANDSHAKE;
		return send_server_hello(c);
	case NOISE_HANDSHAKE:
		if (!psk_set) {
			return send_reject(c, "No encryption key set");
		}
		if (len != 1 + NOISE_HS_LEN || buf[0] != 0x00) {
			return send_reject(c, "Bad handshake packet len");
		}
		ret = noise_generate_e(&e, &msg2[1]);
		ret = ret ?: noise_respond(psk, &buf[1], e, &msg2[1], &c->rx_cs, &c->tx_cs);
		if (ret == -EBADMSG) {
			return send_reject(c, NOISE_HANDSHAKE_REJECT);
		}
		if (ret < 0) {
			return send_reject(c, "Handshake error");
		}
		c->noise = NOISE_READY;
		return send_raw(c, msg2, sizeof(msg2), "", 0);
	default:
		if (noise_decrypt(&c->rx_cs, buf, len) < 0 || len < NOISE_TAG_LEN + 4) {
			LOG_WRN("client %d: decrypt failed", c->fd);
			return -EBADMSG;
		}
		type = sys_get_be16(buf);
		mlen = sys_get_be16(&buf[2]);
		if (mlen > len - NOISE_TAG_LEN - 4) {
			return -EBADMSG;
		}
		return dispatch(c, type, &buf[4], mlen);
	}
}

/* Parse every complete frame in the client's buffer. */
static int process_rx(struct client *c)
{
	while (c->len > 0 && c->fd >= 0) {
		size_t len;
		int ret;

		if (c->rx[0] != 0x01) {
			/* aioesphomeapi tells a plaintext user the device needs encryption. */
			return send_reject(c, "Bad indicator byte");
		}
		if (c->len < 3) {
			return 0;
		}
		len = sys_get_be16(&c->rx[1]);
		if (len > sizeof(c->rx) - 3) {
			LOG_WRN("oversized frame");
			return -EMSGSIZE;
		}
		if (c->len - 3 < len) {
			return 0;
		}

		ret = noise_frame(c, &c->rx[3], len);
		if (ret < 0) {
			return ret;
		}

		memmove(c->rx, &c->rx[3 + len], c->len - 3 - len);
		c->len -= 3 + len;
	}

	return 0;
}
#else
/* Parse every complete frame in the client's buffer. */
static int process_rx(struct client *c)
{
	/* A command callback may publish, fail a send and close this client. */
	while (c->len > 0 && c->fd >= 0) {
		size_t pos = 1;
		uint32_t len, type;
		int ret;

		if (c->rx[0] != 0x00) {
			/* 0x01 is a Noise client; CONFIG_ESPHOME_API_NOISE is off. */
			LOG_WRN("bad preamble 0x%02x", c->rx[0]);
			return -EPROTO;
		}
		ret = get_varint(c->rx, c->len, &pos, &len);
		if (ret == 0) {
			ret = get_varint(c->rx, c->len, &pos, &type);
		}
		if (ret == -EAGAIN) {
			return 0;
		}
		if (ret < 0 || len > sizeof(c->rx) - pos) {
			LOG_WRN("bad or oversized frame");
			return -EMSGSIZE;
		}
		if (c->len - pos < len) {
			return 0;
		}

		ret = dispatch(c, type, &c->rx[pos], len);
		if (ret < 0) {
			return ret;
		}

		pos += len;
		memmove(c->rx, &c->rx[pos], c->len - pos);
		c->len -= pos;
	}

	return 0;
}
#endif /* CONFIG_ESPHOME_API_NOISE */

static void handle_rx(struct client *c)
{
	ssize_t n = zsock_recv(c->fd, &c->rx[c->len], sizeof(c->rx) - c->len, 0);

	if (n <= 0) {
		close_client(c);
		return;
	}

	c->len += n;
	c->last_rx = k_uptime_get();
	if (process_rx(c) < 0) {
		close_client(c);
	}
}

static void handle_accept(int listen_fd)
{
	int one = 1;
	int fd = zsock_accept(listen_fd, NULL, NULL);

	if (fd < 0) {
		LOG_ERR("accept: %d", errno);
		return;
	}

	ARRAY_FOR_EACH_PTR(clients, c) {
		if (c->fd < 0) {
			*c = (struct client){.fd = fd, .last_rx = k_uptime_get()};
			/* Frames are small and latency matters more than segments. */
			(void)zsock_setsockopt(fd, NET_IPPROTO_TCP, ZSOCK_TCP_NODELAY, &one,
					       sizeof(one));
			LOG_DBG("client %d connected", fd);
			return;
		}
	}

	LOG_WRN("no free client slot");
	zsock_close(fd);
}

static int open_listener(void)
{
	int one = 1;
	int fd;
/* One IPv6 socket serves IPv4 too when v4-mapped addresses are on (as the HTTP server does). */
#if defined(CONFIG_NET_IPV6) &&                                                                    \
	(defined(CONFIG_NET_IPV4_MAPPING_TO_IPV6) || !defined(CONFIG_NET_IPV4))
#define LISTEN_FAMILY NET_AF_INET6
	struct net_sockaddr_in6 addr = {
		.sin6_family = NET_AF_INET6,
		.sin6_port = net_htons(CONFIG_ESPHOME_API_PORT),
	};
#else
#define LISTEN_FAMILY NET_AF_INET
	/* shortcut: without v4 mapping only IPv4 is served; add a v6 listener if needed. */
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(CONFIG_ESPHOME_API_PORT),
	};
#endif

	fd = zsock_socket(LISTEN_FAMILY, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	if (fd < 0) {
		return -errno;
	}
	(void)zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR, &one, sizeof(one));
	if (IS_ENABLED(CONFIG_NET_IPV4_MAPPING_TO_IPV6) && LISTEN_FAMILY == NET_AF_INET6) {
		int off = 0;

		(void)zsock_setsockopt(fd, NET_IPPROTO_IPV6, ZSOCK_IPV6_V6ONLY, &off, sizeof(off));
	}
	if (zsock_bind(fd, (struct net_sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(fd, CONFIG_ESPHOME_API_MAX_CLIENTS) < 0) {
		int err = -errno;

		zsock_close(fd);
		return err;
	}

	return fd;
}

static void server_thread(void *p1, void *p2, void *p3)
{
	struct zsock_pollfd fds[1 + CONFIG_ESPHOME_API_MAX_CLIENTS];
	struct client *owner[ARRAY_SIZE(fds)];
	int listen_fd;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Binding the wildcard address works before an interface is up. */
	while ((listen_fd = open_listener()) < 0) {
		LOG_ERR("listen on port %d: %d", CONFIG_ESPHOME_API_PORT, listen_fd);
		k_sleep(K_SECONDS(1));
	}
	LOG_INF("listening on port %d", CONFIG_ESPHOME_API_PORT);
#if defined(CONFIG_ESPHOME_API_NOISE)
	if (psa_crypto_init() != PSA_SUCCESS) {
		LOG_ERR("PSA crypto init failed");
		return;
	}
	if (!psk_set) {
		LOG_WRN("no encryption key: clients are refused until esphome_api_set_psk()");
	}
#endif

	while (true) {
		int64_t now = k_uptime_get();
		int timeout = -1;
		int nfds = 1;

		fds[0] = (struct zsock_pollfd){.fd = listen_fd, .events = ZSOCK_POLLIN};

		k_mutex_lock(&lock, K_FOREVER);
		ARRAY_FOR_EACH_PTR(clients, c) {
			if (c->fd < 0) {
				continue;
			}
			int64_t left = c->last_rx + IDLE_TIMEOUT_MS - now;

			if (left <= 0) {
				LOG_INF("client %d idle, closing", c->fd);
				close_client(c);
				continue;
			}
			timeout = timeout < 0 ? (int)left : MIN(timeout, (int)left);
			owner[nfds] = c;
			fds[nfds++] = (struct zsock_pollfd){.fd = c->fd, .events = ZSOCK_POLLIN};
		}
		k_mutex_unlock(&lock);

		if (zsock_poll(fds, nfds, timeout) < 0) {
			LOG_ERR("poll: %d", errno);
			k_sleep(K_MSEC(100));
			continue;
		}

		k_mutex_lock(&lock, K_FOREVER);
		for (int i = 1; i < nfds; i++) {
			if (fds[i].revents != 0 && owner[i]->fd == fds[i].fd) {
				handle_rx(owner[i]);
			}
		}
		if (fds[0].revents & ZSOCK_POLLIN) {
			handle_accept(listen_fd);
		}
		k_mutex_unlock(&lock);
	}
}

#if defined(CONFIG_ESPHOME_API_MDNS)
#include <zephyr/net/dns_sd.h>

/* TXT keys Home Assistant and ESPHome tools read; "mac" is required for discovery. */
#define TXT_FRIENDLY "friendly_name=" CONFIG_ESPHOME_API_FRIENDLY_NAME
#define TXT_VERSION  "version=" VERSION_STR
#define TXT_MAC      "mac=000000000000"
#define TXT_BOARD    "board=" CONFIG_BOARD
#define TXT_NOISE    "api_encryption=Noise_NNpsk0_25519_ChaChaPoly_SHA256"
#if defined(CONFIG_WIFI)
#define TXT_NETWORK "network=wifi"
#elif defined(CONFIG_NET_L2_ETHERNET)
#define TXT_NETWORK "network=ethernet"
#endif

/* Each TXT string is a length byte followed by "key=value". */
#define TXT_LEN(kv) sizeof(kv)
#define TXT_SIZE                                                                                   \
	((sizeof(CONFIG_ESPHOME_API_FRIENDLY_NAME) > 1 ? TXT_LEN(TXT_FRIENDLY) : 0) +              \
	 TXT_LEN(TXT_VERSION) + TXT_LEN(TXT_MAC) + TXT_LEN(TXT_BOARD) +                            \
	 COND_CODE_1(CONFIG_ESPHOME_API_NOISE, (TXT_LEN(TXT_NOISE)), (0)) +                        \
	 COND_CODE_1(IS_ENABLED(CONFIG_WIFI), (TXT_LEN("network=wifi")),                           \
		     (COND_CODE_1(IS_ENABLED(CONFIG_NET_L2_ETHERNET),                              \
				  (TXT_LEN("network=ethernet")), (0)))))

BUILD_ASSERT(sizeof(TXT_FRIENDLY) <= 256 && sizeof(TXT_BOARD) <= 256, "TXT string too long");

/* Filled at init; the record is skipped while the instance name is still empty. */
static char mdns_txt[TXT_SIZE + 1];
static char mdns_instance[sizeof(((HelloResponse *)0)->name)];

DNS_SD_REGISTER_TCP_SERVICE(esphome_api_sd, mdns_instance, "_esphomelib", "local", mdns_txt,
			    CONFIG_ESPHOME_API_PORT);

static size_t txt_add(size_t pos, const char *kv)
{
	size_t len = strlen(kv);

	mdns_txt[pos] = len;
	memcpy(&mdns_txt[pos + 1], kv, len);

	return pos + 1 + len;
}

static int mdns_init(void)
{
	char mac_kv[sizeof(TXT_MAC)];
	uint8_t mac[6];
	size_t pos = 0;

	if (sizeof(CONFIG_ESPHOME_API_FRIENDLY_NAME) > 1) {
		pos = txt_add(pos, TXT_FRIENDLY);
	}
	pos = txt_add(pos, TXT_VERSION);
	get_mac(mac);
	/* Lowercase without separators, as ESPHome sends it. */
	snprintk(mac_kv, sizeof(mac_kv), "mac=%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2],
		 mac[3], mac[4], mac[5]);
	pos = txt_add(pos, mac_kv);
	pos = txt_add(pos, TXT_BOARD);
#if defined(TXT_NETWORK)
	pos = txt_add(pos, TXT_NETWORK);
#endif
#if defined(CONFIG_ESPHOME_API_NOISE)
	pos = txt_add(pos, TXT_NOISE);
#endif
	__ASSERT_NO_MSG(pos == TXT_SIZE);

	if (strlen(DEVICE_NAME) >= sizeof(mdns_instance)) {
		LOG_WRN("hostname \"%s\" is longer than ESPHome's 31 characters", DEVICE_NAME);
	}
	set_str(mdns_instance, sizeof(mdns_instance), DEVICE_NAME);

	return 0;
}

/* After drivers, so the interface has its MAC. */
SYS_INIT(mdns_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif /* CONFIG_ESPHOME_API_MDNS */

K_THREAD_DEFINE(esphome_api_tid, CONFIG_ESPHOME_API_STACK_SIZE, server_thread, NULL, NULL, NULL,
		CONFIG_ESPHOME_API_THREAD_PRIORITY, 0, 0);

static int publish(struct esphome_entity *e)
{
	uint16_t type;
	const pb_msgdesc_t *fields;

	fill_state(e, &type, &fields);

	/* Encoded per client: with Noise each has its own key and nonce. */
	ARRAY_FOR_EACH_PTR(clients, c) {
		if (c->fd >= 0 && c->subscribed && send_msg(c, type, fields, &msg) < 0) {
			close_client(c);
		}
	}

	return 0;
}

int esphome_api_publish_float(struct esphome_entity *entity, float state)
{
	int ret;

	__ASSERT_NO_MSG(!k_is_in_isr());
	if (entity->type != ESPHOME_SENSOR) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);
	entity->state.f = state;
	entity->has_state = true;
	ret = publish(entity);
	k_mutex_unlock(&lock);

	return ret;
}

int esphome_api_publish_bool(struct esphome_entity *entity, bool state)
{
	int ret;

	__ASSERT_NO_MSG(!k_is_in_isr());
	if (entity->type != ESPHOME_BINARY_SENSOR && entity->type != ESPHOME_SWITCH) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);
	entity->state.b = state;
	entity->has_state = true;
	ret = publish(entity);
	k_mutex_unlock(&lock);

	return ret;
}

#if defined(CONFIG_ESPHOME_API_NOISE)
int esphome_api_set_psk(const uint8_t key[32])
{
	k_mutex_lock(&lock, K_FOREVER);
	memcpy(psk, key, sizeof(psk));
	psk_set = true;
	k_mutex_unlock(&lock);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		return settings_save_one("esphome_api/psk", key, sizeof(psk));
	}
	return 0;
}

static int psk_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(name, "psk") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(psk) || read_cb(cb_arg, psk, sizeof(psk)) != sizeof(psk)) {
		return -EINVAL;
	}
	psk_set = true;

	return 0;
}

#if defined(CONFIG_SETTINGS)
SETTINGS_STATIC_HANDLER_DEFINE(esphome_api, "esphome_api", NULL, psk_settings_set, NULL, NULL);
#endif

/* Development key from Kconfig; a key from settings, loaded later, replaces it. */
static int psk_init(void)
{
	size_t olen;

	ARG_UNUSED(psk_settings_set);
	if (sizeof(CONFIG_ESPHOME_API_NOISE_PSK) == 1) {
		return 0;
	}
	if (base64_decode(psk, sizeof(psk), &olen, CONFIG_ESPHOME_API_NOISE_PSK,
			  sizeof(CONFIG_ESPHOME_API_NOISE_PSK) - 1) != 0 ||
	    olen != sizeof(psk)) {
		LOG_ERR("CONFIG_ESPHOME_API_NOISE_PSK is not a base64 32-byte key");
		return 0;
	}
	psk_set = true;

	return 0;
}

SYS_INIT(psk_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif /* CONFIG_ESPHOME_API_NOISE */

/*
 * Drives the server over loopback the way aioesphomeapi does.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/net/esphome_api.h>
#include <zephyr/net/socket.h>
#include <zephyr/ztest.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <api.pb.h>

/* FNV-1 of the object_id, as ESPHome computes it (checked with a Python port). */
#define KEY_TEMPERATURE 0x35a123f9
#define KEY_STOPPED     0xad23af94
#define KEY_RELAY       0xd3a92fe4
#define KEY_RESET       0xef75803a

#if defined(CONFIG_NET_HOSTNAME_ENABLE)
#define DEVICE_NAME CONFIG_NET_HOSTNAME
#else
#define DEVICE_NAME CONFIG_ESPHOME_API_DEVICE_NAME
#endif

static int relay_calls;
static bool relay_last;
static int reset_calls;

static void relay_cb(struct esphome_entity *e, bool state)
{
	relay_calls++;
	relay_last = state;
	/* Confirm from inside the callback, as an app would. */
	zassert_ok(esphome_api_publish_bool(e, state));
}

static void reset_cb(struct esphome_entity *e)
{
	reset_calls++;
}

ESPHOME_SENSOR_DEFINE(temperature, .name = "Temperature", .unit = "°C",
		      .device_class = "temperature", .state_class = ESPHOME_STATE_CLASS_MEASUREMENT,
		      .accuracy_decimals = 1);
ESPHOME_BINARY_SENSOR_DEFINE(stopped, .name = "Stopped");
ESPHOME_SWITCH_DEFINE(relay, .on_switch = relay_cb);
ESPHOME_BUTTON_DEFINE(reset, .name = "Reset", .on_press = reset_cb);

static uint8_t buf[512];
static int open_fd = -1;

static int connect_to(int family, const char *ip)
{
	struct net_sockaddr_storage ss = {0};
	struct net_sockaddr_in *in = (struct net_sockaddr_in *)&ss;
	struct net_sockaddr_in6 *in6 = (struct net_sockaddr_in6 *)&ss;
	struct zsock_timeval tv = {.tv_sec = 1};
	int fd = zsock_socket(family, NET_SOCK_STREAM, NET_IPPROTO_TCP);

	zassert_true(fd >= 0);
	open_fd = fd;
	if (family == NET_AF_INET) {
		in->sin_family = NET_AF_INET;
		in->sin_port = net_htons(CONFIG_ESPHOME_API_PORT);
		zassert_equal(zsock_inet_pton(NET_AF_INET, ip, &in->sin_addr), 1);
	} else {
		in6->sin6_family = NET_AF_INET6;
		in6->sin6_port = net_htons(CONFIG_ESPHOME_API_PORT);
		zassert_equal(zsock_inet_pton(NET_AF_INET6, ip, &in6->sin6_addr), 1);
	}
	zassert_ok(zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &tv, sizeof(tv)));
	zassert_ok(zsock_connect(fd, (struct net_sockaddr *)&ss,
				 family == NET_AF_INET ? sizeof(*in) : sizeof(*in6)),
		   "connect %s: %d", ip, errno);

	return fd;
}

static int connect_server(void)
{
	return connect_to(NET_AF_INET, "127.0.0.1");
}

static void send_raw(int fd, const uint8_t *data, size_t len)
{
	zassert_equal(zsock_send(fd, data, len, 0), len);
}

static void send_frame(int fd, uint32_t type, const pb_msgdesc_t *fields, const void *msg)
{
	uint8_t frame[128];
	size_t len = 0;
	pb_ostream_t os;

	if (fields != NULL) {
		zassert_true(pb_get_encoded_size(&len, fields, msg));
	}
	size_t hdr = 2;

	/* Test messages are short enough for a single-byte length. */
	zassert_true(len < 128);
	frame[0] = 0x00;
	frame[1] = len;
	do {
		frame[hdr++] = (type & 0x7f) | (type > 0x7f ? 0x80 : 0);
		type >>= 7;
	} while (type != 0);
	if (fields != NULL) {
		os = pb_ostream_from_buffer(&frame[hdr], sizeof(frame) - hdr);
		zassert_true(pb_encode(&os, fields, msg));
	}
	send_raw(fd, frame, hdr + len);
}

static int recv_exact(int fd, uint8_t *dst, size_t len)
{
	while (len > 0) {
		ssize_t n = zsock_recv(fd, dst, len, 0);

		if (n <= 0) {
			return n == 0 ? -ECONNRESET : -errno;
		}
		dst += n;
		len -= n;
	}

	return 0;
}

static int recv_varint(int fd, uint32_t *val)
{
	uint8_t b;
	int ret;

	*val = 0;
	for (int shift = 0; shift < 35; shift += 7) {
		ret = recv_exact(fd, &b, 1);
		if (ret < 0) {
			return ret;
		}
		*val |= (uint32_t)(b & 0x7f) << shift;
		if ((b & 0x80) == 0) {
			return 0;
		}
	}

	return -EBADMSG;
}

/* Returns the body length into buf, or a negative errno (-ECONNRESET when closed). */
static int recv_frame(int fd, uint32_t *type)
{
	uint8_t preamble;
	uint32_t len;
	int ret;

	ret = recv_exact(fd, &preamble, 1);
	if (ret < 0) {
		return ret;
	}
	zassert_equal(preamble, 0x00);
	zassert_ok(recv_varint(fd, &len));
	zassert_ok(recv_varint(fd, type));
	zassert_true(len <= sizeof(buf));
	ret = recv_exact(fd, buf, len);

	return ret < 0 ? ret : (int)len;
}

static void expect_frame(int fd, uint32_t type, const pb_msgdesc_t *fields, void *msg)
{
	uint32_t got;
	int len = recv_frame(fd, &got);
	pb_istream_t is;

	zassert_true(len >= 0, "recv %d", len);
	zassert_equal(got, type, "expected %u, got %u", type, got);
	if (fields != NULL) {
		is = pb_istream_from_buffer(buf, len);
		zassert_true(pb_decode(&is, fields, msg));
	}
}

static void expect_closed(int fd)
{
	uint32_t type;

	zassert_equal(recv_frame(fd, &type), -ECONNRESET);
	zsock_close(fd);
	open_fd = -1;
}

static int connect_hello(void)
{
	int fd = connect_server();
	HelloRequest req = {.api_version_major = 1, .api_version_minor = 19};
	HelloResponse resp;

	strcpy(req.client_info, "ztest");
	send_frame(fd, HelloRequest_msgid, HelloRequest_fields, &req);
	expect_frame(fd, HelloResponse_msgid, HelloResponse_fields, &resp);
	zassert_equal(resp.api_version_major, ESPHOME_API_VERSION_MAJOR);
	zassert_equal(resp.api_version_minor, ESPHOME_API_VERSION_MINOR);
	zassert_str_equal(resp.name, DEVICE_NAME);

	return fd;
}

static void disconnect(int fd)
{
	send_frame(fd, DisconnectRequest_msgid, NULL, NULL);
	expect_frame(fd, DisconnectResponse_msgid, NULL, NULL);
	expect_closed(fd);
}

ZTEST(esphome_api_server, test_hello_ping_disconnect)
{
	int fd = connect_hello();

	send_frame(fd, PingRequest_msgid, NULL, NULL);
	expect_frame(fd, PingResponse_msgid, NULL, NULL);
	disconnect(fd);
}

#if defined(CONFIG_NET_IPV6)
/* The IPv4 tests above reach the same listener through v4-mapped addresses. */
ZTEST(esphome_api_server, test_ipv6_ping)
{
	int fd = connect_to(NET_AF_INET6, "::1");

	send_frame(fd, PingRequest_msgid, NULL, NULL);
	expect_frame(fd, PingResponse_msgid, NULL, NULL);
	disconnect(fd);
}
#endif

ZTEST(esphome_api_server, test_request_before_hello_closes)
{
	int fd = connect_server();

	send_frame(fd, DeviceInfoRequest_msgid, NULL, NULL);
	expect_closed(fd);
}

ZTEST(esphome_api_server, test_noise_preamble_closes)
{
	static const uint8_t noise_hello[] = {0x01, 0x00, 0x00};
	int fd = connect_server();

	send_raw(fd, noise_hello, sizeof(noise_hello));
	expect_closed(fd);
}

ZTEST(esphome_api_server, test_oversized_frame_closes)
{
	/* length 0x4000 > CONFIG_ESPHOME_API_RX_BUF_SIZE */
	static const uint8_t frame[] = {0x00, 0x80, 0x80, 0x01, 0x01};
	int fd = connect_hello();

	send_raw(fd, frame, sizeof(frame));
	expect_closed(fd);
}

ZTEST(esphome_api_server, test_device_info_and_capabilities)
{
	int fd = connect_hello();
	DeviceInfoResponse info;

	send_frame(fd, DeviceInfoRequest_msgid, NULL, NULL);
	expect_frame(fd, DeviceInfoResponse_msgid, DeviceInfoResponse_fields, &info);
	zassert_str_equal(info.name, DEVICE_NAME);
	zassert_str_equal(info.model, CONFIG_BOARD);
	zassert_equal(strlen(info.mac_address), 17);
	zassert_false(info.api_encryption_supported);

	send_frame(fd, DeviceCapabilitiesRequest_msgid, NULL, NULL);
	expect_frame(fd, DeviceCapabilitiesResponse_msgid, NULL, NULL);
	disconnect(fd);
}

ZTEST(esphome_api_server, test_unknown_message_ignored)
{
	int fd = connect_hello();

	/* SubscribeLogsRequest (28) with level 3: Home Assistant sends it, we have no logs. */
	static const uint8_t logs[] = {0x00, 0x02, 28, 0x08, 0x03};

	send_raw(fd, logs, sizeof(logs));
	send_frame(fd, PingRequest_msgid, NULL, NULL);
	expect_frame(fd, PingResponse_msgid, NULL, NULL);
	disconnect(fd);
}

ZTEST(esphome_api_server, test_list_entities)
{
	int fd = connect_hello();
	uint32_t seen = 0;
	uint32_t type;
	int len;

	send_frame(fd, ListEntitiesRequest_msgid, NULL, NULL);
	while ((len = recv_frame(fd, &type)) >= 0 && type != ListEntitiesDoneResponse_msgid) {
		pb_istream_t is = pb_istream_from_buffer(buf, len);

		switch (type) {
		case ListEntitiesSensorResponse_msgid: {
			ListEntitiesSensorResponse m;

			zassert_true(pb_decode(&is, ListEntitiesSensorResponse_fields, &m));
			zassert_equal(m.key, KEY_TEMPERATURE);
			zassert_str_equal(m.object_id, "temperature");
			zassert_str_equal(m.name, "Temperature");
			zassert_str_equal(m.unit_of_measurement, "°C");
			zassert_str_equal(m.device_class, "temperature");
			zassert_equal(m.state_class, SensorStateClass_STATE_CLASS_MEASUREMENT);
			zassert_equal(m.accuracy_decimals, 1);
			seen |= BIT(0);
			break;
		}
		case ListEntitiesBinarySensorResponse_msgid: {
			ListEntitiesBinarySensorResponse m;

			zassert_true(pb_decode(&is, ListEntitiesBinarySensorResponse_fields, &m));
			zassert_equal(m.key, KEY_STOPPED);
			seen |= BIT(1);
			break;
		}
		case ListEntitiesSwitchResponse_msgid: {
			ListEntitiesSwitchResponse m;

			zassert_true(pb_decode(&is, ListEntitiesSwitchResponse_fields, &m));
			zassert_equal(m.key, KEY_RELAY);
			/* No .name: falls back to the object_id. */
			zassert_str_equal(m.name, "relay");
			seen |= BIT(2);
			break;
		}
		case ListEntitiesButtonResponse_msgid: {
			ListEntitiesButtonResponse m;

			zassert_true(pb_decode(&is, ListEntitiesButtonResponse_fields, &m));
			zassert_equal(m.key, KEY_RESET);
			seen |= BIT(3);
			break;
		}
		default:
			zassert_unreachable("unexpected message %u", type);
		}
	}
	zassert_true(len >= 0);
	zassert_equal(seen, BIT_MASK(4));
	disconnect(fd);
}

ZTEST(esphome_api_server, test_states_and_commands)
{
	int fd = connect_hello();
	SensorStateResponse sensor;
	BinarySensorStateResponse binary;
	SwitchStateResponse sw;
	SwitchCommandRequest cmd = {.key = KEY_RELAY, .state = true};
	ButtonCommandRequest press = {.key = KEY_RESET};

	zassert_ok(esphome_api_publish_float(&temperature, 21.5f));
	zassert_equal(esphome_api_publish_float(&stopped, 1.0f), -EINVAL);
	zassert_equal(esphome_api_publish_bool(&reset, true), -EINVAL);

	/* Initial states in section order (relay, stopped, temperature); buttons have none. */
	send_frame(fd, SubscribeStatesRequest_msgid, NULL, NULL);
	expect_frame(fd, SwitchStateResponse_msgid, SwitchStateResponse_fields, &sw);
	zassert_equal(sw.key, KEY_RELAY);
	zassert_true(sw.missing_state);
	expect_frame(fd, BinarySensorStateResponse_msgid, BinarySensorStateResponse_fields,
		     &binary);
	zassert_equal(binary.key, KEY_STOPPED);
	zassert_true(binary.missing_state);
	expect_frame(fd, SensorStateResponse_msgid, SensorStateResponse_fields, &sensor);
	zassert_equal(sensor.key, KEY_TEMPERATURE);
	zassert_false(sensor.missing_state);
	zassert_equal(sensor.state, 21.5f);

	/* Updates after subscribing are pushed. */
	zassert_ok(esphome_api_publish_bool(&stopped, true));
	expect_frame(fd, BinarySensorStateResponse_msgid, BinarySensorStateResponse_fields,
		     &binary);
	zassert_true(binary.state);
	zassert_false(binary.missing_state);

	/* Switch command reaches the callback, which confirms the state. */
	send_frame(fd, SwitchCommandRequest_msgid, SwitchCommandRequest_fields, &cmd);
	expect_frame(fd, SwitchStateResponse_msgid, SwitchStateResponse_fields, &sw);
	zassert_equal(relay_calls, 1);
	zassert_true(relay_last);
	zassert_true(sw.state);

	send_frame(fd, ButtonCommandRequest_msgid, ButtonCommandRequest_fields, &press);
	/* A key that matches nothing is ignored. */
	press.key = KEY_RELAY;
	send_frame(fd, ButtonCommandRequest_msgid, ButtonCommandRequest_fields, &press);
	send_frame(fd, PingRequest_msgid, NULL, NULL);
	expect_frame(fd, PingResponse_msgid, NULL, NULL);
	zassert_equal(reset_calls, 1);
	disconnect(fd);
}

ZTEST(esphome_api_server, test_unsubscribed_client_gets_no_states)
{
	int fd = connect_hello();
	uint32_t type;

	zassert_ok(esphome_api_publish_float(&temperature, 1.0f));
	send_frame(fd, PingRequest_msgid, NULL, NULL);
	zassert_true(recv_frame(fd, &type) >= 0);
	zassert_equal(type, PingResponse_msgid);
	disconnect(fd);
}

ZTEST(esphome_api_server, test_idle_client_closed)
{
	int fd = connect_hello();
	struct zsock_timeval tv = {.tv_sec = CONFIG_ESPHOME_API_IDLE_TIMEOUT + 2};

	zassert_ok(zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &tv, sizeof(tv)));
	expect_closed(fd);
}

static void *setup(void)
{
	/* Let the server thread open its listener. */
	k_sleep(K_MSEC(100));
	return NULL;
}

/* A failed assertion leaves its socket open; close it so later tests get a slot. */
static void after(void *fixture)
{
	if (open_fd >= 0) {
		zsock_close(open_fd);
		open_fd = -1;
		k_sleep(K_MSEC(50));
	}
}

ZTEST_SUITE(esphome_api_server, NULL, setup, NULL, after, NULL);

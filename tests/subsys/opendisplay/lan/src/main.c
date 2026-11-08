/*
 * OpenDisplay LAN transport over loopback: framing, eviction, idle timeout,
 * port from WifiConfig and the DNS-SD record.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/ztest.h>

#include "od_internal.h"

#define DEFAULT_PORT CONFIG_OPENDISPLAY_LAN_PORT
#define IMG_SIZE     (DIV_ROUND_UP(296, 8) * 128)

static int dial(uint16_t port)
{
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(port),
	};
	struct zsock_timeval tv = {.tv_sec = 5};
	int fd;

	zsock_inet_pton(NET_AF_INET, "127.0.0.1", &addr.sin_addr);
	fd = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	zassert_true(fd >= 0);
	zassert_ok(zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &tv, sizeof(tv)));
	if (zsock_connect(fd, (struct net_sockaddr *)&addr, sizeof(addr)) < 0) {
		int err = -errno;

		zsock_close(fd);
		return err;
	}

	return fd;
}

static void send_all(int fd, const void *buf, size_t len)
{
	const uint8_t *p = buf;

	while (len > 0U) {
		ssize_t n = zsock_send(fd, p, len, 0);

		zassert_true(n > 0, "send: %d", errno);
		p += n;
		len -= (size_t)n;
	}
}

static void send_cmd(int fd, uint16_t cmd, const uint8_t *payload, size_t len)
{
	static uint8_t frame[4096];

	sys_put_le16((uint16_t)(len + 2U), frame);
	sys_put_be16(cmd, &frame[2]);
	memcpy(&frame[4], payload, len);
	send_all(fd, frame, len + 4U);
}

static int recv_all(int fd, uint8_t *buf, size_t len)
{
	while (len > 0U) {
		ssize_t n = zsock_recv(fd, buf, len, 0);

		if (n <= 0) {
			return n == 0 ? -ENOTCONN : -errno;
		}
		buf += n;
		len -= (size_t)n;
	}

	return 0;
}

/* Read one frame; returns its length or a negative errno. */
static int recv_frame(int fd, uint8_t *buf, size_t size)
{
	uint8_t hdr[2];
	uint16_t len;
	int err = recv_all(fd, hdr, sizeof(hdr));

	if (err < 0) {
		return err;
	}
	len = sys_get_le16(hdr);
	zassert_true(len >= 2U && len <= size, "frame length %u", len);
	err = recv_all(fd, buf, len);

	return err < 0 ? err : len;
}

static void expect(int fd, uint8_t status, uint8_t echo)
{
	uint8_t buf[128];
	int len = recv_frame(fd, buf, sizeof(buf));

	zassert_true(len >= 2, "recv: %d", len);
	zassert_equal(buf[0], status, "status 0x%02x", buf[0]);
	zassert_equal(buf[1], echo, "echo 0x%02x", buf[1]);
}

static bool closed_by_peer(int fd)
{
	uint8_t b;

	return zsock_recv(fd, &b, 1, 0) == 0;
}

static int dial_retry(uint16_t port)
{
	/* The server picks up a port change within a second. */
	for (int i = 0; i < 30; i++) {
		int fd = dial(port);

		if (fd >= 0) {
			return fd;
		}
		k_msleep(100);
	}

	return -ECONNREFUSED;
}

ZTEST(opendisplay_lan, test_fw_version)
{
	int fd = dial_retry(DEFAULT_PORT);

	zassert_true(fd >= 0);
	send_cmd(fd, OD_CMD_FW_VERSION, NULL, 0);
	expect(fd, OD_ACK, 0x43);
	zsock_close(fd);
}

ZTEST(opendisplay_lan, test_coalesced_and_split_frames)
{
	static const uint8_t two[] = {2, 0, 0x00, 0x43, 2, 0, 0x00, 0x44};
	int fd = dial_retry(DEFAULT_PORT);

	zassert_true(fd >= 0);
	send_all(fd, two, sizeof(two));
	expect(fd, OD_ACK, 0x43);
	expect(fd, OD_ACK, 0x44);

	for (size_t i = 0; i < 4; i++) {
		send_all(fd, &two[i], 1);
		k_msleep(20);
	}
	expect(fd, OD_ACK, 0x43);
	zsock_close(fd);
}

ZTEST(opendisplay_lan, test_bad_length_drops_client)
{
	static const uint8_t zero[] = {0, 0};
	static const uint8_t huge[] = {0xFF, 0x0F};
	int fd = dial_retry(DEFAULT_PORT);

	send_all(fd, zero, sizeof(zero));
	zassert_true(closed_by_peer(fd));
	zsock_close(fd);

	fd = dial_retry(DEFAULT_PORT);
	send_all(fd, huge, sizeof(huge));
	zassert_true(closed_by_peer(fd), "4095 is over the 4094 limit");
	zsock_close(fd);
}

ZTEST(opendisplay_lan, test_new_client_evicts_old)
{
	int a = dial_retry(DEFAULT_PORT);
	int b;

	send_cmd(a, OD_CMD_FW_VERSION, NULL, 0);
	expect(a, OD_ACK, 0x43);
	b = dial_retry(DEFAULT_PORT);
	zassert_true(closed_by_peer(a));
	send_cmd(b, OD_CMD_FW_VERSION, NULL, 0);
	expect(b, OD_ACK, 0x43);
	zsock_close(a);
	zsock_close(b);
}

ZTEST(opendisplay_lan, test_idle_timeout)
{
	int fd = dial_retry(DEFAULT_PORT);
	int64_t start = k_uptime_get();

	zassert_true(closed_by_peer(fd));
	zassert_true(k_uptime_get() - start >= CONFIG_OPENDISPLAY_LAN_IDLE_TIMEOUT * 1000 - 100);
	zsock_close(fd);
}

ZTEST(opendisplay_lan, test_direct_write_large_chunks)
{
	static uint8_t img[IMG_SIZE];
	int fd = dial_retry(DEFAULT_PORT);

	memset(img, 0xAA, sizeof(img));
	send_cmd(fd, OD_CMD_DW_START, NULL, 0);
	expect(fd, OD_ACK, 0x70);
	/* py-opendisplay sends max_frame - 2 = 4092 data bytes per chunk on LAN. */
	for (size_t off = 0; off < sizeof(img); off += 4092U) {
		send_cmd(fd, OD_CMD_DW_DATA, &img[off], MIN(4092U, sizeof(img) - off));
		expect(fd, OD_ACK, 0x71);
	}
	send_cmd(fd, OD_CMD_DW_END, (const uint8_t[]){0}, 1);
	expect(fd, OD_ACK, 0x72);
	expect(fd, OD_ACK, 0x73);
	zsock_close(fd);
}

static const struct dns_sd_rec *find_rec(void)
{
	STRUCT_SECTION_FOREACH(dns_sd_rec, rec) {
		if (strcmp(rec->service, "_opendisplay") == 0) {
			return rec;
		}
	}

	return NULL;
}

static bool txt_has(const struct dns_sd_rec *rec, const char *kv)
{
	for (size_t i = 0; i < rec->text_size; i += 1U + (uint8_t)rec->text[i]) {
		if ((uint8_t)rec->text[i] == strlen(kv) &&
		    memcmp(&rec->text[i + 1U], kv, strlen(kv)) == 0) {
			return true;
		}
	}

	return false;
}

ZTEST(opendisplay_lan, test_mdns_record)
{
	const struct dns_sd_rec *rec = find_rec();
	int fd = dial_retry(DEFAULT_PORT);

	zassert_true(fd >= 0, "server up");
	zsock_close(fd);
	zassert_not_null(rec);
	zassert_str_equal(rec->instance, "od-test");
	zassert_str_equal(rec->proto, "_tcp");
	zassert_equal(net_ntohs(*rec->port), DEFAULT_PORT);
	zassert_true(txt_has(rec, "tls=0"));
	zassert_true(txt_has(rec, "pv=2.2"));
	zassert_true(txt_has(rec, "cm=04"), "WiFi bit only");
	zassert_true(txt_has(rec, "fw=1.0"));
}

/* Config with a WifiConfig whose server_port is @p port. */
static size_t wifi_config(uint8_t *buf, uint16_t port)
{
	static const uint8_t zero[2];
	size_t len = 3 + 2 + OD_WIFI_LEN + 2;
	uint16_t crc;

	memset(buf, 0, len);
	buf[2] = 1;
	buf[4] = OD_PKT_WIFI;
	sys_put_be16(port, &buf[5 + OD_WIFI_PORT_OFFSET]);
	crc = crc16_itu_t(crc16_itu_t(0xFFFF, zero, 2), &buf[2], len - 4);
	sys_put_le16(crc, &buf[len - 2]);

	return len;
}

ZTEST(opendisplay_lan, test_port_from_config)
{
	uint8_t cfg[3 + 2 + OD_WIFI_LEN + 2];
	const struct dns_sd_rec *rec = find_rec();
	int fd = dial_retry(DEFAULT_PORT);

	zassert_equal(wifi_config(cfg, 2450), 167);
	send_cmd(fd, OD_CMD_CONFIG_WRITE, cfg, sizeof(cfg));
	expect(fd, OD_ACK, 0x41);
	zsock_close(fd);

	fd = dial_retry(2450);
	zassert_true(fd >= 0, "listening on the configured port");
	zassert_equal(net_ntohs(*rec->port), 2450);
	zassert_equal(dial(DEFAULT_PORT), -ECONNREFUSED);

	send_cmd(fd, OD_CMD_CONFIG_CLEAR, NULL, 0);
	expect(fd, OD_ACK, 0x45);
	zsock_close(fd);
	fd = dial_retry(DEFAULT_PORT);
	zassert_true(fd >= 0, "back on the default port");
	zsock_close(fd);
}

ZTEST_SUITE(opendisplay_lan, NULL, NULL, NULL, NULL, NULL);

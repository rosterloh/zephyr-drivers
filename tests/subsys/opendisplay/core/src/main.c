/*
 * OpenDisplay core: command frames in, response frames out, over a capture
 * transport.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "od_internal.h"

#define MAX_FRAMES 16

static struct {
	uint8_t buf[MAX_FRAMES][128];
	size_t len[MAX_FRAMES];
	size_t count;
} cap;

static int cap_send(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(tp);
	zassert_true(cap.count < MAX_FRAMES);
	zassert_true(len <= sizeof(cap.buf[0]));
	memcpy(cap.buf[cap.count], buf, len);
	cap.len[cap.count++] = len;
	return 0;
}

static const struct od_transport tp = {.send = cap_send};

static const uint8_t host_config[] = {
#include "config_vector.inc"
};

static void cmd(uint16_t op, const uint8_t *payload, size_t len)
{
	uint8_t req[256];

	sys_put_be16(op, req);
	memcpy(&req[2], payload, len);
	od_handle(&tp, req, len + 2U);
}

static void expect_ack(size_t idx, uint8_t status, uint8_t echo)
{
	zassert_true(cap.count > idx, "no frame %zu", idx);
	zassert_equal(cap.buf[idx][0], status, "status 0x%02x", cap.buf[idx][0]);
	zassert_equal(cap.buf[idx][1], echo, "echo 0x%02x", cap.buf[idx][1]);
}

/* Send READ_CONFIG and reassemble the container. */
static size_t read_config(uint8_t *out, size_t size)
{
	size_t total;
	size_t n;

	cmd(OD_CMD_CONFIG_READ, NULL, 0);
	zassert_true(cap.count >= 1U);
	expect_ack(0, OD_ACK, 0x40);
	zassert_equal(sys_get_le16(&cap.buf[0][2]), 0);
	total = sys_get_le16(&cap.buf[0][4]);
	zassert_true(total <= size);
	n = cap.len[0] - 6U;
	memcpy(out, &cap.buf[0][6], n);
	for (size_t i = 1; i < cap.count; i++) {
		zassert_true(cap.len[i] <= 100U);
		expect_ack(i, OD_ACK, 0x40);
		zassert_equal(sys_get_le16(&cap.buf[i][2]), i, "chunk number");
		memcpy(&out[n], &cap.buf[i][4], cap.len[i] - 4U);
		n += cap.len[i] - 4U;
	}
	zassert_equal(n, total, "assembled %zu of %zu", n, total);

	return total;
}

/* Find packet @p type; returns its body or NULL. */
static const uint8_t *find_pkt(const uint8_t *cfg, size_t len, uint8_t type, size_t body_len)
{
	static const struct {
		uint8_t type;
		uint16_t len;
	} sizes[] = {
		{OD_PKT_SYSTEM, OD_SYSTEM_LEN},
		{OD_PKT_MANUFACTURER, OD_MANUFACTURER_LEN},
		{OD_PKT_POWER, OD_POWER_LEN},
		{OD_PKT_DISPLAY, OD_DISPLAY_LEN},
		{OD_PKT_WIFI, OD_WIFI_LEN},
		{OD_PKT_SECURITY, OD_SECURITY_LEN},
		{OD_PKT_DATA_EXTENDED, OD_DATA_EXTENDED_LEN},
	};

	for (size_t off = 3; off + 2U < len - 2U;) {
		size_t plen = 0;

		for (size_t i = 0; i < ARRAY_SIZE(sizes); i++) {
			if (sizes[i].type == cfg[off + 1U]) {
				plen = sizes[i].len;
			}
		}
		zassert_not_equal(plen, 0, "unexpected packet 0x%02x", cfg[off + 1U]);
		if (cfg[off + 1U] == type) {
			zassert_equal(plen, body_len);
			return &cfg[off + 2U];
		}
		off += 2U + plen;
	}

	return NULL;
}

static uint16_t crc(const uint8_t *cfg, size_t len)
{
	static const uint8_t zero[2];

	return crc16_itu_t(crc16_itu_t(0xFFFF, zero, 2), &cfg[2], len - 4U);
}

/* WRITE_CONFIG as py-opendisplay chunks it: [total][200 bytes], then 200-byte 0x42s. */
static int write_config(const uint8_t *blob, size_t len)
{
	uint8_t first[202];
	size_t off = 200;

	if (len <= 200U) {
		cmd(OD_CMD_CONFIG_WRITE, blob, len);
		return cap.buf[cap.count - 1][0] == OD_ACK ? 0 : -1;
	}
	sys_put_le16((uint16_t)len, first);
	memcpy(&first[2], blob, 200);
	cmd(OD_CMD_CONFIG_WRITE, first, sizeof(first));
	expect_ack(cap.count - 1, OD_ACK, 0x41);
	while (off < len) {
		size_t n = MIN(len - off, 200U);

		cmd(OD_CMD_CONFIG_CHUNK, &blob[off], n);
		if (cap.buf[cap.count - 1][0] != OD_ACK) {
			return -1;
		}
		expect_ack(cap.count - 1, OD_ACK, 0x42);
		off += n;
	}

	return 0;
}

static void before(void *f)
{
	ARG_UNUSED(f);
	memset(&cap, 0, sizeof(cap));
}

ZTEST(opendisplay_core, test_fw_version)
{
	cmd(OD_CMD_FW_VERSION, NULL, 0);
	zassert_equal(cap.count, 1);
	expect_ack(0, OD_ACK, 0x43);
	zassert_equal(cap.buf[0][2], CONFIG_OPENDISPLAY_FW_VERSION_MAJOR);
	zassert_equal(cap.buf[0][3], CONFIG_OPENDISPLAY_FW_VERSION_MINOR);
	zassert_between_inclusive(cap.buf[0][4], 1, 40, "sha length");
	zassert_equal(cap.len[0], 6U + cap.buf[0][4], "patch byte follows the sha");
	zassert_equal(cap.buf[0][5 + cap.buf[0][4]], CONFIG_OPENDISPLAY_FW_VERSION_PATCH);
}

ZTEST(opendisplay_core, test_msd)
{
	static const uint8_t expect[] = {0x46, 0x24, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 90, 0x5e};
	struct opendisplay_status st = {
		.dynamic = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
		.temperature_c = 5,
		.battery_mv = 3500,
	};

	opendisplay_set_status(&st);
	cmd(OD_CMD_READ_MSD, NULL, 0);
	zassert_equal(cap.count, 1);
	expect_ack(0, OD_ACK, 0x44);
	zassert_equal(cap.len[0], 2U + OPENDISPLAY_MSD_LEN);
	zassert_mem_equal(&cap.buf[0][2], expect, sizeof(expect));
	/* 3500 mV = 350 * 10 mV: low byte 0x5e, bit 8 in byte 15 bit 0. */
	zassert_equal(cap.buf[0][17] & 0x01, 1);
	zassert_equal(cap.buf[0][17] >> 4, 1, "loop counter bumped once");
}

ZTEST(opendisplay_core, test_unknown_command)
{
	cmd(0x0077, NULL, 0);
	zassert_equal(cap.count, 1);
	expect_ack(0, OD_NACK, 0x77);

	/* Too short to carry an opcode: dropped. */
	od_handle(&tp, (const uint8_t[]){0x00}, 1);
	zassert_equal(cap.count, 1);
}

ZTEST(opendisplay_core, test_read_config)
{
	static uint8_t cfg[CONFIG_OPENDISPLAY_CONFIG_BUF_SIZE];
	const uint8_t *p;
	size_t len = read_config(cfg, sizeof(cfg));

	zassert_equal(sys_get_le16(cfg), len);
	zassert_equal(cfg[2], 1, "version");
	zassert_equal(sys_get_le16(&cfg[len - 2U]), crc(cfg, len));

	zassert_not_null(find_pkt(cfg, len, OD_PKT_SYSTEM, OD_SYSTEM_LEN));
	p = find_pkt(cfg, len, OD_PKT_MANUFACTURER, OD_MANUFACTURER_LEN);
	zassert_not_null(p);
	zassert_equal(sys_get_le16(p), CONFIG_OPENDISPLAY_MANUFACTURER_ID);
	p = find_pkt(cfg, len, OD_PKT_POWER, OD_POWER_LEN);
	zassert_not_null(p);
	zassert_equal(p[0], CONFIG_OPENDISPLAY_POWER_MODE);
	p = find_pkt(cfg, len, OD_PKT_DISPLAY, OD_DISPLAY_LEN);
	zassert_not_null(p);
	zassert_equal(sys_get_le16(&p[4]), 296, "width from devicetree");
	zassert_equal(sys_get_le16(&p[6]), 128, "height from devicetree");
	zassert_equal(p[21], OD_COLOR_MONO);
}

ZTEST(opendisplay_core, test_write_config)
{
	static uint8_t cfg[CONFIG_OPENDISPLAY_CONFIG_BUF_SIZE];
	const uint8_t *wifi = &host_config[3 + 24 + 24 + 32 + 48 + 2];
	const uint8_t *ext = wifi + OD_WIFI_LEN + 2;
	const uint8_t *p;
	size_t len;

	zassert_equal(write_config(host_config, sizeof(host_config)), 0);
	zassert_mem_equal(od_config_packet(OD_PKT_WIFI), wifi, OD_WIFI_LEN);
	zassert_is_null(od_config_packet(OD_PKT_SECURITY));

	memset(&cap, 0, sizeof(cap));
	len = read_config(cfg, sizeof(cfg));
	p = find_pkt(cfg, len, OD_PKT_WIFI, OD_WIFI_LEN);
	zassert_not_null(p, "host packet kept");
	zassert_mem_equal(p, wifi, OD_WIFI_LEN);
	p = find_pkt(cfg, len, OD_PKT_DATA_EXTENDED, OD_DATA_EXTENDED_LEN);
	zassert_not_null(p);
	zassert_mem_equal(p, ext, OD_DATA_EXTENDED_LEN);
	/* Host wrote a zero-size display; the device still reports its own. */
	p = find_pkt(cfg, len, OD_PKT_DISPLAY, OD_DISPLAY_LEN);
	zassert_equal(sys_get_le16(&p[4]), 296);

	memset(&cap, 0, sizeof(cap));
	cmd(OD_CMD_CONFIG_CLEAR, NULL, 0);
	expect_ack(0, OD_ACK, 0x45);
	zassert_is_null(od_config_packet(OD_PKT_WIFI));
}

ZTEST(opendisplay_core, test_write_config_rejects)
{
	static uint8_t bad[sizeof(host_config)];
	uint8_t big[202] = {0};

	/* Corrupt one byte: CRC mismatch on the last chunk. */
	memcpy(bad, host_config, sizeof(bad));
	bad[150] ^= 0x01;
	zassert_not_equal(write_config(bad, sizeof(bad)), 0);
	expect_ack(cap.count - 1, OD_NACK, 0x42);
	zassert_is_null(od_config_packet(OD_PKT_WIFI));

	/* Chunk without a start. */
	memset(&cap, 0, sizeof(cap));
	cmd(OD_CMD_CONFIG_CHUNK, host_config, 10);
	expect_ack(0, OD_NACK, 0x42);

	/* Declared total larger than the buffer. */
	memset(&cap, 0, sizeof(cap));
	sys_put_le16(CONFIG_OPENDISPLAY_CONFIG_BUF_SIZE + 1, big);
	cmd(OD_CMD_CONFIG_WRITE, big, sizeof(big));
	expect_ack(0, OD_NACK, 0x41);

	/* Truncated packet in a single-frame write. */
	memset(&cap, 0, sizeof(cap));
	uint8_t trunc[3 + 2 + 10 + 2] = {0, 0, 1, 0, OD_PKT_SYSTEM};

	sys_put_le16(crc(trunc, sizeof(trunc)), &trunc[sizeof(trunc) - 2]);
	cmd(OD_CMD_CONFIG_WRITE, trunc, sizeof(trunc));
	expect_ack(0, OD_NACK, 0x41);
}

ZTEST_SUITE(opendisplay_core, NULL, NULL, before, NULL, NULL);

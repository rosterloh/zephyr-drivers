/*
 * OpenDisplay encryption: AUTHENTICATE, the AES-CCM envelope and its gate,
 * against a host side checked with py-opendisplay vectors.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <psa/crypto.h>
#include <zephyr/opendisplay/opendisplay.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#include "od_internal.h"
#include "vectors.inc"

#define CCM_ALG PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 12)

static const uint8_t master[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const uint8_t device_id[4] = {0, 0, 0, 1};

static struct {
	uint8_t buf[300];
	size_t len;
	int count;
} rx;

static int cap_send(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(tp);
	zassert_true(len <= sizeof(rx.buf));
	memcpy(rx.buf, buf, len);
	rx.len = len;
	rx.count++;
	return 0;
}

static const struct od_transport ble = {.send = cap_send};
static const struct od_transport ble2 = {.send = cap_send};
static const struct od_transport tls = {.send = cap_send, .secure = true};

/* Host side, as py-opendisplay does it. */
static struct {
	uint8_t key[16];
	uint8_t id[8];
	uint64_t ctr;
} host;

static psa_key_id_t import(const uint8_t key[16], psa_algorithm_t alg, psa_key_usage_t usage)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t id;

	zassert_equal(psa_crypto_init(), PSA_SUCCESS);
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 128);
	psa_set_key_algorithm(&attr, alg);
	psa_set_key_usage_flags(&attr, usage);
	zassert_equal(psa_import_key(&attr, key, 16, &id), PSA_SUCCESS);

	return id;
}

static void cmac(const uint8_t key[16], const uint8_t *in, size_t len, uint8_t out[16])
{
	psa_key_id_t id = import(key, PSA_ALG_CMAC, PSA_KEY_USAGE_SIGN_MESSAGE);
	size_t olen;

	zassert_equal(psa_mac_compute(id, PSA_ALG_CMAC, in, len, out, 16, &olen), PSA_SUCCESS);
	psa_destroy_key(id);
}

static void host_derive(const uint8_t *mk, const uint8_t sn[16], const uint8_t cn[16])
{
	uint8_t in[64];
	uint8_t blk[16];
	psa_key_id_t id;
	size_t olen;

	memcpy(in, "OpenDisplay session", 20);
	memcpy(&in[20], device_id, 4);
	memcpy(&in[24], cn, 16);
	memcpy(&in[40], sn, 16);
	in[56] = 0x00;
	in[57] = 0x80;
	cmac(mk, in, 58, in);
	sys_put_be64(1, blk);
	memcpy(&blk[8], in, 8);
	id = import(mk, PSA_ALG_ECB_NO_PADDING, PSA_KEY_USAGE_ENCRYPT);
	zassert_equal(psa_cipher_encrypt(id, PSA_ALG_ECB_NO_PADDING, blk, 16, host.key, 16, &olen),
		      PSA_SUCCESS);
	psa_destroy_key(id);
	memcpy(in, cn, 16);
	memcpy(&in[16], sn, 16);
	cmac(host.key, in, 32, blk);
	memcpy(host.id, blk, 8);
	host.ctr = 0;
}

static void challenge_mac(const uint8_t *mk, const uint8_t sn[16], const uint8_t cn[16],
			  uint8_t out[16])
{
	uint8_t in[36];

	memcpy(in, sn, 16);
	memcpy(&in[16], cn, 16);
	memcpy(&in[32], device_id, 4);
	cmac(mk, in, sizeof(in), out);
}

/* [cmd:2][id:8][ctr:8][ct][tag:12] */
static size_t host_seal(uint16_t cmd, uint64_t ctr, const uint8_t *data, size_t len, uint8_t *out)
{
	psa_key_id_t id = import(host.key, CCM_ALG, PSA_KEY_USAGE_ENCRYPT);
	uint8_t pt[256];
	size_t olen;

	pt[0] = (uint8_t)len;
	memcpy(&pt[1], data, len);
	sys_put_be16(cmd, out);
	memcpy(&out[2], host.id, 8);
	sys_put_be64(ctr, &out[10]);
	zassert_equal(psa_aead_encrypt(id, CCM_ALG, &out[5], 13, out, 2, pt, len + 1, &out[18], 256,
				       &olen),
		      PSA_SUCCESS);
	psa_destroy_key(id);

	return 18 + olen;
}

/* Decrypt the last reply into [status][echo][payload]; returns its length. */
static size_t host_open(uint8_t *out)
{
	psa_key_id_t id = import(host.key, CCM_ALG, PSA_KEY_USAGE_DECRYPT);
	uint8_t pt[256];
	size_t olen;

	zassert_true(rx.len >= 31, "reply not enveloped (%zu bytes)", rx.len);
	zassert_mem_equal(&rx.buf[2], host.id, 8, "reply uses the session id");
	zassert_true(rx.buf[10] & 0x80, "device counter half");
	zassert_equal(psa_aead_decrypt(id, CCM_ALG, &rx.buf[5], 13, rx.buf, 2, &rx.buf[18],
				       rx.len - 18, pt, sizeof(pt), &olen),
		      PSA_SUCCESS);
	psa_destroy_key(id);
	out[0] = rx.buf[0];
	out[1] = rx.buf[1];
	memcpy(&out[2], &pt[1], pt[0]);

	return 2 + pt[0];
}

static void cmd(const struct od_transport *tp, uint16_t op, const uint8_t *data, size_t len)
{
	uint8_t req[2 + 256];

	sys_put_be16(op, req);
	memcpy(&req[2], data, len);
	rx.count = 0;
	od_handle(tp, req, len + 2);
}

static void expect3(uint8_t echo, uint8_t marker)
{
	uint8_t want[3] = {0x00, echo, marker};

	zassert_equal(rx.len, 3, "len %zu", rx.len);
	zassert_mem_equal(rx.buf, want, 3, "%02x %02x %02x", rx.buf[0], rx.buf[1], rx.buf[2]);
}

/* Config holding a SecurityConfig with @p key and @p timeout seconds. */
static void write_security(const uint8_t *key, uint16_t timeout)
{
	static const uint8_t zero[2];
	uint8_t buf[3 + 2 + OD_SECURITY_LEN + 2];
	uint16_t crc;

	memset(buf, 0, sizeof(buf));
	buf[2] = 1;
	buf[4] = OD_PKT_SECURITY;
	buf[5] = key != NULL;
	if (key != NULL) {
		memcpy(&buf[6], key, 16);
	}
	sys_put_le16(timeout, &buf[5 + 17]);
	crc = crc16_itu_t(crc16_itu_t(0xFFFF, zero, 2), &buf[2], sizeof(buf) - 4);
	sys_put_le16(crc, &buf[sizeof(buf) - 2]);
	cmd(&tls, OD_CMD_CONFIG_WRITE, buf, sizeof(buf));
	zassert_equal(sys_get_be16(rx.buf), 0x0041);
}

static uint8_t sn[16];

static void step1(const struct od_transport *tp)
{
	cmd(tp, OD_CMD_AUTHENTICATE, (const uint8_t[]){0}, 1);
	zassert_equal(rx.len, 23);
	zassert_equal(sys_get_be16(rx.buf), 0x0050);
	zassert_equal(rx.buf[2], 0x00);
	zassert_mem_equal(&rx.buf[19], device_id, 4);
	memcpy(sn, &rx.buf[3], 16);
}

/* Full handshake; returns the step-2 status. */
static uint8_t authenticate(const struct od_transport *tp, const uint8_t *mk)
{
	uint8_t req[32];
	uint8_t proof[16];

	step1(tp);
	memset(req, 0x5A, 16);
	challenge_mac(mk, sn, req, &req[16]);
	cmd(tp, OD_CMD_AUTHENTICATE, req, sizeof(req));
	zassert_equal(sys_get_be16(rx.buf), 0x0050);
	if (rx.buf[2] != 0) {
		zassert_equal(rx.len, 3);
		return rx.buf[2];
	}
	zassert_equal(rx.len, 19);
	host_derive(mk, sn, req);
	/* CMAC(session key, server nonce || client nonce || device id) */
	challenge_mac(host.key, sn, req, proof);
	zassert_mem_equal(&rx.buf[3], proof, 16, "server proof");

	return 0;
}

static void sealed_cmd(const struct od_transport *tp, uint16_t op, const uint8_t *data, size_t len)
{
	uint8_t req[300];
	size_t n = host_seal(op, host.ctr++, data, len, req);

	rx.count = 0;
	od_handle(tp, req, n);
}

static void before(void *f)
{
	ARG_UNUSED(f);
	cmd(&tls, OD_CMD_CONFIG_CLEAR, NULL, 0);
	od_disconnected(&ble);
	od_disconnected(&ble2);
	memset(&rx, 0, sizeof(rx));
}

ZTEST(opendisplay_security, test_vectors)
{
	uint8_t mac[16];
	uint8_t psk[16];
	uint8_t frame[64];
	static const uint8_t data[] = {0xaa, 0xbb, 0xcc};
	const uint8_t *sn_kat = (const uint8_t[]){0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
						  0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
	const uint8_t *cn_kat = (const uint8_t[]){0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
						  0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f};

	challenge_mac(master, sn_kat, cn_kat, mac);
	zassert_mem_equal(mac, kat_mac, 16);
	host_derive(master, sn_kat, cn_kat);
	zassert_mem_equal(host.key, kat_session_key, 16);
	zassert_mem_equal(host.id, kat_session_id, 8);
	zassert_equal(host_seal(0x0041, 5, data, sizeof(data), frame), sizeof(kat_frame));
	zassert_mem_equal(frame, kat_frame, sizeof(kat_frame));

	zassert_equal(od_sec_tls_psk(psk), -ENOENT, "disabled");
	write_security(master, 0);
	zassert_equal(od_sec_tls_psk(psk), 0);
	zassert_mem_equal(psk, kat_psk, 16);
}

ZTEST(opendisplay_security, test_not_configured)
{
	cmd(&ble, OD_CMD_AUTHENTICATE, (const uint8_t[]){0}, 1);
	expect3(0x50, 0x03);
	cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	zassert_equal(rx.len, 18, "plain MSD");

	/* An all-zero key leaves encryption off. */
	write_security((const uint8_t[16]){0}, 0);
	cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	zassert_equal(rx.len, 18);
}

ZTEST(opendisplay_security, test_gate)
{
	write_security(master, 0);

	cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	expect3(0x44, OD_AUTH_REQUIRED);
	cmd(&ble, OD_CMD_CONFIG_CLEAR, NULL, 0);
	expect3(0x45, OD_AUTH_REQUIRED);
	cmd(&ble, OD_CMD_FW_VERSION, NULL, 0);
	zassert_equal(sys_get_be16(rx.buf), 0x0043, "handshake stays plain");

	/* TLS-origin frames skip the gate and the envelope. */
	cmd(&tls, OD_CMD_READ_MSD, NULL, 0);
	zassert_equal(rx.len, 18);
}

ZTEST(opendisplay_security, test_session)
{
	uint8_t out[300];
	uint8_t req[300];
	uint8_t msd[OPENDISPLAY_MSD_LEN];
	size_t n;

	write_security(master, 0);
	zassert_equal(authenticate(&ble, master), 0);

	sealed_cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	n = host_open(out);
	opendisplay_get_msd(msd);
	zassert_equal(n, 2 + sizeof(msd));
	zassert_equal(sys_get_be16(out), 0x0044);
	zassert_mem_equal(&out[2], msd, sizeof(msd));

	/* FW_VERSION replies stay plain, enveloped request or not. */
	sealed_cmd(&ble, OD_CMD_FW_VERSION, NULL, 0);
	zassert_equal(sys_get_be16(rx.buf), 0x0043);
	zassert_true(rx.len < 31);

	/* Plain frame during a session. */
	cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	expect3(0x44, OD_AUTH_REQUIRED);

	/* Replay, and a counter far behind, are refused. */
	n = host_seal(OD_CMD_READ_MSD, 100, NULL, 0, req);
	rx.count = 0;
	od_handle(&ble, req, n);
	zassert_true(rx.len >= 31);
	od_handle(&ble, req, n);
	expect3(0x44, OD_NACK);
	n = host_seal(OD_CMD_READ_MSD, 30, NULL, 0, req);
	od_handle(&ble, req, n);
	expect3(0x44, OD_NACK);
	/* Out of order within the window is fine. */
	n = host_seal(OD_CMD_READ_MSD, 90, NULL, 0, req);
	od_handle(&ble, req, n);
	zassert_true(rx.len >= 31);

	/* Tampered tag, and the session survives it. */
	n = host_seal(OD_CMD_READ_MSD, 101, NULL, 0, req);
	req[n - 1] ^= 1;
	od_handle(&ble, req, n);
	expect3(0x44, OD_NACK);
	host.ctr = 102;
	sealed_cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	zassert_true(rx.len >= 31);

	/* Another link has no session. */
	sealed_cmd(&ble2, OD_CMD_READ_MSD, NULL, 0);
	expect3(0x44, OD_AUTH_REQUIRED);

	/* A disconnect ends it. */
	od_disconnected(&ble);
	sealed_cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	expect3(0x44, OD_AUTH_REQUIRED);
}

ZTEST(opendisplay_security, test_clear_ends_encryption)
{
	write_security(master, 0);
	zassert_equal(authenticate(&ble, master), 0);
	sealed_cmd(&ble, OD_CMD_CONFIG_CLEAR, NULL, 0);
	zassert_equal(rx.len, 2, "ack goes out after the key is gone");
	zassert_equal(sys_get_be16(rx.buf), 0x0045);
	cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	zassert_equal(rx.len, 18);
}

ZTEST(opendisplay_security, test_wrong_key_and_stale_challenge)
{
	uint8_t other[16];
	uint8_t req[32] = {0};

	memset(other, 0x77, sizeof(other));
	write_security(master, 0);
	zassert_equal(authenticate(&ble, other), 0x01);

	/* Step 2 without a fresh step 1. */
	cmd(&ble, OD_CMD_AUTHENTICATE, req, sizeof(req));
	expect3(0x50, 0x01);

	/* Step 2 from another link. */
	step1(&ble);
	cmd(&ble2, OD_CMD_AUTHENTICATE, req, sizeof(req));
	expect3(0x50, 0x01);
	zassert_equal(authenticate(&ble, master), 0);
}

ZTEST(opendisplay_security, test_session_timeout)
{
	write_security(master, 1);
	zassert_equal(authenticate(&ble, master), 0);
	sealed_cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	zassert_true(rx.len >= 31);
	k_msleep(1100);
	sealed_cmd(&ble, OD_CMD_READ_MSD, NULL, 0);
	expect3(0x44, OD_AUTH_REQUIRED);
}

/* Runs last: the limit lasts a minute. */
ZTEST(opendisplay_security, test_zz_rate_limit)
{
	write_security(master, 0);
	for (int i = 0; i < 10; i++) {
		cmd(&ble, OD_CMD_AUTHENTICATE, (const uint8_t[]){0}, 1);
		zassert_equal(rx.buf[2], 0x00, "attempt %d", i);
	}
	cmd(&ble, OD_CMD_AUTHENTICATE, (const uint8_t[]){0}, 1);
	expect3(0x50, 0x04);
}

ZTEST_SUITE(opendisplay_security, NULL, NULL, before, NULL, NULL);

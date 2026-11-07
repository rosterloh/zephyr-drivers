/*
 * OpenDisplay encryption: challenge-response AUTHENTICATE (0x50) with the
 * SecurityConfig master key, then AES-128-CCM envelopes on links that are not
 * already secure (BLE):
 *
 *   [cmd:2][session_id:8][counter:8 BE][ciphertext of [len:1][payload]][tag:12]
 *
 * with the 2 leading bytes as AAD and nonce bytes 3..15 as the CCM nonce.
 * Replies use the same shape with the reply's [status][echo] in front.
 * All entry points run with od_lock held.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <psa/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "od_internal.h"

LOG_MODULE_DECLARE(opendisplay, CONFIG_OPENDISPLAY_LOG_LEVEL);

#define KEY_LEN      16U
#define NONCE_LEN    16U
#define TAG_LEN      12U
#define ENV_OVERHEAD (2U + NONCE_LEN + 1U + TAG_LEN)
#define CCM_ALG      PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, TAG_LEN)

/* SecurityConfig body */
#define SEC_ENABLED 0U
#define SEC_KEY     1U
#define SEC_TIMEOUT 17U

/* AUTHENTICATE status byte */
#define AUTH_OK         0x00U
#define AUTH_FAILED     0x01U
#define AUTH_NOT_CONFIG 0x03U
#define AUTH_RATE_LIMIT 0x04U

#define AUTH_MAX_ATTEMPTS 10U
#define AUTH_WINDOW_MS    (60 * MSEC_PER_SEC)
#define CHALLENGE_MS      (30 * MSEC_PER_SEC)

/* Hosts use a fixed placeholder device id. */
static const uint8_t device_id[4] = {0x00, 0x00, 0x00, 0x01};

static struct {
	const struct od_transport *tp;
	bool active;
	bool challenge;
	uint8_t master[KEY_LEN]; /* key the session was opened with */
	uint8_t key[KEY_LEN];
	uint8_t id[8];
	uint8_t server_nonce[NONCE_LEN];
	int64_t challenge_at;
	int64_t opened_at;
	/* Own counters start at 2^63 so they never repeat the host's nonces. */
	uint64_t tx;
	uint64_t rx_top;
	uint64_t rx_seen; /* bit i: rx_top - i was accepted */
	bool rx_any;
	int64_t attempts_since;
	uint8_t attempts;
} ses;

static uint8_t sealed[ENV_OVERHEAD + OD_SEC_MAX_PAYLOAD];
/* [len:1][payload], so at most 256 bytes. */
static uint8_t opened[256];

/* Master key from the host-written SecurityConfig, or NULL when disabled. */
static const uint8_t *master_key(void)
{
	const uint8_t *sec = od_config_packet(OD_PKT_SECURITY);
	static const uint8_t zero[KEY_LEN];

	if (sec == NULL || sec[SEC_ENABLED] != 1U ||
	    memcmp(&sec[SEC_KEY], zero, sizeof(zero)) == 0) {
		return NULL;
	}

	return &sec[SEC_KEY];
}

bool od_sec_enabled(void)
{
	return master_key() != NULL;
}

static psa_status_t import(const uint8_t key[KEY_LEN], psa_algorithm_t alg, psa_key_usage_t usage,
			   psa_key_id_t *id)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t st = psa_crypto_init();

	if (st != PSA_SUCCESS) {
		return st;
	}
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 128U);
	psa_set_key_algorithm(&attr, alg);
	psa_set_key_usage_flags(&attr, usage);

	return psa_import_key(&attr, key, KEY_LEN, id);
}

static int cmac(const uint8_t key[KEY_LEN], const uint8_t *in, size_t len, uint8_t out[16])
{
	psa_key_id_t id;
	size_t olen;
	psa_status_t st = import(key, PSA_ALG_CMAC, PSA_KEY_USAGE_SIGN_MESSAGE, &id);

	if (st == PSA_SUCCESS) {
		st = psa_mac_compute(id, PSA_ALG_CMAC, in, len, out, 16U, &olen);
		(void)psa_destroy_key(id);
	}

	return st == PSA_SUCCESS ? 0 : -EIO;
}

static int aes_block(const uint8_t key[KEY_LEN], const uint8_t in[16], uint8_t out[16])
{
	psa_key_id_t id;
	size_t olen;
	psa_status_t st = import(key, PSA_ALG_ECB_NO_PADDING, PSA_KEY_USAGE_ENCRYPT, &id);

	if (st == PSA_SUCCESS) {
		st = psa_cipher_encrypt(id, PSA_ALG_ECB_NO_PADDING, in, 16U, out, 16U, &olen);
		(void)psa_destroy_key(id);
	}

	return st == PSA_SUCCESS ? 0 : -EIO;
}

static int ccm(bool encrypt, const uint8_t nonce[NONCE_LEN], const uint8_t ad[2], const uint8_t *in,
	       size_t len, uint8_t *out, size_t size, size_t *olen)
{
	psa_key_id_t id;
	psa_status_t st = import(ses.key, CCM_ALG,
				 encrypt ? PSA_KEY_USAGE_ENCRYPT : PSA_KEY_USAGE_DECRYPT, &id);

	if (st != PSA_SUCCESS) {
		return -EIO;
	}
	if (encrypt) {
		st = psa_aead_encrypt(id, CCM_ALG, &nonce[3], NONCE_LEN - 3U, ad, 2U, in, len, out,
				      size, olen);
	} else {
		st = psa_aead_decrypt(id, CCM_ALG, &nonce[3], NONCE_LEN - 3U, ad, 2U, in, len, out,
				      size, olen);
	}
	(void)psa_destroy_key(id);

	return st == PSA_SUCCESS ? 0 : -EBADMSG;
}

int od_sec_tls_psk(uint8_t psk[16])
{
	static const char label[] = "opendisplay-tls-psk";
	const uint8_t *master = master_key();

	if (master == NULL) {
		return -ENOENT;
	}

	return cmac(master, (const uint8_t *)label, sizeof(label) - 1U, psk);
}

void od_sec_reset(const struct od_transport *tp)
{
	int64_t since = ses.attempts_since;
	uint8_t attempts = ses.attempts;

	/* Keeps the rate limit across sessions. */
	if (tp == NULL || ses.tp == tp) {
		memset(&ses, 0, sizeof(ses));
		ses.attempts_since = since;
		ses.attempts = attempts;
	}
}

/* Session for @p tp that is still valid for the current key and timeout. */
static bool session(const struct od_transport *tp)
{
	const uint8_t *master = master_key();
	const uint8_t *sec = od_config_packet(OD_PKT_SECURITY);
	uint16_t timeout;

	if (!ses.active || ses.tp != tp) {
		return false;
	}
	if (master == NULL || memcmp(master, ses.master, KEY_LEN) != 0) {
		od_sec_reset(NULL);
		return false;
	}
	timeout = sys_get_le16(&sec[SEC_TIMEOUT]);
	if (timeout != 0U && k_uptime_get() - ses.opened_at >= (int64_t)timeout * MSEC_PER_SEC) {
		LOG_INF("session expired");
		od_sec_reset(NULL);
		return false;
	}

	return true;
}

static void auth_reply(const struct od_transport *tp, uint8_t status, const uint8_t *data,
		       size_t len)
{
	uint8_t buf[3 + NONCE_LEN + sizeof(device_id)];

	buf[0] = OD_ACK;
	buf[1] = (uint8_t)OD_CMD_AUTHENTICATE;
	buf[2] = status;
	memcpy(&buf[3], data, len);
	(void)tp->send(tp, buf, 3U + len);
}

static int open_session(const uint8_t *master, const uint8_t client_nonce[NONCE_LEN],
			uint8_t proof[16])
{
	static const char label[] = "OpenDisplay session";
	uint8_t in[sizeof(label) + sizeof(device_id) + 2U * NONCE_LEN + 2U];
	uint8_t blk[16];
	uint8_t *p = in;
	int err;

	memcpy(p, label, sizeof(label)); /* label and its NUL */
	p += sizeof(label);
	memcpy(p, device_id, sizeof(device_id));
	p += sizeof(device_id);
	memcpy(p, client_nonce, NONCE_LEN);
	p += NONCE_LEN;
	memcpy(p, ses.server_nonce, NONCE_LEN);
	p += NONCE_LEN;
	*p++ = 0x00U;
	*p++ = 0x80U;

	err = cmac(master, in, sizeof(in), in);
	if (err == 0) {
		sys_put_be64(1U, blk);
		memcpy(&blk[8], in, 8U);
		err = aes_block(master, blk, ses.key);
	}
	if (err == 0) {
		memcpy(in, client_nonce, NONCE_LEN);
		memcpy(&in[NONCE_LEN], ses.server_nonce, NONCE_LEN);
		err = cmac(ses.key, in, 2U * NONCE_LEN, blk);
		memcpy(ses.id, blk, sizeof(ses.id));
	}
	if (err == 0) {
		memcpy(in, ses.server_nonce, NONCE_LEN);
		memcpy(&in[NONCE_LEN], client_nonce, NONCE_LEN);
		memcpy(&in[2U * NONCE_LEN], device_id, sizeof(device_id));
		err = cmac(ses.key, in, 2U * NONCE_LEN + sizeof(device_id), proof);
	}

	return err;
}

void od_sec_authenticate(const struct od_transport *tp, const uint8_t *data, size_t len)
{
	const uint8_t *master = master_key();
	int64_t now = k_uptime_get();
	uint8_t in[2U * NONCE_LEN + sizeof(device_id)];
	uint8_t mac[16];
	uint8_t diff = 0U;

	if (master == NULL) {
		auth_reply(tp, AUTH_NOT_CONFIG, NULL, 0);
		return;
	}
	if (ses.attempts == 0U || now - ses.attempts_since >= AUTH_WINDOW_MS) {
		ses.attempts = 0U;
		ses.attempts_since = now;
	}
	if (ses.attempts >= AUTH_MAX_ATTEMPTS) {
		auth_reply(tp, AUTH_RATE_LIMIT, NULL, 0);
		return;
	}
	ses.attempts++;

	if (len == 1U && data[0] == 0x00U) {
		/* A new challenge ends any session. */
		od_sec_reset(NULL);
		if (psa_crypto_init() != PSA_SUCCESS ||
		    psa_generate_random(ses.server_nonce, NONCE_LEN) != PSA_SUCCESS) {
			auth_reply(tp, AUTH_FAILED, NULL, 0);
			return;
		}
		ses.tp = tp;
		ses.challenge = true;
		ses.challenge_at = now;
		memcpy(in, ses.server_nonce, NONCE_LEN);
		memcpy(&in[NONCE_LEN], device_id, sizeof(device_id));
		auth_reply(tp, AUTH_OK, in, NONCE_LEN + sizeof(device_id));
		return;
	}

	if (len != 2U * NONCE_LEN || !ses.challenge || ses.tp != tp ||
	    now - ses.challenge_at > CHALLENGE_MS) {
		od_sec_reset(NULL);
		auth_reply(tp, AUTH_FAILED, NULL, 0);
		return;
	}

	/* Expect CMAC(master, server_nonce || client_nonce || device_id). */
	memcpy(in, ses.server_nonce, NONCE_LEN);
	memcpy(&in[NONCE_LEN], data, NONCE_LEN);
	memcpy(&in[2U * NONCE_LEN], device_id, sizeof(device_id));
	if (cmac(master, in, sizeof(in), mac) != 0) {
		diff = 1U;
	}
	for (size_t i = 0; i < sizeof(mac); i++) {
		diff |= mac[i] ^ data[NONCE_LEN + i];
	}
	ses.challenge = false;
	if (diff != 0U || open_session(master, data, mac) != 0) {
		LOG_WRN("authentication failed");
		od_sec_reset(NULL);
		auth_reply(tp, AUTH_FAILED, NULL, 0);
		return;
	}

	memcpy(ses.master, master, KEY_LEN);
	ses.active = true;
	ses.opened_at = now;
	ses.tx = BIT64(63);
	ses.attempts = 0U;
	LOG_INF("session opened");
	auth_reply(tp, AUTH_OK, mac, sizeof(mac));
}

/* Accept each counter once, up to 63 behind the newest. */
static bool fresh(uint64_t ctr, bool commit)
{
	uint64_t back;

	if (!ses.rx_any || ctr > ses.rx_top) {
		if (commit) {
			back = ses.rx_any ? ctr - ses.rx_top : 64U;
			ses.rx_seen = back >= 64U ? 0U : ses.rx_seen << back;
			ses.rx_seen |= 1U;
			ses.rx_top = ctr;
			ses.rx_any = true;
		}
		return true;
	}
	back = ses.rx_top - ctr;
	if (back >= 64U || (ses.rx_seen & BIT64(back)) != 0U) {
		return false;
	}
	if (commit) {
		ses.rx_seen |= BIT64(back);
	}

	return true;
}

int od_sec_open(const struct od_transport *tp, const uint8_t *req, size_t len,
		const uint8_t **payload, size_t *plen)
{
	const uint8_t *nonce = &req[2];
	size_t ctlen;
	size_t olen;
	uint64_t ctr;

	if (!session(tp) || len < ENV_OVERHEAD) {
		return -EACCES;
	}
	ctlen = len - 2U - NONCE_LEN;
	ctr = sys_get_be64(&nonce[8]);
	if (memcmp(nonce, ses.id, sizeof(ses.id)) != 0 || !fresh(ctr, false) ||
	    ctlen - TAG_LEN > sizeof(opened) ||
	    ccm(false, nonce, req, &req[2U + NONCE_LEN], ctlen, opened, sizeof(opened), &olen) !=
		    0 ||
	    olen < 1U || opened[0] > olen - 1U) {
		return -EBADMSG;
	}
	(void)fresh(ctr, true);
	*payload = &opened[1];
	*plen = opened[0];

	return 0;
}

int od_sec_send(const struct od_transport *tp, const uint8_t *buf, size_t len)
{
	static uint8_t pt[1U + OD_SEC_MAX_PAYLOAD];
	uint8_t *nonce = &sealed[2];
	size_t olen;
	int err;

	/* FW_VERSION is part of the handshake and always plain. */
	if (len < 2U || (buf[0] == OD_ACK && buf[1] == (uint8_t)OD_CMD_FW_VERSION) ||
	    !session(tp)) {
		return tp->send(tp, buf, len);
	}
	if (len - 2U > OD_SEC_MAX_PAYLOAD) {
		return -EMSGSIZE;
	}

	pt[0] = (uint8_t)(len - 2U);
	memcpy(&pt[1], &buf[2], len - 2U);
	sealed[0] = buf[0];
	sealed[1] = buf[1];
	memcpy(nonce, ses.id, sizeof(ses.id));
	sys_put_be64(ses.tx++, &nonce[8]);
	err = ccm(true, nonce, sealed, pt, len - 1U, &sealed[2U + NONCE_LEN],
		  sizeof(sealed) - 2U - NONCE_LEN, &olen);
	if (err != 0) {
		return err;
	}

	return tp->send(tp, sealed, 2U + NONCE_LEN + olen);
}

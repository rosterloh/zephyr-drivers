/*
 * Noise_NNpsk0_25519_ChaChaPoly_SHA256 on PSA (Noise spec rev 34, sections 5 and 9).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "noise.h"

#define PROTOCOL_NAME "Noise_NNpsk0_25519_ChaChaPoly_SHA256"
#define PROLOGUE      "NoiseAPIInit\0"

static void wipe(void *p, size_t len)
{
	volatile uint8_t *v = p;

	while (len-- > 0) {
		*v++ = 0;
	}
}

static int hmac(const uint8_t key[NOISE_KEY_LEN], const uint8_t *data, size_t len,
		uint8_t out[NOISE_KEY_LEN])
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t id;
	psa_status_t st;
	size_t olen;

	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
	psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
	if (psa_import_key(&attr, key, NOISE_KEY_LEN, &id) != PSA_SUCCESS) {
		return -EIO;
	}
	st = psa_mac_compute(id, PSA_ALG_HMAC(PSA_ALG_SHA_256), data, len, out, NOISE_KEY_LEN,
			     &olen);
	psa_destroy_key(id);

	return st == PSA_SUCCESS ? 0 : -EIO;
}

/* Noise HKDF: up to three 32-byte outputs; out2/out3 may be NULL. */
static int hkdf(const uint8_t ck[NOISE_KEY_LEN], const uint8_t *ikm, size_t len, uint8_t *out1,
		uint8_t *out2, uint8_t *out3)
{
	uint8_t temp[NOISE_KEY_LEN];
	uint8_t buf[NOISE_KEY_LEN + 1];
	int ret;

	ret = hmac(ck, ikm, len, temp);
	buf[0] = 0x01;
	ret = ret ?: hmac(temp, buf, 1, out1);
	if (out2 != NULL) {
		memcpy(buf, out1, NOISE_KEY_LEN);
		buf[NOISE_KEY_LEN] = 0x02;
		ret = ret ?: hmac(temp, buf, sizeof(buf), out2);
	}
	if (out3 != NULL) {
		memcpy(buf, out2, NOISE_KEY_LEN);
		buf[NOISE_KEY_LEN] = 0x03;
		ret = ret ?: hmac(temp, buf, sizeof(buf), out3);
	}
	wipe(temp, sizeof(temp));
	wipe(buf, sizeof(buf));

	return ret;
}

static int import_cipher(const uint8_t k[NOISE_KEY_LEN], psa_key_id_t *id)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;

	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attr, PSA_ALG_CHACHA20_POLY1305);
	psa_set_key_type(&attr, PSA_KEY_TYPE_CHACHA20);
	psa_set_key_bits(&attr, 256);

	return psa_import_key(&attr, k, NOISE_KEY_LEN, id) == PSA_SUCCESS ? 0 : -EIO;
}

/* ChaChaPoly nonce: 32 zero bits then the counter, little endian. */
static int aead(psa_key_id_t id, uint64_t n, bool enc, const uint8_t *ad, size_t ad_len,
		uint8_t *buf, size_t len, size_t *olen)
{
	uint8_t nonce[12] = {0};
	psa_status_t st;

	sys_put_le64(n, &nonce[4]);
	if (enc) {
		st = psa_aead_encrypt(id, PSA_ALG_CHACHA20_POLY1305, nonce, sizeof(nonce), ad,
				      ad_len, buf, len, buf, len + NOISE_TAG_LEN, olen);
	} else {
		st = psa_aead_decrypt(id, PSA_ALG_CHACHA20_POLY1305, nonce, sizeof(nonce), ad,
				      ad_len, buf, len, buf, len, olen);
	}

	if (st == PSA_ERROR_INVALID_SIGNATURE) {
		return -EBADMSG;
	}
	return st == PSA_SUCCESS ? 0 : -EIO;
}

static int ss_aead(struct noise_ss *ss, bool enc, uint8_t *buf, size_t len, size_t *olen)
{
	psa_key_id_t id;
	int ret = import_cipher(ss->k, &id);

	ret = ret ?: aead(id, ss->n, enc, ss->h, sizeof(ss->h), buf, len, olen);
	psa_destroy_key(id);
	if (ret == 0) {
		ss->n++;
	}

	return ret;
}

void esphome_noise_init(struct noise_ss *ss)
{
	size_t olen;

	/* The name is longer than HASHLEN, so h = HASH(name). */
	BUILD_ASSERT(sizeof(PROTOCOL_NAME) - 1 > NOISE_KEY_LEN);
	(void)psa_hash_compute(PSA_ALG_SHA_256, PROTOCOL_NAME, sizeof(PROTOCOL_NAME) - 1, ss->h,
			       sizeof(ss->h), &olen);
	memcpy(ss->ck, ss->h, sizeof(ss->ck));
	ss->n = 0;
	/* "NoiseAPIInit" followed by two NUL bytes. */
	(void)esphome_noise_mix_hash(ss, PROLOGUE, sizeof(PROLOGUE));
}

int esphome_noise_mix_hash(struct noise_ss *ss, const uint8_t *data, size_t len)
{
	psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
	psa_status_t st;
	size_t olen;

	st = psa_hash_setup(&op, PSA_ALG_SHA_256);
	if (st == PSA_SUCCESS) {
		st = psa_hash_update(&op, ss->h, sizeof(ss->h));
	}
	if (st == PSA_SUCCESS) {
		st = psa_hash_update(&op, data, len);
	}
	if (st == PSA_SUCCESS) {
		st = psa_hash_finish(&op, ss->h, sizeof(ss->h), &olen);
	}
	if (st != PSA_SUCCESS) {
		psa_hash_abort(&op);
		return -EIO;
	}

	return 0;
}

int esphome_noise_mix_key(struct noise_ss *ss, const uint8_t *ikm, size_t len)
{
	ss->n = 0;
	return hkdf(ss->ck, ikm, len, ss->ck, ss->k, NULL);
}

int esphome_noise_mix_key_and_hash(struct noise_ss *ss, const uint8_t *ikm, size_t len)
{
	uint8_t temp_h[NOISE_KEY_LEN];
	int ret = hkdf(ss->ck, ikm, len, ss->ck, temp_h, ss->k);

	ss->n = 0;
	ret = ret ?: esphome_noise_mix_hash(ss, temp_h, sizeof(temp_h));
	wipe(temp_h, sizeof(temp_h));

	return ret;
}

int esphome_noise_encrypt_and_hash(struct noise_ss *ss, uint8_t *buf, size_t len)
{
	size_t olen;
	int ret = ss_aead(ss, true, buf, len, &olen);

	return ret ?: esphome_noise_mix_hash(ss, buf, olen);
}

int esphome_noise_decrypt_and_hash(struct noise_ss *ss, uint8_t *buf, size_t len)
{
	uint8_t ct[NOISE_TAG_LEN + 16];
	size_t olen;
	int ret;

	/* h mixes in the ciphertext, which decrypting in place overwrites. */
	if (len > sizeof(ct)) {
		return -EMSGSIZE;
	}
	memcpy(ct, buf, len);
	ret = ss_aead(ss, false, buf, len, &olen);

	return ret ?: esphome_noise_mix_hash(ss, ct, len);
}

int esphome_noise_split(struct noise_ss *ss, struct noise_cs *c1, struct noise_cs *c2)
{
	uint8_t k1[NOISE_KEY_LEN], k2[NOISE_KEY_LEN];
	int ret = hkdf(ss->ck, NULL, 0, k1, k2, NULL);

	*c1 = (struct noise_cs){.key = PSA_KEY_ID_NULL};
	*c2 = (struct noise_cs){.key = PSA_KEY_ID_NULL};
	ret = ret ?: import_cipher(k1, &c1->key);
	ret = ret ?: import_cipher(k2, &c2->key);
	if (ret != 0) {
		esphome_noise_cs_destroy(c1);
		esphome_noise_cs_destroy(c2);
	}
	wipe(k1, sizeof(k1));
	wipe(k2, sizeof(k2));
	wipe(ss, sizeof(*ss));

	return ret;
}

int esphome_noise_generate_e(psa_key_id_t *e, uint8_t pub[NOISE_KEY_LEN])
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	size_t olen;

	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
	psa_set_key_bits(&attr, 255);
	if (psa_generate_key(&attr, e) != PSA_SUCCESS) {
		return -EIO;
	}
	if (psa_export_public_key(*e, pub, NOISE_KEY_LEN, &olen) != PSA_SUCCESS ||
	    olen != NOISE_KEY_LEN) {
		psa_destroy_key(*e);
		return -EIO;
	}

	return 0;
}

int esphome_noise_respond(const uint8_t psk[NOISE_KEY_LEN], const uint8_t msg1[NOISE_HS_LEN],
			  psa_key_id_t e, uint8_t msg2[NOISE_HS_LEN], struct noise_cs *rx,
			  struct noise_cs *tx)
{
	struct noise_ss ss;
	uint8_t tag[NOISE_TAG_LEN];
	uint8_t dh[NOISE_KEY_LEN];
	size_t olen;
	int ret;

	esphome_noise_init(&ss);
	/* <- psk, e */
	ret = esphome_noise_mix_key_and_hash(&ss, psk, NOISE_KEY_LEN);
	ret = ret ?: esphome_noise_mix_hash(&ss, msg1, NOISE_KEY_LEN);
	ret = ret ?: esphome_noise_mix_key(&ss, msg1, NOISE_KEY_LEN);
	memcpy(tag, &msg1[NOISE_KEY_LEN], sizeof(tag));
	ret = ret ?: esphome_noise_decrypt_and_hash(&ss, tag, sizeof(tag));

	/* -> e, ee */
	if (ret == 0 && psa_export_public_key(e, msg2, NOISE_KEY_LEN, &olen) != PSA_SUCCESS) {
		ret = -EIO;
	}
	ret = ret ?: esphome_noise_mix_hash(&ss, msg2, NOISE_KEY_LEN);
	ret = ret ?: esphome_noise_mix_key(&ss, msg2, NOISE_KEY_LEN);
	if (ret == 0 && psa_raw_key_agreement(PSA_ALG_ECDH, e, msg1, NOISE_KEY_LEN, dh, sizeof(dh),
					      &olen) != PSA_SUCCESS) {
		ret = -EIO;
	}
	ret = ret ?: esphome_noise_mix_key(&ss, dh, sizeof(dh));
	ret = ret ?: esphome_noise_encrypt_and_hash(&ss, &msg2[NOISE_KEY_LEN], 0);
	ret = ret ?: esphome_noise_split(&ss, rx, tx);

	psa_destroy_key(e);
	wipe(dh, sizeof(dh));
	wipe(&ss, sizeof(ss));

	return ret;
}

int esphome_noise_encrypt(struct noise_cs *cs, uint8_t *buf, size_t len)
{
	size_t olen;
	int ret = aead(cs->key, cs->n, true, NULL, 0, buf, len, &olen);

	if (ret == 0) {
		cs->n++;
	}
	return ret;
}

int esphome_noise_decrypt(struct noise_cs *cs, uint8_t *buf, size_t len)
{
	size_t olen;
	int ret;

	if (len < NOISE_TAG_LEN) {
		return -EBADMSG;
	}
	ret = aead(cs->key, cs->n, false, NULL, 0, buf, len, &olen);
	if (ret == 0) {
		cs->n++;
	}
	return ret;
}

void esphome_noise_cs_destroy(struct noise_cs *cs)
{
	if (cs->key != PSA_KEY_ID_NULL) {
		psa_destroy_key(cs->key);
		cs->key = PSA_KEY_ID_NULL;
	}
}

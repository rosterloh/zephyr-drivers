/*
 * Noise_NNpsk0_25519_ChaChaPoly_SHA256 as ESPHome uses it. Internal to esphome_api;
 * the symmetric-state calls are exposed so the tests can play the initiator.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESPHOME_API_NOISE_H_
#define ESPHOME_API_NOISE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <psa/crypto.h>

#define NOISE_KEY_LEN 32
#define NOISE_TAG_LEN 16
/* Handshake messages: e (32) + tag of the empty payload (16). */
#define NOISE_HS_LEN  (NOISE_KEY_LEN + NOISE_TAG_LEN)

struct noise_ss {
	uint8_t ck[NOISE_KEY_LEN];
	uint8_t h[NOISE_KEY_LEN];
	uint8_t k[NOISE_KEY_LEN];
	uint64_t n;
};

/* Transport cipher for one direction. */
struct noise_cs {
	psa_key_id_t key;
	uint64_t n;
};

void noise_init(struct noise_ss *ss);
int noise_mix_hash(struct noise_ss *ss, const uint8_t *data, size_t len);
int noise_mix_key(struct noise_ss *ss, const uint8_t *ikm, size_t len);
int noise_mix_key_and_hash(struct noise_ss *ss, const uint8_t *ikm, size_t len);
/* Encrypts @p len bytes of @p buf in place and appends the tag. */
int noise_encrypt_and_hash(struct noise_ss *ss, uint8_t *buf, size_t len);
/* @p len includes the tag; returns -EBADMSG on a tag mismatch. */
int noise_decrypt_and_hash(struct noise_ss *ss, uint8_t *buf, size_t len);
/* Initiator-to-responder cipher in @p c1, the other direction in @p c2. Wipes @p ss. */
int noise_split(struct noise_ss *ss, struct noise_cs *c1, struct noise_cs *c2);

/* X25519 key for an ephemeral, and its public half in @p pub. */
int noise_generate_e(psa_key_id_t *e, uint8_t pub[NOISE_KEY_LEN]);

/*
 * Responder side of the handshake: read @p msg1, write @p msg2 with ephemeral @p e (destroyed
 * on return) and set up the transport ciphers. Returns -EBADMSG when the PSK does not match.
 */
int noise_respond(const uint8_t psk[NOISE_KEY_LEN], const uint8_t msg1[NOISE_HS_LEN],
		  psa_key_id_t e, uint8_t msg2[NOISE_HS_LEN], struct noise_cs *rx,
		  struct noise_cs *tx);

/* Transport: in place, @p buf must have NOISE_TAG_LEN spare bytes. */
int noise_encrypt(struct noise_cs *cs, uint8_t *buf, size_t len);
int noise_decrypt(struct noise_cs *cs, uint8_t *buf, size_t len);
void noise_cs_destroy(struct noise_cs *cs);

#endif /* ESPHOME_API_NOISE_H_ */

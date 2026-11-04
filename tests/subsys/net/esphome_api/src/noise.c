/*
 * Noise: known-answer vectors from the Python noise library aioesphomeapi uses, then the
 * server over loopback with this file playing the initiator.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/esphome_api.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>
#include <api.pb.h>
#include <noise.h>

/* gen_noise_vectors.py: psk 00..1f, initiator e 0x11.., responder e 0x22..; Ping both ways. */
static const uint8_t psk[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
			      0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
			      0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
static const uint8_t ie_priv[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
				  0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
				  0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
static const uint8_t re_priv[] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
				  0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
				  0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
static const uint8_t msg1[] = {0x7b, 0x4e, 0x90, 0x9b, 0xbe, 0x7f, 0xfe, 0x44, 0xc4, 0x65,
			       0xa2, 0x20, 0x03, 0x7d, 0x60, 0x8e, 0xe3, 0x58, 0x97, 0xd3,
			       0x1e, 0xf9, 0x72, 0xf0, 0x7f, 0x74, 0x89, 0x2c, 0xb0, 0xf7,
			       0x3f, 0x13, 0xd1, 0xbc, 0x1a, 0x38, 0xb6, 0x79, 0x36, 0x85,
			       0xdc, 0x7e, 0x2f, 0xfd, 0xf8, 0xea, 0x09, 0x33};
static const uint8_t msg2[] = {0x0f, 0xaa, 0x68, 0x4e, 0xd2, 0x88, 0x67, 0xb9, 0x7f, 0x4a,
			       0x6a, 0x2d, 0xee, 0x5d, 0xf8, 0xce, 0x97, 0x4e, 0x76, 0xb7,
			       0x01, 0x8e, 0x3f, 0x22, 0xa1, 0xc4, 0xcf, 0x26, 0x78, 0x57,
			       0x0f, 0x20, 0x73, 0x36, 0xb9, 0x59, 0x33, 0xb3, 0x21, 0xb1,
			       0xae, 0x86, 0x67, 0x3f, 0x68, 0x9f, 0x5c, 0x9f};
static const uint8_t c2s[] = {0x08, 0x2a, 0x88, 0xd4, 0x45, 0x7b, 0x72, 0x64, 0x6c, 0xa1,
			      0xef, 0x30, 0x4b, 0xe3, 0x4f, 0x8f, 0xfa, 0xc1, 0xf6, 0x59};
static const uint8_t s2c[] = {0xd9, 0x42, 0x3b, 0xf4, 0x8b, 0xde, 0xa6, 0x2b, 0xb1, 0x0b,
			      0xbd, 0x87, 0xa7, 0xfb, 0x5e, 0xcc, 0x26, 0x72, 0x10, 0x50};

static psa_key_id_t import_e(const uint8_t priv[32])
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t id;

	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
	psa_set_key_bits(&attr, 255);
	zassert_equal(psa_import_key(&attr, priv, 32, &id), PSA_SUCCESS);

	return id;
}

/* Initiator side, as aioesphomeapi runs it; the server tests use it too. */
static void initiator_write(struct noise_ss *ss, const uint8_t key[32], psa_key_id_t e,
			    uint8_t m1[NOISE_HS_LEN])
{
	size_t olen;

	noise_init(ss);
	zassert_ok(noise_mix_key_and_hash(ss, key, 32));
	zassert_equal(psa_export_public_key(e, m1, 32, &olen), PSA_SUCCESS);
	zassert_ok(noise_mix_hash(ss, m1, 32));
	zassert_ok(noise_mix_key(ss, m1, 32));
	zassert_ok(noise_encrypt_and_hash(ss, &m1[32], 0));
}

static void initiator_read(struct noise_ss *ss, psa_key_id_t e, const uint8_t m2[NOISE_HS_LEN],
			   struct noise_cs *c1, struct noise_cs *c2)
{
	uint8_t tag[NOISE_TAG_LEN];
	uint8_t dh[32];
	size_t olen;

	zassert_ok(noise_mix_hash(ss, m2, 32));
	zassert_ok(noise_mix_key(ss, m2, 32));
	zassert_equal(psa_raw_key_agreement(PSA_ALG_ECDH, e, m2, 32, dh, sizeof(dh), &olen),
		      PSA_SUCCESS);
	psa_destroy_key(e);
	zassert_ok(noise_mix_key(ss, dh, sizeof(dh)));
	memcpy(tag, &m2[32], sizeof(tag));
	zassert_ok(noise_decrypt_and_hash(ss, tag, sizeof(tag)));
	zassert_ok(noise_split(ss, c1, c2));
}

ZTEST(esphome_api_noise, test_initiator_kat)
{
	struct noise_ss ss;
	struct noise_cs c1, c2;
	psa_key_id_t e = import_e(ie_priv);
	uint8_t m1[NOISE_HS_LEN];
	uint8_t frame[sizeof(s2c)];

	initiator_write(&ss, psk, e, m1);
	zassert_mem_equal(m1, msg1, sizeof(msg1));
	initiator_read(&ss, e, msg2, &c1, &c2);

	memcpy(frame, "\x00\x07\x00\x00", 4);
	zassert_ok(noise_encrypt(&c1, frame, 4));
	zassert_mem_equal(frame, c2s, sizeof(c2s));
	memcpy(frame, s2c, sizeof(s2c));
	zassert_ok(noise_decrypt(&c2, frame, sizeof(frame)));
	zassert_mem_equal(frame, "\x00\x08\x00\x00", 4);

	noise_cs_destroy(&c1);
	noise_cs_destroy(&c2);
}

ZTEST(esphome_api_noise, test_respond_kat)
{
	struct noise_cs rx, tx;
	uint8_t out[NOISE_HS_LEN];
	uint8_t frame[sizeof(c2s)];

	zassert_ok(noise_respond(psk, msg1, import_e(re_priv), out, &rx, &tx));
	zassert_mem_equal(out, msg2, sizeof(msg2));

	memcpy(frame, c2s, sizeof(c2s));
	zassert_ok(noise_decrypt(&rx, frame, sizeof(frame)));
	zassert_mem_equal(frame, "\x00\x07\x00\x00", 4);
	/* Replayed frame: the nonce has moved on. */
	memcpy(frame, c2s, sizeof(c2s));
	zassert_equal(noise_decrypt(&rx, frame, sizeof(frame)), -EBADMSG);

	memcpy(frame, "\x00\x08\x00\x00", 4);
	zassert_ok(noise_encrypt(&tx, frame, 4));
	zassert_mem_equal(frame, s2c, sizeof(s2c));

	noise_cs_destroy(&rx);
	noise_cs_destroy(&tx);
}

ZTEST(esphome_api_noise, test_respond_wrong_psk)
{
	struct noise_cs rx = {0}, tx = {0};
	uint8_t bad[32];
	uint8_t out[NOISE_HS_LEN];

	memcpy(bad, psk, sizeof(bad));
	bad[0] ^= 1;
	zassert_equal(noise_respond(bad, msg1, import_e(re_priv), out, &rx, &tx), -EBADMSG);
	zassert_equal(rx.key, PSA_KEY_ID_NULL);
}

/* Server over loopback. */

static int open_fd = -1;

static int connect_server(void)
{
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(CONFIG_ESPHOME_API_PORT),
	};
	struct zsock_timeval tv = {.tv_sec = 1};
	int fd = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);

	zassert_true(fd >= 0);
	open_fd = fd;
	zassert_equal(zsock_inet_pton(NET_AF_INET, "127.0.0.1", &addr.sin_addr), 1);
	zassert_ok(zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &tv, sizeof(tv)));
	zassert_ok(zsock_connect(fd, (struct net_sockaddr *)&addr, sizeof(addr)), "connect: %d",
		   errno);

	return fd;
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

/* One 0x01 frame into buf; returns its length. */
static int recv_noise_frame(int fd, uint8_t *buf, size_t size)
{
	uint8_t hdr[3];
	size_t len;
	int ret = recv_exact(fd, hdr, sizeof(hdr));

	if (ret < 0) {
		return ret;
	}
	zassert_equal(hdr[0], 0x01);
	len = sys_get_be16(&hdr[1]);
	zassert_true(len <= size);
	ret = recv_exact(fd, buf, len);

	return ret < 0 ? ret : (int)len;
}

static void send_noise_frame(int fd, const uint8_t *payload, size_t len)
{
	uint8_t hdr[3] = {0x01, len >> 8, len & 0xff};

	zassert_equal(zsock_send(fd, hdr, sizeof(hdr), 0), sizeof(hdr));
	if (len > 0) {
		zassert_equal(zsock_send(fd, payload, len, 0), len);
	}
}

static void expect_closed(int fd)
{
	uint8_t b;

	zassert_true(zsock_recv(fd, &b, 1, 0) <= 0);
	zsock_close(fd);
	open_fd = -1;
}

/* Hello and handshake as aioesphomeapi does them; returns the server's reply frame. */
static int handshake(int fd, const uint8_t key[32], struct noise_ss *ss, psa_key_id_t *e,
		     uint8_t *reply, size_t size)
{
	uint8_t m1[1 + NOISE_HS_LEN] = {0x00};
	char hello[64];
	int len;

	zassert_ok(noise_generate_e(e, &m1[1]));
	initiator_write(ss, key, *e, &m1[1]);

	send_noise_frame(fd, NULL, 0);
	send_noise_frame(fd, m1, sizeof(m1));

	len = recv_noise_frame(fd, hello, sizeof(hello));
	zassert_true(len > 0);
	zassert_equal(hello[0], 0x01, "chosen protocol");
	zassert_str_equal(&hello[1], CONFIG_ESPHOME_API_DEVICE_NAME);
	zassert_equal(strlen(&hello[1 + strlen(&hello[1]) + 1]), 12, "mac");

	return recv_noise_frame(fd, reply, size);
}

ZTEST(esphome_api_noise, test_server_ping)
{
	int fd = connect_server();
	struct noise_ss ss;
	struct noise_cs c1, c2;
	psa_key_id_t e;
	uint8_t frame[64];
	int len;

	len = handshake(fd, psk, &ss, &e, frame, sizeof(frame));
	zassert_equal(len, 1 + NOISE_HS_LEN);
	zassert_equal(frame[0], 0x00);
	initiator_read(&ss, e, &frame[1], &c1, &c2);

	/* Two pings check that both nonces advance. */
	for (int i = 0; i < 2; i++) {
		sys_put_be16(PingRequest_msgid, frame);
		sys_put_be16(0, &frame[2]);
		zassert_ok(noise_encrypt(&c1, frame, 4));
		send_noise_frame(fd, frame, 4 + NOISE_TAG_LEN);

		len = recv_noise_frame(fd, frame, sizeof(frame));
		zassert_equal(len, 4 + NOISE_TAG_LEN);
		zassert_ok(noise_decrypt(&c2, frame, len));
		zassert_equal(sys_get_be16(frame), PingResponse_msgid);
		zassert_equal(sys_get_be16(&frame[2]), 0);
	}

	noise_cs_destroy(&c1);
	noise_cs_destroy(&c2);
	zsock_close(fd);
	open_fd = -1;
}

ZTEST(esphome_api_noise, test_server_wrong_psk)
{
	int fd = connect_server();
	struct noise_ss ss;
	psa_key_id_t e;
	uint8_t bad[32] = {0};
	uint8_t frame[64];
	int len;

	len = handshake(fd, bad, &ss, &e, frame, sizeof(frame));
	psa_destroy_key(e);
	zassert_true(len > 1);
	frame[len] = '\0';
	zassert_equal(frame[0], 0x01, "reject marker");
	zassert_str_equal((char *)&frame[1], "Handshake MAC failure");
	expect_closed(fd);
}

ZTEST(esphome_api_noise, test_server_rejects_plaintext)
{
	int fd = connect_server();
	/* Plaintext HelloRequest with an empty body. */
	const uint8_t hello[] = {0x00, 0x00, 0x01};
	uint8_t frame[64];
	int len;

	zassert_equal(zsock_send(fd, hello, sizeof(hello), 0), sizeof(hello));
	/* aioesphomeapi's plaintext client reads the 0x01 marker as "requires encryption". */
	len = recv_noise_frame(fd, frame, sizeof(frame) - 1);
	zassert_true(len > 1);
	zassert_equal(frame[0], 0x01);
	expect_closed(fd);
}

static void *setup(void)
{
	zassert_equal(psa_crypto_init(), PSA_SUCCESS);
	/* Let the server thread open its listener. */
	k_sleep(K_MSEC(100));
	return NULL;
}

static void after(void *fixture)
{
	if (open_fd >= 0) {
		zsock_close(open_fd);
		open_fd = -1;
		k_sleep(K_MSEC(50));
	}
}

ZTEST_SUITE(esphome_api_noise, NULL, setup, NULL, after, NULL);

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/cdr/cdr.h>
#include <zephyr/ztest.h>

ZTEST_SUITE(cdr, NULL, NULL, NULL, NULL, NULL);

/* Encapsulation header then a u8, an aligned u32 and an aligned f64 must
 * produce exactly the bytes CDR mandates, including inter-field padding. */
ZTEST(cdr, test_known_vector)
{
	uint8_t buf[32];
	struct cdr_writer w;

	cdr_writer_init(&w, buf, sizeof(buf));
	cdr_write_encapsulation(&w);
	cdr_write_u8(&w, 0x01);
	cdr_write_u32(&w, 0xAABBCCDDu);

	static const uint8_t expected[] = {
		0x00, 0x01, 0x00, 0x00, /* CDR_LE encapsulation */
		0x01,                   /* u8 */
		0x00, 0x00, 0x00,       /* padding to 4-byte boundary */
		0xDD, 0xCC, 0xBB, 0xAA, /* u32, little-endian */
	};

	zassert_equal(cdr_writer_finish(&w), sizeof(expected), "wrong length");
	zassert_mem_equal(buf, expected, sizeof(expected), "wrong bytes");
}

ZTEST(cdr, test_scalar_roundtrip)
{
	uint8_t buf[64];
	struct cdr_writer w;

	cdr_writer_init(&w, buf, sizeof(buf));
	cdr_write_encapsulation(&w);
	cdr_write_u32(&w, 0x12345678u);
	cdr_write_f64(&w, -0.25);
	size_t len = cdr_writer_finish(&w);

	zassert_not_equal(len, 0, "encode overflowed");

	struct cdr_reader r;

	cdr_reader_init(&r, buf, len);
	zassert_true(cdr_read_encapsulation(&r), "missing encapsulation");
	zassert_equal(cdr_read_u32(&r), 0x12345678u, "u32 mismatch");
	zassert_equal(cdr_read_f64(&r), -0.25, "f64 mismatch");
	zassert_true(cdr_reader_ok(&r), "reader failed");
}

ZTEST(cdr, test_string_roundtrip)
{
	uint8_t buf[64];
	struct cdr_writer w;

	cdr_writer_init(&w, buf, sizeof(buf));
	cdr_write_encapsulation(&w);
	cdr_write_string(&w, "pan_joint");
	cdr_write_string(&w, "");
	size_t len = cdr_writer_finish(&w);

	zassert_not_equal(len, 0, "encode overflowed");

	struct cdr_reader r;
	const char *s;
	uint32_t slen;

	cdr_reader_init(&r, buf, len);
	zassert_true(cdr_read_encapsulation(&r), "missing encapsulation");
	zassert_true(cdr_read_string_ref(&r, &s, &slen), "string read failed");
	zassert_equal(slen, sizeof("pan_joint"), "wrong string length");
	zassert_mem_equal(s, "pan_joint", slen, "wrong string bytes");

	/* Second (empty) string can be skipped. */
	cdr_skip_string(&r);
	zassert_true(cdr_reader_ok(&r), "skip failed");
}

ZTEST(cdr, test_overflow_reports_zero)
{
	uint8_t buf[6]; /* room for encapsulation + a bit, not the u32 */
	struct cdr_writer w;

	cdr_writer_init(&w, buf, sizeof(buf));
	cdr_write_encapsulation(&w);
	cdr_write_u32(&w, 0xDEADBEEFu);

	zassert_equal(cdr_writer_finish(&w), 0, "overflow not detected");
}

ZTEST(cdr, test_reader_rejects_bad_encapsulation)
{
	const uint8_t bad[] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct cdr_reader r;

	cdr_reader_init(&r, bad, sizeof(bad));
	zassert_false(cdr_read_encapsulation(&r), "bad header accepted");
	zassert_false(cdr_reader_ok(&r), "reader should be failed");
}

ZTEST(cdr, test_reader_underflow)
{
	uint8_t buf[16];
	struct cdr_writer w;

	cdr_writer_init(&w, buf, sizeof(buf));
	cdr_write_encapsulation(&w);
	cdr_write_u32(&w, 1);
	size_t len = cdr_writer_finish(&w);

	struct cdr_reader r;

	cdr_reader_init(&r, buf, len);
	zassert_true(cdr_read_encapsulation(&r), "missing encapsulation");
	(void)cdr_read_u32(&r);
	(void)cdr_read_u32(&r); /* nothing left */
	zassert_false(cdr_reader_ok(&r), "underflow not detected");
}

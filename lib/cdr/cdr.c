/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/cdr/cdr.h>

#define CDR_ENCAPSULATION_SIZE 4

/* CDR_LE representation identifier (0x0001) followed by its options field. */
static const uint8_t cdr_le_header[CDR_ENCAPSULATION_SIZE] = {0x00, 0x01, 0x00, 0x00};

static size_t cdr_write_rel(const struct cdr_writer *w)
{
	return (w->len >= w->origin) ? (w->len - w->origin) : 0;
}

static size_t cdr_read_rel(const struct cdr_reader *r)
{
	return (r->off >= r->origin) ? (r->off - r->origin) : 0;
}

static void cdr_reserve(struct cdr_writer *w, size_t len)
{
	if (w->len + len > w->cap) {
		w->ok = false;
	}
}

static void cdr_write_zeroes(struct cdr_writer *w, size_t len)
{
	cdr_reserve(w, len);
	if (!w->ok) {
		return;
	}
	memset(w->buf + w->len, 0, len);
	w->len += len;
}

static void cdr_align(struct cdr_writer *w, size_t alignment)
{
	size_t rem = cdr_write_rel(w) % alignment;

	if (rem != 0) {
		cdr_write_zeroes(w, alignment - rem);
	}
}

static void cdr_read_align(struct cdr_reader *r, size_t alignment)
{
	size_t rem = cdr_read_rel(r) % alignment;

	if (rem != 0) {
		r->off += alignment - rem;
	}
	if (r->off > r->len) {
		r->ok = false;
	}
}

void cdr_writer_init(struct cdr_writer *w, uint8_t *buf, size_t cap)
{
	w->buf = buf;
	w->len = 0;
	w->cap = cap;
	w->origin = 0;
	w->ok = true;
}

void cdr_write_bytes(struct cdr_writer *w, const void *src, size_t len)
{
	cdr_reserve(w, len);
	if (!w->ok) {
		return;
	}
	memcpy(w->buf + w->len, src, len);
	w->len += len;
}

void cdr_write_encapsulation(struct cdr_writer *w)
{
	cdr_write_bytes(w, cdr_le_header, sizeof(cdr_le_header));
	w->origin = w->len;
}

void cdr_write_u8(struct cdr_writer *w, uint8_t v)
{
	cdr_write_bytes(w, &v, sizeof(v));
}

void cdr_write_u32(struct cdr_writer *w, uint32_t v)
{
	uint8_t bytes[4] = {
		(uint8_t)v,
		(uint8_t)(v >> 8),
		(uint8_t)(v >> 16),
		(uint8_t)(v >> 24),
	};

	cdr_align(w, 4);
	cdr_write_bytes(w, bytes, sizeof(bytes));
}

void cdr_write_f32(struct cdr_writer *w, float v)
{
	cdr_align(w, 4);
	cdr_write_bytes(w, &v, sizeof(v));
}

void cdr_write_f64(struct cdr_writer *w, double v)
{
	cdr_align(w, 8);
	cdr_write_bytes(w, &v, sizeof(v));
}

void cdr_write_string(struct cdr_writer *w, const char *s)
{
	size_t len = strlen(s) + 1;

	cdr_write_u32(w, (uint32_t)len);
	cdr_write_bytes(w, s, len);
}

size_t cdr_writer_finish(const struct cdr_writer *w)
{
	return w->ok ? w->len : 0;
}

void cdr_reader_init(struct cdr_reader *r, const uint8_t *buf, size_t len)
{
	r->buf = buf;
	r->len = len;
	r->off = 0;
	r->origin = 0;
	r->ok = true;
}

bool cdr_read_encapsulation(struct cdr_reader *r)
{
	if (r->len < CDR_ENCAPSULATION_SIZE ||
	    memcmp(r->buf, cdr_le_header, CDR_ENCAPSULATION_SIZE) != 0) {
		r->ok = false;
		return false;
	}
	r->off = CDR_ENCAPSULATION_SIZE;
	r->origin = CDR_ENCAPSULATION_SIZE;
	return true;
}

uint32_t cdr_read_u32(struct cdr_reader *r)
{
	cdr_read_align(r, 4);
	if (!r->ok || r->len - r->off < sizeof(uint32_t)) {
		r->ok = false;
		return 0;
	}

	uint32_t v = (uint32_t)r->buf[r->off] | ((uint32_t)r->buf[r->off + 1] << 8) |
		     ((uint32_t)r->buf[r->off + 2] << 16) | ((uint32_t)r->buf[r->off + 3] << 24);
	r->off += sizeof(uint32_t);
	return v;
}

double cdr_read_f64(struct cdr_reader *r)
{
	cdr_read_align(r, 8);
	if (!r->ok || r->len - r->off < sizeof(uint64_t)) {
		r->ok = false;
		return 0.0;
	}

	uint64_t bits =
		(uint64_t)r->buf[r->off] | ((uint64_t)r->buf[r->off + 1] << 8) |
		((uint64_t)r->buf[r->off + 2] << 16) | ((uint64_t)r->buf[r->off + 3] << 24) |
		((uint64_t)r->buf[r->off + 4] << 32) | ((uint64_t)r->buf[r->off + 5] << 40) |
		((uint64_t)r->buf[r->off + 6] << 48) | ((uint64_t)r->buf[r->off + 7] << 56);
	double v;

	memcpy(&v, &bits, sizeof(v));
	r->off += sizeof(uint64_t);
	return v;
}

void cdr_skip_string(struct cdr_reader *r)
{
	uint32_t len = cdr_read_u32(r);

	if (!r->ok || r->len - r->off < len) {
		r->ok = false;
		return;
	}
	r->off += len;
}

bool cdr_read_string_ref(struct cdr_reader *r, const char **ptr, uint32_t *len)
{
	uint32_t n = cdr_read_u32(r);

	if (!r->ok || r->len - r->off < n) {
		r->ok = false;
		return false;
	}
	*ptr = (const char *)(r->buf + r->off);
	*len = n;
	r->off += n;
	return true;
}

bool cdr_reader_ok(const struct cdr_reader *r)
{
	return r->ok;
}

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_CDR_CDR_H_
#define ZEPHYR_INCLUDE_CDR_CDR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup cdr CDR serialization
 * @brief Little-endian Common Data Representation (CDR) codec.
 *
 * Encodes and decodes DDS/ROS 2 @c rmw payloads as consumed by zenoh-pico ROS
 * bridges. Only the primitive layer is provided: scalars, strings and the
 * 4-byte encapsulation header. Alignment follows the CDR rules relative to the
 * start of the body (the byte after the encapsulation header). Message layouts
 * are assembled by the caller on top of these primitives.
 * @{
 */

/** Serialization cursor. Treat the fields as private; use the API below. */
struct cdr_writer {
	uint8_t *buf;  /**< Destination buffer. */
	size_t len;    /**< Bytes written so far. */
	size_t cap;    /**< Capacity of @ref cdr_writer.buf. */
	size_t origin; /**< Offset the alignment is measured from. */
	bool ok;       /**< Cleared on overflow; sticky. */
};

/** Deserialization cursor. Treat the fields as private; use the API below. */
struct cdr_reader {
	const uint8_t *buf; /**< Source buffer. */
	size_t len;         /**< Length of @ref cdr_reader.buf. */
	size_t off;         /**< Current read offset. */
	size_t origin;      /**< Offset the alignment is measured from. */
	bool ok;            /**< Cleared on underflow/error; sticky. */
};

/** Initialise a writer over @p buf of @p cap bytes. */
void cdr_writer_init(struct cdr_writer *w, uint8_t *buf, size_t cap);

/** Write the CDR_LE encapsulation header and start the body at this offset. */
void cdr_write_encapsulation(struct cdr_writer *w);

/** Append @p len raw bytes with no alignment. */
void cdr_write_bytes(struct cdr_writer *w, const void *src, size_t len);

/** Append an unaligned octet. */
void cdr_write_u8(struct cdr_writer *w, uint8_t v);

/** Append a 4-byte aligned unsigned 32-bit integer. */
void cdr_write_u32(struct cdr_writer *w, uint32_t v);

/** Append a 4-byte aligned 32-bit float. */
void cdr_write_f32(struct cdr_writer *w, float v);

/** Append an 8-byte aligned 64-bit float. */
void cdr_write_f64(struct cdr_writer *w, double v);

/** Append a length-prefixed, NUL-terminated string. */
void cdr_write_string(struct cdr_writer *w, const char *s);

/** @return Bytes written, or 0 if any write overflowed the buffer. */
size_t cdr_writer_finish(const struct cdr_writer *w);

/** Initialise a reader over @p buf of @p len bytes. */
void cdr_reader_init(struct cdr_reader *r, const uint8_t *buf, size_t len);

/**
 * Validate and skip the CDR_LE encapsulation header.
 *
 * @return true if the header is present and little-endian, false otherwise
 *         (the reader is also marked failed).
 */
bool cdr_read_encapsulation(struct cdr_reader *r);

/** Read a 4-byte aligned unsigned 32-bit integer. */
uint32_t cdr_read_u32(struct cdr_reader *r);

/** Read an 8-byte aligned 64-bit float. */
double cdr_read_f64(struct cdr_reader *r);

/** Skip a length-prefixed string. */
void cdr_skip_string(struct cdr_reader *r);

/**
 * Borrow the next string without copying.
 *
 * @param r   Reader.
 * @param ptr Set to the string bytes (including the trailing NUL); valid only
 *            while the source buffer lives.
 * @param len Set to the byte length (including the NUL).
 * @return true on success, false on underflow (the reader is marked failed).
 */
bool cdr_read_string_ref(struct cdr_reader *r, const char **ptr, uint32_t *len);

/** @return false once any read on @p r has failed. */
bool cdr_reader_ok(const struct cdr_reader *r);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_CDR_CDR_H_ */

/*
 * Streaming zlib inflate for compressed direct write (window <= 512 bytes).
 *
 * uzlib pulls input and cannot pause mid-symbol, so compressed bytes are
 * buffered and only decoded while at least MARGIN bytes are queued: enough
 * for the largest dynamic block header plus the symbols behind a few output
 * bytes. The rest is decoded at END, when the stream is complete.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <uzlib.h>
#include <zephyr/sys/util.h>

#include "od_internal.h"

#define WINDOW 512U
#define MARGIN 640U
#define OUT    8U

static struct {
	struct uzlib_uncomp d;
	uint8_t dict[WINDOW];
	uint8_t in[2U * MARGIN];
	bool header;
	bool done;
} inf;

static int pump(bool final)
{
	uint8_t out[OUT];

	if (!inf.header) {
		int cinfo;

		if (inf.d.source_limit - inf.d.source < 2 && !final) {
			return 0;
		}
		cinfo = uzlib_zlib_parse_header(&inf.d);
		/* Window is 2^(cinfo + 8); larger ones need a bigger dictionary. */
		if (cinfo < 0 || cinfo > 1) {
			return -ENOTSUP;
		}
		inf.header = true;
	}

	while (!inf.done && (final || (size_t)(inf.d.source_limit - inf.d.source) >= MARGIN)) {
		int res;
		int err;

		inf.d.dest_start = out;
		inf.d.dest = out;
		inf.d.dest_limit = out + sizeof(out);
		res = uzlib_uncompress_chksum(&inf.d);
		if (res != TINF_OK && res != TINF_DONE) {
			return -EBADMSG;
		}
		err = od_direct_feed(out, inf.d.dest - out);
		if (err != 0) {
			return err;
		}
		inf.done = res == TINF_DONE;
	}

	return 0;
}

static int push(const uint8_t *buf, size_t len, bool final)
{
	for (;;) {
		size_t avail = inf.d.source_limit - inf.d.source;
		size_t n = MIN(len, sizeof(inf.in) - avail);
		int err;

		memmove(inf.in, inf.d.source, avail);
		if (n > 0U) {
			memcpy(&inf.in[avail], buf, n);
		}
		inf.d.source = inf.in;
		inf.d.source_limit = inf.in + avail + n;
		buf += n;
		len -= n;

		err = pump(final && len == 0U);
		if (err != 0 || len == 0U) {
			return err;
		}
	}
}

void od_inflate_start(void)
{
	uzlib_init();
	uzlib_uncompress_init(&inf.d, inf.dict, sizeof(inf.dict));
	/* Distances into unwritten history must not leak an older image. */
	memset(inf.dict, 0, sizeof(inf.dict));
	inf.d.source = inf.in;
	inf.d.source_limit = inf.in;
	inf.d.source_read_cb = NULL;
	inf.header = false;
	inf.done = false;
}

int od_inflate_data(const uint8_t *buf, size_t len)
{
	return push(buf, len, false);
}

int od_inflate_end(void)
{
	int err = push(NULL, 0U, true);

	return err != 0 ? err : (inf.done ? 0 : -EBADMSG);
}

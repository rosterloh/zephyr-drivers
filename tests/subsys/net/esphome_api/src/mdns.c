/*
 * Checks the DNS-SD record the mDNS responder advertises.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/version.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/ztest.h>

static const struct dns_sd_rec *find_rec(void)
{
	STRUCT_SECTION_FOREACH(dns_sd_rec, rec) {
		if (strcmp(rec->service, "_esphomelib") == 0) {
			return rec;
		}
	}
	return NULL;
}

/* Value of TXT key @p key, or NULL. */
static const char *txt_get(const struct dns_sd_rec *rec, const char *key, char *out, size_t size)
{
	size_t klen = strlen(key);

	for (size_t pos = 0; pos < rec->text_size;) {
		size_t len = (uint8_t)rec->text[pos];
		const char *kv = &rec->text[pos + 1];

		zassert_true(pos + 1 + len <= rec->text_size, "TXT string overruns record");
		if (len > klen && strncmp(kv, key, klen) == 0 && kv[klen] == '=') {
			zassert_true(len - klen <= size);
			memcpy(out, kv + klen + 1, len - klen - 1);
			out[len - klen - 1] = '\0';
			return out;
		}
		pos += 1 + len;
	}
	return NULL;
}

ZTEST(esphome_api_mdns, test_record)
{
	const struct dns_sd_rec *rec = find_rec();

	zassert_not_null(rec);
	zassert_str_equal(rec->instance, CONFIG_NET_HOSTNAME);
	zassert_str_equal(rec->proto, "_tcp");
	zassert_str_equal(rec->domain, "local");
	zassert_equal(net_ntohs(*rec->port), CONFIG_ESPHOME_API_PORT);
}

ZTEST(esphome_api_mdns, test_txt)
{
	const struct dns_sd_rec *rec = find_rec();
	char val[64];

	zassert_not_null(rec);
	zassert_not_null(txt_get(rec, "mac", val, sizeof(val)));
	zassert_equal(strlen(val), 12);
	for (int i = 0; i < 12; i++) {
		zassert_true((val[i] >= '0' && val[i] <= '9') || (val[i] >= 'a' && val[i] <= 'f'),
			     "mac must be lowercase hex: %s", val);
	}
	zassert_not_null(txt_get(rec, "version", val, sizeof(val)));
	zassert_str_equal(val, "Zephyr " KERNEL_VERSION_STRING);
	zassert_not_null(txt_get(rec, "board", val, sizeof(val)));
	zassert_str_equal(val, CONFIG_BOARD);
	zassert_not_null(txt_get(rec, "friendly_name", val, sizeof(val)));
	zassert_str_equal(val, CONFIG_ESPHOME_API_FRIENDLY_NAME);
	zassert_equal(txt_get(rec, "api_encryption", val, sizeof(val)) != NULL,
		      IS_ENABLED(CONFIG_ESPHOME_API_NOISE));
}

ZTEST_SUITE(esphome_api_mdns, NULL, NULL, NULL, NULL, NULL);

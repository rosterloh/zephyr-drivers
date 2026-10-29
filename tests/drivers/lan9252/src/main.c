/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/ethercat/lan9252.h>
#include <zephyr/ztest.h>

#define LAN9252_NODE DT_ALIAS(lan9252_0)

static const struct device *const lan9252 = DEVICE_DT_GET(LAN9252_NODE);

ZTEST_SUITE(lan9252_driver, NULL, NULL, NULL, NULL, NULL);

ZTEST(lan9252_driver, test_device_ready_and_identity)
{
	struct lan9252_id id;
	uint32_t byte_test;

	zassert_true(device_is_ready(lan9252), "LAN9252 device is not ready");
	zassert_ok(lan9252_get_id(lan9252, &id), "get_id failed");
	zassert_equal(id.chip_id, LAN9252_CHIP_ID, "unexpected chip id");
	zassert_equal(id.revision, 0x0001, "unexpected revision");

	zassert_ok(lan9252_read_reg(lan9252, LAN9252_REG_BYTE_TEST, &byte_test),
		   "BYTE_TEST read failed");
	zassert_equal(byte_test, LAN9252_BYTE_TEST_VALUE, "unexpected BYTE_TEST");
}

ZTEST(lan9252_driver, test_direct_register_read_write)
{
	uint32_t value;

	zassert_ok(lan9252_write_reg(lan9252, LAN9252_REG_GPT_CFG, 0xa5a55a5a),
		   "GPT_CFG write failed");
	zassert_ok(lan9252_read_reg(lan9252, LAN9252_REG_GPT_CFG, &value), "GPT_CFG read failed");
	zassert_equal(value, 0xa5a55a5a, "direct register round trip mismatch");
}

ZTEST(lan9252_driver, test_burst_register_access_uses_dword_increment)
{
	uint32_t write_values[] = {0x11223344, 0x55667788};
	uint32_t read_values[ARRAY_SIZE(write_values)];

	zassert_ok(lan9252_write_regs(lan9252, LAN9252_REG_GPT_CFG, write_values,
				      ARRAY_SIZE(write_values)),
		   "burst write failed");
	zassert_ok(lan9252_read_regs(lan9252, LAN9252_REG_GPT_CFG, read_values,
				     ARRAY_SIZE(read_values)),
		   "burst read failed");
	zassert_mem_equal(read_values, write_values, sizeof(read_values),
			  "burst register round trip mismatch");
}

ZTEST(lan9252_driver, test_esc_csr_read_write)
{
	uint8_t write_value[] = {0xde, 0xad, 0xbe, 0xef};
	uint8_t read_value[sizeof(write_value)];

	zassert_ok(lan9252_write_esc_reg(lan9252, 0x0120, write_value, sizeof(write_value)),
		   "ESC write failed");
	zassert_ok(lan9252_read_esc_reg(lan9252, 0x0120, read_value, sizeof(read_value)),
		   "ESC read failed");
	zassert_mem_equal(read_value, write_value, sizeof(read_value),
			  "ESC register round trip mismatch");
}

ZTEST(lan9252_driver, test_esc_csr_rejects_invalid_size)
{
	uint8_t value[3];

	zassert_equal(lan9252_read_esc_reg(lan9252, 0x0000, value, sizeof(value)), -EINVAL,
		      "3-byte ESC read should be rejected");
	zassert_equal(lan9252_write_esc_reg(lan9252, 0x0000, value, sizeof(value)), -EINVAL,
		      "3-byte ESC write should be rejected");
}

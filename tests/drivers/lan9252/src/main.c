/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/ethercat/lan9252.h>
#include <zephyr/ztest.h>

#include "lan9252_emul.h"

#define LAN9252_NODE DT_ALIAS(lan9252_0)

static const struct device *const lan9252 = DEVICE_DT_GET(LAN9252_NODE);
static const struct emul *const lan9252_emul = EMUL_DT_GET(LAN9252_NODE);

static void lan9252_after(void *fixture)
{
	ARG_UNUSED(fixture);

	lan9252_emul_set_csr_stuck_busy(lan9252_emul, false);
}

ZTEST_SUITE(lan9252_driver, NULL, NULL, NULL, lan9252_after, NULL);

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

ZTEST(lan9252_driver, test_esc_csr_byte_and_word_lanes)
{
	const uint8_t word[] = {0x34, 0x12};
	const uint8_t byte = 0x56;
	const uint8_t expected[] = {0x34, 0x12, 0x56, 0x00};
	const uint8_t zero[4] = {0};
	uint8_t dword[4];
	uint8_t readback_word[2];
	uint8_t readback_byte;

	zassert_ok(lan9252_write_esc_reg(lan9252, 0x0130, zero, sizeof(zero)), "clear failed");
	zassert_ok(lan9252_write_esc_reg(lan9252, 0x0130, word, sizeof(word)), "word write failed");
	zassert_ok(lan9252_write_esc_reg(lan9252, 0x0132, &byte, sizeof(byte)),
		   "byte write failed");

	zassert_ok(lan9252_read_esc_reg(lan9252, 0x0130, dword, sizeof(dword)),
		   "dword read failed");
	zassert_mem_equal(dword, expected, sizeof(dword), "sub-dword writes landed on wrong bytes");

	zassert_ok(lan9252_read_esc_reg(lan9252, 0x0130, readback_word, sizeof(readback_word)),
		   "word read failed");
	zassert_mem_equal(readback_word, word, sizeof(word), "word read mismatch");
	zassert_ok(lan9252_read_esc_reg(lan9252, 0x0132, &readback_byte, 1), "byte read failed");
	zassert_equal(readback_byte, byte, "byte read mismatch");
}

ZTEST(lan9252_driver, test_rejects_misaligned_addresses)
{
	uint8_t buf[4] = {0};
	uint32_t value;

	zassert_equal(lan9252_read_esc_reg(lan9252, 0x0121, buf, 2), -EINVAL,
		      "odd-address word ESC access should be rejected");
	zassert_equal(lan9252_write_esc_reg(lan9252, 0x0122, buf, 4), -EINVAL,
		      "non-dword-aligned dword ESC access should be rejected");
	zassert_equal(lan9252_read_reg(lan9252, LAN9252_REG_GPT_CFG + 2, &value), -EINVAL,
		      "unaligned System CSR read should be rejected");
}

ZTEST(lan9252_driver, test_burst_length_bounds)
{
	uint32_t values[9] = {0};

	zassert_equal(lan9252_read_regs(lan9252, LAN9252_REG_GPT_CFG, values, 0), -EINVAL,
		      "zero-length burst should be rejected");
	zassert_equal(lan9252_write_regs(lan9252, LAN9252_REG_GPT_CFG, values, ARRAY_SIZE(values)),
		      -EINVAL, "burst over 8 dwords should be rejected");
}

ZTEST(lan9252_driver, test_esc_csr_busy_timeout)
{
	uint8_t value[4];

	lan9252_emul_set_csr_stuck_busy(lan9252_emul, true);

	zassert_equal(lan9252_read_esc_reg(lan9252, 0x0120, value, sizeof(value)), -ETIMEDOUT,
		      "a CSR stuck busy should time out");
	zassert_equal(lan9252_write_esc_reg(lan9252, 0x0120, value, sizeof(value)), -ETIMEDOUT,
		      "a later access should time out waiting for the stuck CSR");

	lan9252_emul_set_csr_stuck_busy(lan9252_emul, false);

	zassert_ok(lan9252_read_esc_reg(lan9252, 0x0120, value, sizeof(value)),
		   "ESC access should recover once the CSR is idle");
}

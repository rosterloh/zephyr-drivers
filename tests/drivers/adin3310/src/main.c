/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/ethernet/adin3310.h>
#include <zephyr/ztest.h>

#define ADIN3310_NODE DT_ALIAS(adin3310_0)

static const struct device *const adin3310 = DEVICE_DT_GET(ADIN3310_NODE);

ZTEST_SUITE(adin3310_driver, NULL, NULL, NULL, NULL, NULL);

ZTEST(adin3310_driver, test_device_ready_and_port_count)
{
	uint8_t port_count;

	zassert_true(device_is_ready(adin3310), "ADIN3310 device is not ready");
	zassert_ok(adin3310_get_port_count(adin3310, &port_count), "port count read failed");
	zassert_equal(port_count, ADIN3310_PORT_COUNT, "unexpected ADIN3310 port count");
}

ZTEST(adin3310_driver, test_management_transfer)
{
	const uint8_t tx[] = {0xa5, 0x5a, 0x03, 0x10};
	uint8_t rx[sizeof(tx)];

	zassert_ok(adin3310_transfer(adin3310, tx, rx, sizeof(tx)), "SPI transfer failed");
	zassert_mem_equal(rx, tx, sizeof(rx), "emulated management response mismatch");
}

ZTEST(adin3310_driver, test_management_transfer_rejects_bad_arguments)
{
	uint8_t byte = 0;

	zassert_equal(adin3310_transfer(NULL, &byte, &byte, sizeof(byte)), -EINVAL,
		      "NULL device should be rejected");
	zassert_equal(adin3310_transfer(adin3310, NULL, &byte, sizeof(byte)), -EINVAL,
		      "NULL TX buffer should be rejected");
	zassert_equal(adin3310_transfer(adin3310, &byte, NULL, sizeof(byte)), -EINVAL,
		      "NULL RX buffer should be rejected");
	zassert_equal(adin3310_transfer(adin3310, &byte, &byte, 0), -EINVAL,
		      "zero-length transfer should be rejected");
}

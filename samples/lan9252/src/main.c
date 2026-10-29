/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/ethercat/lan9252.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lan9252_sample, LOG_LEVEL_INF);

#define LAN9252_NODE DT_ALIAS(lan9252_0)

#if !DT_NODE_HAS_STATUS(LAN9252_NODE, okay)
#error "This sample requires a LAN9252 devicetree node with alias lan9252-0"
#endif

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(LAN9252_NODE);
	struct lan9252_id id;
	uint32_t byte_test;
	uint32_t hw_cfg;
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("%s is not ready", dev->name);
		return 0;
	}

	ret = lan9252_get_id(dev, &id);
	if (ret != 0) {
		LOG_ERR("failed to read ID_REV: %d", ret);
		return 0;
	}

	ret = lan9252_read_reg(dev, LAN9252_REG_BYTE_TEST, &byte_test);
	if (ret != 0) {
		LOG_ERR("failed to read BYTE_TEST: %d", ret);
		return 0;
	}

	ret = lan9252_read_reg(dev, LAN9252_REG_HW_CFG, &hw_cfg);
	if (ret != 0) {
		LOG_ERR("failed to read HW_CFG: %d", ret);
		return 0;
	}

	LOG_INF("LAN9252 chip=0x%04x revision=0x%04x", id.chip_id, id.revision);
	LOG_INF("BYTE_TEST=0x%08x HW_CFG=0x%08x", byte_test, hw_cfg);

	return 0;
}

/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ETHERCAT_LAN9252_H_
#define ZEPHYR_INCLUDE_DRIVERS_ETHERCAT_LAN9252_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LAN9252_CHIP_ID 0x9252U

#define LAN9252_REG_ID_REV                0x0050U
#define LAN9252_REG_IRQ_CFG               0x0054U
#define LAN9252_REG_INT_STS               0x0058U
#define LAN9252_REG_INT_EN                0x005cU
#define LAN9252_REG_BYTE_TEST             0x0064U
#define LAN9252_REG_HW_CFG                0x0074U
#define LAN9252_REG_PMT_CTRL              0x0084U
#define LAN9252_REG_GPT_CFG               0x008cU
#define LAN9252_REG_GPT_CNT               0x0090U
#define LAN9252_REG_FREE_RUN              0x009cU
#define LAN9252_REG_RESET_CTL             0x01f8U
#define LAN9252_REG_ECAT_CSR_DATA         0x0300U
#define LAN9252_REG_ECAT_CSR_CMD          0x0304U
#define LAN9252_REG_ECAT_PRAM_RD_ADDR_LEN 0x0308U
#define LAN9252_REG_ECAT_PRAM_RD_CMD      0x030cU
#define LAN9252_REG_ECAT_PRAM_WR_ADDR_LEN 0x0310U
#define LAN9252_REG_ECAT_PRAM_WR_CMD      0x0314U

#define LAN9252_BYTE_TEST_VALUE 0x87654321U
#define LAN9252_HW_CFG_READY    BIT(27)

struct lan9252_id {
	uint16_t chip_id;
	uint16_t revision;
};

int lan9252_read_reg(const struct device *dev, uint16_t addr, uint32_t *value);
int lan9252_write_reg(const struct device *dev, uint16_t addr, uint32_t value);
int lan9252_read_regs(const struct device *dev, uint16_t addr, uint32_t *values, size_t count);
int lan9252_write_regs(const struct device *dev, uint16_t addr, const uint32_t *values,
		       size_t count);
int lan9252_get_id(const struct device *dev, struct lan9252_id *id);
int lan9252_read_esc_reg(const struct device *dev, uint16_t addr, void *buf, size_t len);
int lan9252_write_esc_reg(const struct device *dev, uint16_t addr, const void *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETHERCAT_LAN9252_H_ */

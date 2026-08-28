/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_SENSOR_ICM20948_ICM20948_H_
#define DRIVERS_SENSOR_ICM20948_ICM20948_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

/* Datasheet: TDK InvenSense DS-000189, ICM-20948 v1.5 */

/*
 * The register map is BANKED. Every address below is only meaningful once the
 * matching bank is selected through REG_BANK_SEL, which is the one register
 * readable from all four banks. Forgetting the bank does not error -- it reads
 * or writes whatever else lives at that offset in the current bank, which is
 * why every access here goes through icm20948_bank_set() first.
 */
#define ICM20948_REG_BANK_SEL 0x7F
#define ICM20948_BANK_SHIFT   4

/* Bank 0 */
#define ICM20948_REG_WHO_AM_I     0x00
#define ICM20948_REG_PWR_MGMT_1   0x06
#define ICM20948_REG_PWR_MGMT_2   0x07
#define ICM20948_REG_ACCEL_XOUT_H 0x2D

#define ICM20948_WHO_AM_I_VAL 0xEA

#define ICM20948_PWR_MGMT_1_RESET    BIT(7)
#define ICM20948_PWR_MGMT_1_SLEEP    BIT(6)
#define ICM20948_PWR_MGMT_1_CLK_AUTO 0x01
#define ICM20948_PWR_MGMT_2_ALL_ON   0x00

/* Bank 2 */
#define ICM20948_REG_GYRO_CONFIG_1 0x01
#define ICM20948_REG_ACCEL_CONFIG  0x14

/* Both full-scale selects sit at bits 2:1 of their config register. */
#define ICM20948_FS_SEL_SHIFT 1

/* Burst read: accel XYZ, gyro XYZ, temperature, all big-endian 16-bit. */
#define ICM20948_SAMPLE_BYTES 14

/*
 * Temperature, per the datasheet's section 8.2: degC = raw/333.87 + 21. Held
 * as micro-units so the conversion stays in integer arithmetic.
 */
#define ICM20948_TEMP_SENSITIVITY_MILLI 333870
#define ICM20948_TEMP_OFFSET_MICRO      21000000

struct icm20948_config {
	struct i2c_dt_spec i2c;
	/* Full-scale register field values (0..3), which are also the indices
	 * into the sensitivity tables in icm20948.c. */
	uint8_t accel_fs;
	uint8_t gyro_fs;
};

struct icm20948_data {
	int16_t accel[3];
	int16_t gyro[3];
	int16_t temp;
};

#endif /* DRIVERS_SENSOR_ICM20948_ICM20948_H_ */

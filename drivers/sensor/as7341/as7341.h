#ifndef DRIVERS_SENSOR_AS7341_AS7341_H_
#define DRIVERS_SENSOR_AS7341_AS7341_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <drivers/sensor/as7341.h>

/* Datasheet: https://look.ams-osram.com/m/24266a3e584de4db/original/AS7341-DS000504.pdf */

/* Device Identification: (0x92): ID:7:2 | Reserved:1:0 */
#define AS7341_DEV_ID 0x9

/* Enable: (0x80): Reserved:7 | FDEN:6 | Reserved:5 | SMUXEN:4 | WEN:3 | Reserved:2 | SP_EN:1 | PON:0 */
#define AS7341_POWER_MASK 	(BIT(1) | BIT(0))
#define AS7341_POWER_ON   	(BIT(1) | BIT(0))
#define AS7341_POWER_OFF  	(0)
#define AS7341_WEN_MASK   	(BIT(3))
#define AS7341_WEN_ON     	(BIT(3))
#define AS7341_WEN_OFF    	(0)
#define AS7341_SMUXEN_MASK	(BIT(4))
#define AS7341_SMUXEN_ON  	(BIT(4))
#define AS7341_SMUXEN_OFF 	(0)
#define AS7341_FDEN_MASK	(BIT(6))
#define AS7341_FDEN_ON  	(BIT(6))
#define AS7341_FDEN_OFF 	(0)

/* Register Addresses */
#define AS7341_REG_ASTATUS	0x60
#define AS7341_REG_ITIME	0x63
#define AS7341_REG_CONFIG	0x70
#define AS7341_REG_STAT		0x71
#define AS7341_REG_EDGE		0x72
#define AS7341_REG_GPIO		0x73
#define AS7341_REG_LED		0x74
#define AS7341_REG_ENABLE	0x80
#define AS7341_REG_ATIME	0x81
#define AS7341_REG_WTIME	0x83
#define AS7341_REG_SP_LOW_TH	0x84
#define AS7341_REG_SP_HIGH_TH	0x86
#define AS7341_REG_AUXID	0x90
#define AS7341_REG_REVID	0x91
#define AS7341_REG_ID		0x92
#define AS7341_REG_STATUS	0x93
#define AS7341_REG_ASTATUS	0x94
#define AS7341_REG_CH0_DATA	0x95
#define AS7341_REG_CH1_DATA	0x97
#define AS7341_REG_CH2_DATA	0x99
#define AS7341_REG_CH3_DATA	0x9B
#define AS7341_REG_CH4_DATA	0x9D
#define AS7341_REG_CH5_DATA	0x9F
#define AS7341_REG_STATUS2	0xA3
#define AS7341_REG_STATUS3	0xA4
#define AS7341_REG_STATUS5	0xA6
#define AS7341_REG_STATUS6	0xA7
#define AS7341_REG_CFG0		0xA9
#define AS7341_REG_CFG1		0xAA
#define AS7341_REG_CFG3		0xAC
#define AS7341_REG_CFG6		0xAF
#define AS7341_REG_CFG8		0xB1
#define AS7341_REG_CFG9		0xB2
#define AS7341_REG_CFG10	0xB3
#define AS7341_REG_CFG12	0xB5
#define AS7341_REG_PERS		0xBD
#define AS7341_REG_GPIO2	0xBE
#define AS7341_REG_ASTEP	0xCA
#define AS7341_REG_AGC_GAIN_MAX	0xCF
#define AS7341_REG_AZ_CONFIG	0xD6
#define AS7341_REG_FD_TIME1	0xD8
#define AS7341_REG_FD_TIME2	0xDA
#define AS7341_REG_FD_CFG0	0xD7
#define AS7341_REG_FD_STATUS	0xDB
#define AS7341_REG_INTENAB	0xF9
#define AS7341_REG_CONTROL	0xFA
#define AS7341_REG_FIFO_MAP	0xFC
#define AS7341_REG_FIFO_LVL	0xFD
#define AS7341_REG_FDATA	0xFE

/* Gain Modes */
#define AS7341_GAIN_MODE_LOW  0x00
#define AS7341_GAIN_MODE_MED  0x10
#define AS7341_GAIN_MODE_HIGH 0x20
#define AS7341_GAIN_MODE_MAX  0x30

#define AS7341_SPECTRAL_INT_HIGH_MSK	(BIT(5))
#define AS7341_SPECTRAL_INT_LOW_MSK	(BIT(4))

struct as7341_config {
	const struct i2c_dt_spec i2c;
};

struct as7341_data {
	uint16_t vis_count;
	uint16_t ir_count;
	uint16_t again;
	uint16_t atime;
	bool powered_on;
};

#endif /* DRIVERS_SENSOR_AS7341_AS7341_H_ */
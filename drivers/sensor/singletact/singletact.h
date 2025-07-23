#ifndef DRIVERS_SENSOR_SINGLETACT_SINGLETACT_H_
#define DRIVERS_SENSOR_SINGLETACT_SINGLETACT_H_

#include <zephyr/types.h>
#include <drivers/sensor/singletact.h>

/* Datasheet:
   https://5361756.fs1.hubspotusercontent-na1.net/hubfs/5361756/SingleTact%20Documents/SingleTact_Datasheet.pdf
   Manual:
   https://5361756.fs1.hubspotusercontent-na1.net/hubfs/5361756/SingleTact%20Documents/SingleTact_Manual.pdf
 */

/* Register Addresses */
#define SINGLETACT_REG_ADDRESS       0x00
#define SINGLETACT_REG_SERIAL_MSB    0x01
#define SINGLETACT_REG_ACCUMULATOR   0x05
#define SINGLETACT_REG_GAIN          0x06
#define SINGLETACT_REG_FW_REV        0x07
#define SINGLETACT_REG_DIS_TIME      0x08
#define SINGLETACT_REG_OUT_CURR      0x09
#define SINGLETACT_REG_OUT_SCALE_MSB 0x0A
#define SINGLETACT_REG_NUM_ELEMENTS  0x0C
#define SINGLETACT_REG_CALIBRATED    0x0D
#define SINGLETACT_REG_BASELINE_MSB  0x29
#define SINGLETACT_REG_FRAME_IDX_MSB 0x80
#define SINGLETACT_REG_TIMESTAMP_MSB 0x82
#define SINGLETACT_REG_OUT_DATA_MSB  0x84

#define SINGLETACT_CMD_READ  0x01
#define SINGLETACT_CMD_WRITE 0x02

#define SINGLETACT_MAX_OUTPUT_VALUE 0x300
#define SINGLETACT_COUNT_OFFSET     0xFF

struct singletact_config {
	const struct i2c_dt_spec i2c;
	uint8_t baseline_samples;
	uint32_t sensor_rating;
};

struct singletact_data {
	uint16_t output_buffer[3];
	uint16_t serial;
	uint8_t fw;
	uint16_t baseline;
};

#endif /* DRIVERS_SENSOR_SINGLETACT_SINGLETACT_H_ */
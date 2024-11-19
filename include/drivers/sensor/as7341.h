/**
 * @file
 * @brief Extended public API for AMS's AS7341 multi-spectral sensor
 *
 * This exposes attributes for the AS7341 which can be used for
 * setting the on-chip gain, integration time, and persist filter parameters.
 */

#ifndef INCLUDE_DRIVERS_SENSOR_AS7341_H_
#define INCLUDE_DRIVERS_SENSOR_AS7341_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_attribute_as7341 {
	/* Sensor ADC Gain Mode */
	SENSOR_ATTR_GAIN_MODE = SENSOR_ATTR_PRIV_START + 1,

	/* Sensor ADC Integration Time (in us)
	 * ASTEP * (ATIME+1) where ASTEP is 2.78µs x (n+1):
	 *
	 * Default: ATIME=0 and ASTEP=999 (2.78ms)
	 */
	SENSOR_ATTR_INTEGRATION_TIME
};

enum sensor_gain_as7341 {
	AS7341_SENSOR_GAIN_0_5X,
	AS7341_SENSOR_GAIN_1X,
	AS7341_SENSOR_GAIN_2X,
	AS7341_SENSOR_GAIN_4X,
	AS7341_SENSOR_GAIN_8X,
	AS7341_SENSOR_GAIN_16X,
	AS7341_SENSOR_GAIN_32X,
	AS7341_SENSOR_GAIN_64X,
	AS7341_SENSOR_GAIN_128X,
	AS7341_SENSOR_GAIN_256X,
	AS7341_SENSOR_GAIN_512X,
};

enum senor_colour_channel_as7341 {
	AS7341_CHANNEL_415nm_F1,
	AS7341_CHANNEL_445nm_F2,
	AS7341_CHANNEL_480nm_F3,
	AS7341_CHANNEL_515nm_F4,
	AS7341_CHANNEL_CLEAR_0,
	AS7341_CHANNEL_NIR_0,
	AS7341_CHANNEL_555nm_F5,
	AS7341_CHANNEL_590nm_F6,
	AS7341_CHANNEL_630nm_F7,
	AS7341_CHANNEL_680nm_F8,
	AS7341_CHANNEL_CLEAR,
	AS7341_CHANNEL_NIR,
};

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_DRIVERS_SENSOR_AS7341_H_ */
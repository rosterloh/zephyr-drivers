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
	/* Sensor ADC Gain Mode
	 * Gain multipler can be one of the following:
	 *
	 * 0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 
	 */
	SENSOR_ATTR_GAIN_MODE = SENSOR_ATTR_PRIV_START + 1,

	/* Sensor ADC Integration Time (in us)
	 * ASTEP * (ATIME+1) where ASTEP is 2.78µs x (n+1):
	 *
	 * Default: ATIME=0 and ASTEP=
	 */
	SENSOR_ATTR_INTEGRATION_TIME
};

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_DRIVERS_SENSOR_AS7341_H_ */
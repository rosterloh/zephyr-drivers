/**
 * @file
 * @brief Extended public API for SingleTact force sensor
 *
 * This exposes attributes for the SingleTact which can be used for
 * setting the addres parameters.
 */

#ifndef INCLUDE_DRIVERS_SENSOR_SINGLETACT_H_
#define INCLUDE_DRIVERS_SENSOR_SINGLETACT_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_attribute_singletact {
	/* Sensor Address */
	SENSOR_ATTR_ADDRESS = SENSOR_ATTR_PRIV_START + 1,
};

enum sensor_channel_singletact {
	/* Raw output in counts */
	SENSOR_CHAN_RAW = SENSOR_CHAN_PRIV_START + 1,
	/* Force in N */
	SENSOR_CHAN_FORCE,
};

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_DRIVERS_SENSOR_SINGLETACT_H_ */
/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_ESPHOME_API_H_
#define ZEPHYR_INCLUDE_NET_ESPHOME_API_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/iterable_sections.h>

/**
 * @brief ESPHome native API server
 * @defgroup esphome_api ESPHome native API
 * @ingroup networking
 * @{
 */

/** API version reported in HelloResponse; matches the pinned api.proto. */
#define ESPHOME_API_VERSION_MAJOR 1
#define ESPHOME_API_VERSION_MINOR 19

/** Entity types. */
enum esphome_entity_type {
	ESPHOME_SENSOR,
	ESPHOME_BINARY_SENSOR,
	ESPHOME_SWITCH,
	ESPHOME_BUTTON,
};

/** Sensor state classes (api.proto SensorStateClass). */
enum esphome_state_class {
	ESPHOME_STATE_CLASS_NONE = 0,
	ESPHOME_STATE_CLASS_MEASUREMENT = 1,
	ESPHOME_STATE_CLASS_TOTAL_INCREASING = 2,
	ESPHOME_STATE_CLASS_TOTAL = 3,
	ESPHOME_STATE_CLASS_MEASUREMENT_ANGLE = 4,
};

/** Entity categories (api.proto EntityCategory). */
enum esphome_entity_category {
	ESPHOME_ENTITY_CATEGORY_NONE = 0,
	ESPHOME_ENTITY_CATEGORY_CONFIG = 1,
	ESPHOME_ENTITY_CATEGORY_DIAGNOSTIC = 2,
};

/**
 * An entity exposed to Home Assistant. Define with the ESPHOME_*_DEFINE macros;
 * fields other than the ones below "private" may be set in the macro arguments.
 */
struct esphome_entity {
	enum esphome_entity_type type;
	/** Stable id; Home Assistant keys the entity on it. Set from the macro's _id. */
	const char *object_id;
	/** Display name; object_id when NULL. */
	const char *name;
	/** Icon such as "mdi:thermometer", or NULL. */
	const char *icon;
	/** Home Assistant device class, or NULL. */
	const char *device_class;
	/** Sensor unit of measurement, or NULL. */
	const char *unit;
	/** Sensor decimals shown in Home Assistant. */
	int8_t accuracy_decimals;
	/** Sensor @ref esphome_state_class. */
	uint8_t state_class;
	/** @ref esphome_entity_category. */
	uint8_t entity_category;
	/** Switch command from Home Assistant. Call esphome_api_publish_bool() to confirm it. */
	void (*on_switch)(struct esphome_entity *entity, bool state);
	/** Button press from Home Assistant. */
	void (*on_press)(struct esphome_entity *entity);
	void *user_data;

	/* private */
	bool has_state;
	union {
		float f;
		bool b;
	} state;
};

#define Z_ESPHOME_ENTITY_DEFINE(_id, _type, ...)                                                   \
	STRUCT_SECTION_ITERABLE(esphome_entity,                                                    \
				_id) = {.type = _type, .object_id = #_id, __VA_ARGS__}

/**
 * Define a sensor. The variable name is the object_id; further fields are designated
 * initializers, e.g.
 * `ESPHOME_SENSOR_DEFINE(temperature, .name = "Temperature", .unit = "°C",
 *  .device_class = "temperature", .state_class = ESPHOME_STATE_CLASS_MEASUREMENT);`
 */
#define ESPHOME_SENSOR_DEFINE(_id, ...) Z_ESPHOME_ENTITY_DEFINE(_id, ESPHOME_SENSOR, __VA_ARGS__)

/** Define a binary sensor; see ESPHOME_SENSOR_DEFINE(). */
#define ESPHOME_BINARY_SENSOR_DEFINE(_id, ...)                                                     \
	Z_ESPHOME_ENTITY_DEFINE(_id, ESPHOME_BINARY_SENSOR, __VA_ARGS__)

/** Define a switch; set .on_switch. See ESPHOME_SENSOR_DEFINE(). */
#define ESPHOME_SWITCH_DEFINE(_id, ...) Z_ESPHOME_ENTITY_DEFINE(_id, ESPHOME_SWITCH, __VA_ARGS__)

/** Define a button; set .on_press. See ESPHOME_SENSOR_DEFINE(). */
#define ESPHOME_BUTTON_DEFINE(_id, ...) Z_ESPHOME_ENTITY_DEFINE(_id, ESPHOME_BUTTON, __VA_ARGS__)

/**
 * Set a sensor's state and send it to subscribed clients.
 * Callable from any thread, not from an ISR.
 *
 * @retval 0 on success
 * @retval -EINVAL if the entity is not a sensor
 */
int esphome_api_publish_float(struct esphome_entity *entity, float state);

/**
 * Set a binary sensor's or switch's state and send it to subscribed clients.
 * Callable from any thread, not from an ISR.
 *
 * @retval 0 on success
 * @retval -EINVAL if the entity is not a binary sensor or switch
 */
int esphome_api_publish_bool(struct esphome_entity *entity, bool state);

/** @} */

#endif /* ZEPHYR_INCLUDE_NET_ESPHOME_API_H_ */

/*
 * One sensor, switch and button exposed to Home Assistant.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/esphome_api.h>

LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);

static void relay_set(struct esphome_entity *e, bool state)
{
	LOG_INF("relay %s", state ? "on" : "off");
	/* Home Assistant shows the state the device reports, so confirm it. */
	esphome_api_publish_bool(e, state);
}

static void reset_press(struct esphome_entity *e)
{
	LOG_INF("reset pressed");
}

ESPHOME_SENSOR_DEFINE(temperature, .name = "Temperature", .unit = "°C",
		      .device_class = "temperature", .state_class = ESPHOME_STATE_CLASS_MEASUREMENT,
		      .accuracy_decimals = 1);
ESPHOME_SWITCH_DEFINE(relay, .name = "Relay", .icon = "mdi:electric-switch",
		      .on_switch = relay_set);
ESPHOME_BUTTON_DEFINE(reset, .name = "Reset", .device_class = "restart", .on_press = reset_press);

int main(void)
{
	float t = 20.0f;

	(void)esphome_api_publish_bool(&relay, false);
	while (true) {
		/* A stand-in reading that ramps 20.0 .. 24.5 °C. */
		(void)esphome_api_publish_float(&temperature, t);
		t = t >= 24.5f ? 20.0f : t + 0.5f;
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

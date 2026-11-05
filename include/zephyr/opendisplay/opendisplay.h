/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_OPENDISPLAY_OPENDISPLAY_H_
#define ZEPHYR_INCLUDE_OPENDISPLAY_OPENDISPLAY_H_

#include <stdint.h>

/**
 * @brief OpenDisplay device (receiver) side
 * @defgroup opendisplay OpenDisplay
 * @{
 *
 * Lets Home Assistant's ``opendisplay`` integration or py-opendisplay push
 * images to the ``zephyr,display`` device. Protocol 2.2 (opendisplay.org).
 */

/** Bytes of application data in the manufacturer-specific advertising record. */
#define OPENDISPLAY_MSD_DYNAMIC_LEN 11

/** Length of the manufacturer-specific data record, company id included. */
#define OPENDISPLAY_MSD_LEN 16

/** Live telemetry carried in the advertising record and READ_MSD. */
struct opendisplay_status {
	/** Application bytes (button state and sensor values per the host's config). */
	uint8_t dynamic[OPENDISPLAY_MSD_DYNAMIC_LEN];
	/** Chip temperature, -40..87 degrees Celsius, 0.5 degree steps are dropped. */
	int8_t temperature_c;
	/** Battery voltage in millivolts (10 mV resolution, at most 5110), 0 if unknown. */
	uint16_t battery_mv;
};

/**
 * @brief Update the telemetry advertised to hosts.
 *
 * Also bumps the 4-bit loop counter so hosts see a fresh record.
 *
 * @param status New values.
 */
void opendisplay_set_status(const struct opendisplay_status *status);

/**
 * @brief Get the 16-byte manufacturer-specific data record.
 *
 * @param msd Output, OPENDISPLAY_MSD_LEN bytes: company id 0x2446 (LE) then the record.
 */
void opendisplay_get_msd(uint8_t msd[OPENDISPLAY_MSD_LEN]);

/** Image upload notifications. Called on the transport thread. */
struct opendisplay_callbacks {
	/** A complete image was written to the display; the refresh follows. */
	void (*image_received)(void);
	/**
	 * The panel refresh finished.
	 *
	 * @param err 0, or the negative errno display_blanking_off() returned.
	 */
	void (*refresh_done)(int err);
};

/**
 * @brief Register upload callbacks.
 *
 * @param cb Callbacks, kept by reference; NULL removes them.
 */
void opendisplay_set_callbacks(const struct opendisplay_callbacks *cb);

/**
 * @brief Start advertising the OpenDisplay GATT service.
 *
 * Call after bt_enable() (and settings_load() when Bluetooth settings are
 * used). Advertising restarts by itself after each disconnect. Needs
 * CONFIG_OPENDISPLAY_BLE.
 *
 * @retval 0 Advertising started.
 * @retval -EAGAIN Bluetooth is not enabled yet.
 * @retval -errno Other negative errno from bt_le_adv_start().
 */
int opendisplay_ble_start(void);

/** @} */

#endif /* ZEPHYR_INCLUDE_OPENDISPLAY_OPENDISPLAY_H_ */

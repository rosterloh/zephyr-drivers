#ifndef INCLUDE_DRIVERS_SEESAW_H_
#define INCLUDE_DRIVERS_SEESAW_H_

/**
 * @brief Seesaw Interface
 * @defgroup seesaw_interface Seesaw Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

enum {
	SEESAW_INPUT = 0x0,
	SEESAW_OUTPUT = 0x1,
	SEESAW_INPUT_PULLUP = 0x2,
	SEESAW_INPUT_PULLDOWN = 0x3,
};

#define NEOKEY_1X4_NEOPIN  3
#define NEOKEY_1X4_BUTTONA 4
#define NEOKEY_1X4_BUTTONB 5
#define NEOKEY_1X4_BUTTONC 6
#define NEOKEY_1X4_BUTTOND 7
#define NEOKEY_1X4_BUTTONMASK                                                                      \
	(BIT(NEOKEY_1X4_BUTTONA) | BIT(NEOKEY_1X4_BUTTONB) | BIT(NEOKEY_1X4_BUTTONC) |             \
	 BIT(NEOKEY_1X4_BUTTOND))

#define NEOKEY_1X4_ROWS 1
#define NEOKEY_1X4_COLS 4
#define NEOKEY_1X4_KEYS (NEOKEY_1X4_ROWS * NEOKEY_1X4_COLS)

// RGB NeoPixel permutations; white and red offsets are always same
// Offset:         W          R          G          B
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBG ((0 << 6) | (0 << 4) | (2 << 2) | (1))
#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBR ((2 << 6) | (2 << 4) | (0 << 2) | (1))
#define NEO_BRG ((1 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGR ((2 << 6) | (2 << 4) | (1 << 2) | (0))

// RGBW NeoPixel permutations; all 4 offsets are distinct
// Offset:         W          R          G          B
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3))
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2))
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3))
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2))
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1))
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1))

#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3))
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2))
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3))
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1))
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1))

#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3))
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2))
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3))
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1))
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1))

#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0))
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0))
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0))
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0))
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0))

// If 400 KHz support is enabled, the third parameter to the constructor
// requires a 16-bit value (in order to select 400 vs 800 KHz speed).
// If only 800 KHz is enabled (as is default on ATtiny), an 8-bit value
// is sufficient to encode pixel color order, saving some space.

#define NEO_KHZ800 0x0000 // 800 KHz datastream
#define NEO_KHZ400 0x0100 // 400 KHz datastream

/**
 * @typedef seesaw_write_pin_mode_t
 * @brief Callback API to configure seesaw gpio.
 *
 * See seesaw_write_pin_mode() for argument description
 */
typedef int (*seesaw_write_pin_mode_t)(const struct device *dev, uint32_t pins, uint8_t mode);

/**
 * @typedef seesaw_read_digital_t
 * @brief Callback API for reading seesaw gpio pins.
 *
 * See seesaw_read_digital() for argument description
 */
typedef int (*seesaw_read_digital_t)(const struct device *dev, uint32_t pins, uint32_t *val);

/**
 * @typedef seesaw_read_analog_t
 * @brief Callback API for reading an seesaw analog pin.
 *
 * See seesaw_read_analog() for argument description
 */
typedef int (*seesaw_read_analog_t)(const struct device *dev, uint8_t pin, uint16_t *val);

/**
 * @typedef seesaw_interrupts_gpio_t
 * @brief Callback API to configure seesaw gpio interrupts.
 *
 * See seesaw_gpio_interrupts() for argument description
 */
typedef int (*seesaw_interrupts_gpio_t)(const struct device *dev, uint32_t pins, uint8_t enable);

/**
 * @typedef seesaw_int_callback_t
 * @brief Define the callback function for interrupts
 *
 * @param "struct device *dev" Pointer to the seesaw device
 */
typedef void (*seesaw_int_callback_t)(const struct device *dev);

/**
 * @typedef seesaw_int_set_t
 * @brief Callback API for setting a seesaw's interrupt handler
 *
 * See seesaw_int_set() for argument description
 */
typedef int (*seesaw_int_set_t)(const struct device *dev, seesaw_int_callback_t handler);

/**
 * @typedef seesaw_neopixel_setup_t
 * @brief Callback API for setting up neopixels attached to seesaw device.
 *
 * See seesaw_neopixel_setup() for argument description
 */
typedef int (*seesaw_neopixel_setup_t)(const struct device *dev, uint16_t type, uint16_t length,
				       uint8_t pin);

/**
 * @typedef seesaw_neopixel_set_colour_t
 * @brief Callback API for setting the colour of a neopixels attached to seesaw device.
 *
 * See seesaw_neopixel_set_colour() for argument description
 */
typedef int (*seesaw_neopixel_set_colour_t)(const struct device *dev, uint8_t n, uint8_t r,
					    uint8_t g, uint8_t b, uint8_t w);

/**
 * @typedef seesaw_neopixel_set_brightness_t
 * @brief Callback API for setting the brightness of a neopixels attached to seesaw device.
 *
 * See seesaw_neopixel_set_brightness() for argument description
 */
typedef int (*seesaw_neopixel_set_brightness_t)(const struct device *dev, uint8_t brightness);

/**
 * @typedef seesaw_neopixel_show_t
 * @brief Callback API for showing the current values on neopixels attached to seesaw device.
 *
 * See seesaw_neopixel_show() for argument description
 */
typedef int (*seesaw_neopixel_show_t)(const struct device *dev);

/** @brief Seesaw driver class operations */
__subsystem struct seesaw_driver_api {
	seesaw_write_pin_mode_t write_pin_mode;
	seesaw_read_digital_t read_digital;
	seesaw_read_analog_t read_analog;
	seesaw_interrupts_gpio_t gpio_interrupts;
	seesaw_int_set_t int_set;
	seesaw_neopixel_setup_t neopixel_setup;
	seesaw_neopixel_set_colour_t neopixel_set_colour;
	seesaw_neopixel_set_brightness_t neopixel_set_brightness;
	seesaw_neopixel_show_t neopixel_show;
};

/**
 * @brief Configures the gpio of a seesaw device.
 *
 * @param dev Pointer to the seesaw device.
 * @param pins Button mask of pins to configure.
 * @param mode Mode to configure pins as.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_write_pin_mode(const struct device *dev, uint32_t pins, uint8_t mode);

static inline int z_impl_seesaw_write_pin_mode(const struct device *dev, uint32_t pins,
					       uint8_t mode)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->write_pin_mode == NULL) {
		return -ENOSYS;
	}

	return api->write_pin_mode(dev, pins, mode);
}

/**
 * @brief Reads the state of the gpios of a seesaw device.
 *
 * @param dev Pointer to the seesaw device.
 * @param pins Button mask of pins to read.
 * @param val Pointer to where values are read into
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_read_digital(const struct device *dev, uint32_t pins, uint32_t *val);

static inline int z_impl_seesaw_read_digital(const struct device *dev, uint32_t pins, uint32_t *val)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->read_digital == NULL) {
		return -ENOSYS;
	}

	return api->read_digital(dev, pins, val);
}

/**
 * @brief Reads the value of a seesaw analog pin.
 *
 * @param dev Pointer to the seesaw device.
 * @param pin Analog pin to read.
 * @param val Pointer to where value is read into
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_read_analog(const struct device *dev, uint8_t pin, uint16_t *val);

static inline int z_impl_seesaw_read_analog(const struct device *dev, uint8_t pin, uint16_t *val)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->read_analog == NULL) {
		return -ENOSYS;
	}

	return api->read_analog(dev, pin, val);
}

/**
 * @brief Configures the gpio interrupts of a seesaw device.
 *
 * @param dev Pointer to the seesaw device.
 * @param pins Button mask of pins to configure.
 * @param enabled 0 to disable or 1 to enable interrupts.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_gpio_interrupts(const struct device *dev, uint32_t pins, uint8_t enabled);

static inline int z_impl_seesaw_gpio_interrupts(const struct device *dev, uint32_t pins,
						uint8_t enabled)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->gpio_interrupts == NULL) {
		return -ENOSYS;
	}

	return api->gpio_interrupts(dev, pins, enabled);
}

/**
 * @brief Set the INT callback function pointer.
 *
 * This sets up the callback for INT. When an IRQ is triggered,
 * the specified function will be called with the device pointer.
 *
 * @param dev Seesaw device structure.
 * @param cb Pointer to the callback function.
 *
 * @return N/A
 */
__syscall void seesaw_int_set(const struct device *dev, seesaw_int_callback_t cb);

static inline void z_impl_seesaw_int_set(const struct device *dev, seesaw_int_callback_t cb)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if ((api != NULL) && (api->int_set != NULL)) {
		api->int_set(dev, cb);
	}
}

/**
 * @brief Sets up a length of neopixels of a type attached to a pin.
 *
 * @param dev Pointer to the seesaw device.
 * @param type Type of neopixels attached to this pin.
 * @param length Length of neopixels in this string.
 * @param pin Data pin to write out on.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_neopixel_setup(const struct device *dev, uint16_t type, uint16_t length,
				    uint8_t pin);

static inline int z_impl_seesaw_neopixel_setup(const struct device *dev, uint16_t type,
					       uint16_t length, uint8_t pin)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->neopixel_setup == NULL) {
		return -ENOSYS;
	}

	return api->neopixel_setup(dev, type, length, pin);
}

/**
 * @brief Set the colour on a particular neopixel.
 *
 * @param dev Pointer to the seesaw device.
 * @param n Index of the pixel to set.
 * @param r Red value.
 * @param g Green value.
 * @param b Blue value.
 * @param w White value.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_neopixel_set_colour(const struct device *dev, uint8_t n, uint8_t r, uint8_t g,
					 uint8_t b, uint8_t w);

static inline int z_impl_seesaw_neopixel_set_colour(const struct device *dev, uint8_t n, uint8_t r,
						    uint8_t g, uint8_t b, uint8_t w)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->neopixel_set_colour == NULL) {
		return -ENOSYS;
	}

	return api->neopixel_set_colour(dev, n, r, g, b, w);
}

/**
 * @brief Set the brightness of connected neopixels.
 *
 * @param dev Pointer to the seesaw device.
 * @param brightness Brightness value.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_neopixel_set_brightness(const struct device *dev, uint8_t brightness);

static inline int z_impl_seesaw_neopixel_set_brightness(const struct device *dev,
							uint8_t brightness)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->neopixel_set_brightness == NULL) {
		return -ENOSYS;
	}

	return api->neopixel_set_brightness(dev, brightness);
}

/**
 * @brief Show the current colour values on connected neopixels.
 *
 * @param dev Pointer to the seesaw device.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_neopixel_show(const struct device *dev);

static inline int z_impl_seesaw_neopixel_show(const struct device *dev)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->neopixel_show == NULL) {
		return -ENOSYS;
	}

	return api->neopixel_show(dev);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include <syscalls/seesaw.h>

#endif /* INCLUDE_DRIVERS_SEESAW_H_ */
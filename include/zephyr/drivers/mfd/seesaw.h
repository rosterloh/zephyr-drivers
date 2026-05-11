/*
 * Public API for the Adafruit Seesaw MFD parent. Consumed by Seesaw subsystem
 * child drivers (gpio/adc/pwm/led_strip/encoder). Not for application use.
 */
#ifndef ZEPHYR_DRIVERS_MFD_SEESAW_H_
#define ZEPHYR_DRIVERS_MFD_SEESAW_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mfd_seesaw_int_cb_t)(const struct device *mfd, void *user_data);

struct mfd_seesaw_int_callback {
	sys_snode_t node;
	mfd_seesaw_int_cb_t handler;
	void *user_data;
};

int mfd_seesaw_read(const struct device *mfd, uint8_t mod, uint8_t reg, uint8_t *buf, size_t len,
		    uint32_t pre_read_delay_us);

int mfd_seesaw_write(const struct device *mfd, uint8_t mod, uint8_t reg, const uint8_t *buf,
		     size_t len);

uint32_t mfd_seesaw_options(const struct device *mfd);
uint8_t mfd_seesaw_hw_id(const struct device *mfd);
uint16_t mfd_seesaw_pid(const struct device *mfd);

int mfd_seesaw_add_int_callback(const struct device *mfd, struct mfd_seesaw_int_callback *cb);

int mfd_seesaw_remove_int_callback(const struct device *mfd, struct mfd_seesaw_int_callback *cb);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_MFD_SEESAW_H_ */

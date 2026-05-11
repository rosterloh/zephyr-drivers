#ifdef CONFIG_GPIO_ADAFRUIT_SEESAW

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/ztest.h>

#include "seesaw_emul.h"

#define GPIO_NODE DT_NODELABEL(gpio_child)

static const struct device *const gpio = DEVICE_DT_GET(GPIO_NODE);

ZTEST(gpio_seesaw, test_ready)
{
	zassert_true(device_is_ready(gpio));
}

ZTEST(gpio_seesaw, test_configure_output)
{
	zassert_ok(gpio_pin_configure(gpio, 4, GPIO_OUTPUT_INACTIVE));
}

ZTEST(gpio_seesaw, test_set_clear_propagates_to_emul)
{
	const struct emul *target = EMUL_DT_GET(DT_PARENT(GPIO_NODE));
	uint32_t reg_val = 0;

	zassert_not_null(target);
	zassert_ok(gpio_pin_configure(gpio, 4, GPIO_OUTPUT_INACTIVE));

	/* Set pin high → writes BIT(4) to BULK_SET. */
	zassert_ok(gpio_pin_set(gpio, 4, 1));
	zassert_ok(mfd_seesaw_mock_get_register(
		target->data, (0x01 << 8) | 0x05 /* GPIO_BASE,BULK_SET */, &reg_val));
	zassert_equal(reg_val, BIT(4), "BULK_SET register did not contain BIT(4); got 0x%08x",
		      reg_val);

	/* Clear pin → writes BIT(4) to BULK_CLR. */
	zassert_ok(gpio_pin_set(gpio, 4, 0));
	zassert_ok(mfd_seesaw_mock_get_register(
		target->data, (0x01 << 8) | 0x06 /* GPIO_BASE,BULK_CLR */, &reg_val));
	zassert_equal(reg_val, BIT(4), "BULK_CLR register did not contain BIT(4); got 0x%08x",
		      reg_val);
}

ZTEST_SUITE(gpio_seesaw, NULL, NULL, NULL, NULL, NULL);

#endif /* CONFIG_GPIO_ADAFRUIT_SEESAW */

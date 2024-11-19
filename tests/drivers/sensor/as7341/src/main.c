#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/fff.h>
#include <zephyr/ztest.h>

#define NODE DT_NODELABEL(as7341)

DEFINE_FFF_GLOBALS;

struct as7341_fixture {
	const struct device *dev;
	const struct emul *mock;
};

static void *as7341_setup(void)
{
	static struct as7341_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(as7341)),
		.mock = EMUL_DT_GET(DT_NODELABEL(as7341)),
	};

	zassert_not_null(fixture.dev);
	zassert_not_null(fixture.mock);
	return &fixture;
}

ZTEST_SUITE(as7341, NULL, as7341_setup, NULL, NULL, NULL);
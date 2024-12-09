#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include <as7341_emul.h>
#include <as7341.h>

struct as7341_fixture {
	const struct device *dev;
	const struct emul *target;
};

static void *as7341_setup(void)
{
	static struct as7341_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(as7341)),
		.target = EMUL_DT_GET(DT_NODELABEL(as7341)),
	};

	zassert_not_null(fixture.dev);
	zassert_not_null(fixture.target);
	return &fixture;
}

static void as7341_before(void *f)
{
	struct as7341_fixture *fixture = f;

	as7341_emul_reset(fixture->target);
}

/**
 * @brief Verify default configuration is correct.
 */
ZTEST(as7341, test_default_config)
{
	const struct as7341_data *data;
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(as7341));

	zassert_not_null(dev);
	data = dev->data;

	/* confirm default settings */
	zexpect_equal(256, data->again);
	zexpect_equal(29, data->atime);
	zexpect_equal(599, data->astep);
	zexpect_equal(50, data->integration_ms);
}

ZTEST_SUITE(as7341, NULL, as7341_setup, as7341_before, NULL, NULL);
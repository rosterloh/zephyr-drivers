#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/fff.h>
#include <zephyr/ztest.h>

#include <seesaw.h>
#include <seesaw_emul.h>

#define NODE DT_NODELABEL(seesaw)

DEFINE_FFF_GLOBALS;

struct seesaw_fixture {
	const struct device *dev;
	const struct emul *target;
};

static void *seesaw_setup(void)
{
	static struct seesaw_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(seesaw)),
		.target = EMUL_DT_GET(DT_NODELABEL(seesaw)),
	};

	zassert_not_null(fixture.dev);
	zassert_not_null(fixture.target);
	return &fixture;
}

ZTEST_SUITE(seesaw, NULL, seesaw_setup, NULL, NULL, NULL);

ZTEST_F(seesaw, test_default_config)
{
	uint32_t options = 0;
	uint16_t reg = SEESAW_STATUS_BASE << 8 | SEESAW_STATUS_OPTIONS;

	zassert_ok(seesaw_mock_get_register(fixture->target->data, reg, &options));
	zassert_equal(0xFFF07, options);
}

ZTEST_F(seesaw, test_setup_gpios)
{
	uint32_t data = 0;
	uint16_t reg = SEESAW_GPIO_BASE << 8 | SEESAW_GPIO_DIRCLR_BULK;

	zassert_ok(seesaw_write_pin_mode(fixture->dev, NEOKEY_1X4_BUTTONMASK, SEESAW_INPUT_PULLUP));

	// All the registers we expect to set are correct
	zassert_ok(seesaw_mock_get_register(fixture->target->data, reg, &data));
	zassert_equal(NEOKEY_1X4_BUTTONMASK, data);

	reg = SEESAW_GPIO_BASE << 8 | SEESAW_GPIO_PULLENSET;
	zassert_ok(seesaw_mock_get_register(fixture->target->data, reg, &data));
	zassert_equal(NEOKEY_1X4_BUTTONMASK, data);

	reg = SEESAW_GPIO_BASE << 8 | SEESAW_GPIO_BULK_SET;
	zassert_ok(seesaw_mock_get_register(fixture->target->data, reg, &data));
	zassert_equal(NEOKEY_1X4_BUTTONMASK, data);

	// Check that another register we did not want to set remains in it's default state
	reg = SEESAW_GPIO_BASE << 8 | SEESAW_GPIO_DIRSET_BULK;
	zassert_ok(seesaw_mock_get_register(fixture->target->data, reg, &data));
	zassert_equal(0, data);
}

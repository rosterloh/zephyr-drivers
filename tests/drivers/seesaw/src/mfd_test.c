#include <zephyr/device.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/ztest.h>

#include "mfd_seesaw_regs.h"

#define MFD_NODE DT_NODELABEL(mfd_seesaw)

static const struct device *const mfd = DEVICE_DT_GET(MFD_NODE);

ZTEST(mfd_seesaw, test_device_ready)
{
	zassert_true(device_is_ready(mfd), "MFD parent not ready");
}

ZTEST(mfd_seesaw, test_hw_id_cached)
{
	/* Legacy emul initialises STATUS_HW_ID = 0x55 (SAMD09). Probe in init()
	 * caches it. */
	zassert_equal(mfd_seesaw_hw_id(mfd), SEESAW_HW_ID_CODE_SAMD09,
		      "expected cached HW ID = 0x%02x, got 0x%02x", SEESAW_HW_ID_CODE_SAMD09,
		      mfd_seesaw_hw_id(mfd));
}

ZTEST(mfd_seesaw, test_options_cached)
{
	uint32_t opts = mfd_seesaw_options(mfd);

	zassert_not_equal(opts, 0, "options bitmap should be non-zero (defaulted by emul)");
}

ZTEST(mfd_seesaw, test_register_write_read_roundtrip)
{
	uint8_t wbuf[2] = {0xAB, 0xCD};
	uint8_t rbuf[2] = {0};

	zassert_ok(
		mfd_seesaw_write(mfd, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, wbuf, sizeof(wbuf)),
		"write failed");
	zassert_ok(
		mfd_seesaw_read(mfd, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, rbuf, sizeof(rbuf), 0),
		"read failed");
	zassert_mem_equal(rbuf, wbuf, sizeof(wbuf), "roundtrip mismatch");
}

ZTEST_SUITE(mfd_seesaw, NULL, NULL, NULL, NULL, NULL);

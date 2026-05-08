#include <zephyr/fff.h>
#include <zephyr/ztest.h>

DEFINE_FFF_GLOBALS;

ZTEST_SUITE(hbridge_build, NULL, NULL, NULL, NULL, NULL);
ZTEST(hbridge_build, test_smoke)
{
	ztest_test_skip();
}

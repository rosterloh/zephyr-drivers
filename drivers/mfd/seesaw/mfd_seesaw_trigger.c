/*
 * Temporary stub. Real implementation lands in Task 3 (INT-line fan-out).
 */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mfd/seesaw.h>

int mfd_seesaw_init_interrupt(const struct device *mfd)
{
	(void)mfd;
	return 0;
}

int mfd_seesaw_add_int_callback(const struct device *mfd, struct mfd_seesaw_int_callback *cb)
{
	(void)mfd;
	(void)cb;
	return -ENOSYS;
}

int mfd_seesaw_remove_int_callback(const struct device *mfd, struct mfd_seesaw_int_callback *cb)
{
	(void)mfd;
	(void)cb;
	return -ENOSYS;
}

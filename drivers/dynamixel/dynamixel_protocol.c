#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "dynamixel_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(dynamixel, CONFIG_DYNAMIXEL_LOG_LEVEL);

int dxl_parse_status_payload(const uint8_t *data, uint8_t width, uint32_t *out)
{
	uint8_t dev_err = data[0];

	if (dev_err != 0) {
		return (int)dev_err;
	}
	switch (width) {
	case 1:
		*out = data[1];
		return 0;
	case 2:
		*out = sys_get_le16(&data[1]);
		return 0;
	case 4:
		*out = sys_get_le32(&data[1]);
		return 0;
	default:
		return -EINVAL;
	}
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#ping-0x01 */
int dxl_ping(const int iface, const uint8_t id)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->expected_id = id;
	ctx->tx_frame.length = 3;
	ctx->tx_frame.ic = DXL_INST_PING;

	err = dxl_tx_wait_rx(ctx);

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#reboot-0x08 */
int dxl_reboot(const int iface, const uint8_t id)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->expected_id = id;
	ctx->tx_frame.length = 3;
	ctx->tx_frame.ic = DXL_INST_REBOOT;

	err = dxl_tx_wait_rx(ctx);

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

static int dxl_read_n(int iface, uint8_t id, enum dxl_control item, uint8_t expected_width,
		      uint32_t *out)
{
	if (iface < 0) {
		return -EINVAL;
	}
	struct dxl_context *ctx = dxl_get_context((uint8_t)iface);
	uint16_t addr;
	uint8_t length;
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (out == NULL) {
		return -EINVAL;
	}
	if (dxl_table_lookup(item, &addr, &length) != 0) {
		return -EINVAL;
	}
	if (length != expected_width) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->expected_id = id;
	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 7;
	ctx->tx_frame.ic = DXL_INST_READ;
	sys_put_le16(addr, &ctx->tx_frame.data[0]);
	sys_put_le16(length, &ctx->tx_frame.data[2]);

	err = dxl_tx_wait_rx(ctx);
	if (err == 0) {
		err = dxl_parse_status_payload(ctx->rx_frame.data, length, out);
	}

	k_mutex_unlock(&ctx->iface_lock);
	return err;
}

int dxl_read_u8(int iface, uint8_t id, enum dxl_control item, uint8_t *out)
{
	uint32_t v = 0;
	int rc = dxl_read_n(iface, id, item, 1, &v);
	if (rc == 0) {
		*out = (uint8_t)v;
	}
	return rc;
}

int dxl_read_u16(int iface, uint8_t id, enum dxl_control item, uint16_t *out)
{
	uint32_t v = 0;
	int rc = dxl_read_n(iface, id, item, 2, &v);
	if (rc == 0) {
		*out = (uint16_t)v;
	}
	return rc;
}

int dxl_read_u32(int iface, uint8_t id, enum dxl_control item, uint32_t *out)
{
	return dxl_read_n(iface, id, item, 4, out);
}

static int dxl_write_n(int iface, uint8_t id, enum dxl_control item, uint8_t expected_width,
		       uint32_t value)
{
	if (iface < 0) {
		return -EINVAL;
	}
	struct dxl_context *ctx = dxl_get_context((uint8_t)iface);
	uint16_t addr;
	uint8_t length;
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (dxl_table_lookup(item, &addr, &length) != 0) {
		return -EINVAL;
	}
	if (length != expected_width) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->expected_id = id;
	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 5 + length;
	ctx->tx_frame.ic = DXL_INST_WRITE;
	sys_put_le16(addr, &ctx->tx_frame.data[0]);
	switch (length) {
	case 1:
		ctx->tx_frame.data[2] = (uint8_t)value;
		break;
	case 2:
		sys_put_le16((uint16_t)value, &ctx->tx_frame.data[2]);
		break;
	case 4:
		sys_put_le32(value, &ctx->tx_frame.data[2]);
		break;
	default:
		k_mutex_unlock(&ctx->iface_lock);
		return -EINVAL;
	}

	err = dxl_tx_wait_rx(ctx);
	if (err == 0) {
		uint8_t dev_err = ctx->rx_frame.data[0];
		err = (dev_err == 0) ? 0 : (int)dev_err;
	}

	k_mutex_unlock(&ctx->iface_lock);
	return err;
}

int dxl_write_u8(int iface, uint8_t id, enum dxl_control item, uint8_t val)
{
	return dxl_write_n(iface, id, item, 1, val);
}

int dxl_write_u16(int iface, uint8_t id, enum dxl_control item, uint16_t val)
{
	return dxl_write_n(iface, id, item, 2, val);
}

int dxl_write_u32(int iface, uint8_t id, enum dxl_control item, uint32_t val)
{
	return dxl_write_n(iface, id, item, 4, val);
}

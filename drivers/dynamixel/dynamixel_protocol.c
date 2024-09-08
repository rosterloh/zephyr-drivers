#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "dynamixel_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(dynamixel, CONFIG_DYNAMIXEL_LOG_LEVEL);

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#ping-0x01 */
int dxl_ping(const int iface, const uint8_t id)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	int err;

	if (ctx == NULL)
	{
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 3; /* instruction + 2 crc */
	ctx->tx_frame.ic = DXL_INST_PING;

	err = dxl_tx_wait_rx(ctx);
	/* validate response */

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#read-0x02 */
int dxl_read(const int iface, const uint8_t id, uint8_t item_idx, void *data)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	struct dxl_control_info control = control_table[item_idx];
	int err;

	if (ctx == NULL)
	{
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 7; /* instruction + 4 parameters + 2 crc */
	ctx->tx_frame.ic = DXL_INST_READ;
	sys_put_le16(control.address, &ctx->tx_frame.data[0]);
	sys_put_le16(control.length, &ctx->tx_frame.data[2]);

	err = dxl_tx_wait_rx(ctx);

	/* https://emanual.robotis.com/docs/en/dxl/protocol2/#error */
	err = ctx->rx_frame.data[0];
	if (control.length == 1)
	{
		*(uint8_t *)data = ctx->rx_frame.data[1];
	}
	else if (control.length == 2)
	{
		*(uint16_t *)data = sys_get_le16(&ctx->rx_frame.data[1]);
	}
	else
	{
		*(uint32_t *)data = sys_get_le16(&ctx->rx_frame.data[1]);
	}

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#write-0x03 */
int dxl_write(const int iface, const uint8_t id, uint8_t item_idx, uint32_t data)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	struct dxl_control_info control = control_table[item_idx];
	int err;

	if (ctx == NULL)
	{
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 5 + control.length; /* instruction + 2 address + parameter data + 2 crc */
	ctx->tx_frame.ic = DXL_INST_WRITE;
	sys_put_le16(control.address, &ctx->tx_frame.data[0]);
	if (control.length == 1)
	{
		sys_put_le32(data & 0xFF, &ctx->tx_frame.data[2]);
	}
	else if (control.length == 2)
	{
		sys_put_le32(data & 0xFFFF, &ctx->tx_frame.data[2]);
	}
	else
	{
		sys_put_le32(data, &ctx->tx_frame.data[2]);
	}

	err = dxl_tx_wait_rx(ctx);

	/* https://emanual.robotis.com/docs/en/dxl/protocol2/#error */
	err = ctx->rx_frame.data[0];

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#reboot-0x08 */
int dxl_reboot(const int iface, const uint8_t id)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	int err;

	if (ctx == NULL)
	{
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 3; /* instruction + 2 crc */
	ctx->tx_frame.ic = DXL_INST_REBOOT;

	err = dxl_tx_wait_rx(ctx);
	/* validate response */

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}
#define DT_DRV_COMPAT waveshare_bus_servo

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "bus_servo_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bus_servo, CONFIG_BUS_SERVO_LOG_LEVEL);

#define BUS_SERVO_PACKET_OVERHEAD            6
#define BUS_SERVO_RESPONSE_LEN_ANY           SIZE_MAX
#define BUS_SERVO_POSITION_EX_LEN            7
#define BUS_SERVO_SYNC_POSITION_EX_ENTRY_LEN (1 + BUS_SERVO_POSITION_EX_LEN)

#define BUS_SERVO_DEFINE_GPIO_CFG(inst, prop)                                                      \
	static struct gpio_dt_spec prop##_cfg_##inst = {                                           \
		.port = DEVICE_DT_GET(DT_INST_PHANDLE(inst, prop)),                                \
		.pin = DT_INST_GPIO_PIN(inst, prop),                                               \
		.dt_flags = DT_INST_GPIO_FLAGS(inst, prop),                                        \
	};

#define BUS_SERVO_DEFINE_GPIO_CFGS(inst)                                                           \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, tx_en_gpios),                                      \
		    (BUS_SERVO_DEFINE_GPIO_CFG(inst, tx_en_gpios)), ())

DT_INST_FOREACH_STATUS_OKAY(BUS_SERVO_DEFINE_GPIO_CFGS)

#define BUS_SERVO_ASSIGN_GPIO_CFG(inst, prop)                                                      \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop), (&prop##_cfg_##inst), (NULL))

#define BUS_SERVO_DT_GET_SERIAL_DEV(inst)                                                          \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
		.tx_en = BUS_SERVO_ASSIGN_GPIO_CFG(inst, tx_en_gpios),                             \
	},

static struct bus_servo_serial_config bus_servo_serial_cfg[] = {
	DT_INST_FOREACH_STATUS_OKAY(BUS_SERVO_DT_GET_SERIAL_DEV)};

#define BUS_SERVO_DT_GET_DEV(inst)                                                                 \
	{                                                                                          \
		.iface_name = DEVICE_DT_NAME(DT_DRV_INST(inst)),                                   \
		.cfg = &bus_servo_serial_cfg[inst],                                                \
	},

static struct bus_servo_context bus_servo_ctx_tbl[] = {
	DT_INST_FOREACH_STATUS_OKAY(BUS_SERVO_DT_GET_DEV)};

uint8_t bus_servo_checksum(const uint8_t *data, size_t len)
{
	uint16_t sum = 0;

	for (size_t i = 0; i < len; i++) {
		sum += data[i];
	}

	return (uint8_t)~(sum & 0xff);
}

struct bus_servo_context *bus_servo_get_context(int iface)
{
	if (iface < 0 || iface >= ARRAY_SIZE(bus_servo_ctx_tbl)) {
		LOG_ERR("Interface %d not available", iface);
		return NULL;
	}

	if (!bus_servo_ctx_tbl[iface].configured) {
		LOG_ERR("Interface %d not configured", iface);
		return NULL;
	}

	return &bus_servo_ctx_tbl[iface];
}

int bus_servo_iface_get_by_name(const char *iface_name)
{
	if (iface_name == NULL) {
		return -EINVAL;
	}

	for (int i = 0; i < ARRAY_SIZE(bus_servo_ctx_tbl); i++) {
		if (strcmp(iface_name, bus_servo_ctx_tbl[i].iface_name) == 0) {
			return i;
		}
	}

	return -ENODEV;
}

int bus_servo_init(int iface, struct bus_servo_iface_param param)
{
	struct bus_servo_context *ctx;
	int rc;

	if (iface < 0 || iface >= ARRAY_SIZE(bus_servo_ctx_tbl)) {
		return -EINVAL;
	}

	ctx = &bus_servo_ctx_tbl[iface];
	if (ctx->configured) {
		return -EBUSY;
	}

	k_mutex_init(&ctx->iface_lock);
	k_sem_init(&ctx->wait_sem, 0, 1);
	ctx->rx_timeout_us = param.rx_timeout_us;
	ctx->expected_id = 0;
	ctx->status_error = 0;
	ctx->response_param_len = 0;

	rc = bus_servo_serial_init(ctx, param);
	if (rc != 0) {
		return rc;
	}

	ctx->configured = true;
	return 0;
}

int bus_servo_disable(int iface)
{
	struct bus_servo_context *ctx = bus_servo_get_context(iface);

	if (ctx == NULL) {
		return -EINVAL;
	}

	ctx->configured = false;
	ctx->rx_timeout_us = 0;
	ctx->status_error = 0;
	ctx->response_param_len = 0;

	return 0;
}

static int build_packet(uint8_t id, enum bus_servo_instruction instruction, const uint8_t *params,
			size_t param_len, uint8_t *packet, size_t *packet_len)
{
	size_t len = param_len + 2;

	if (packet == NULL || packet_len == NULL || len > UINT8_MAX ||
	    param_len + BUS_SERVO_PACKET_OVERHEAD > BUS_SERVO_MAX_PACKET_SIZE) {
		return -EINVAL;
	}

	packet[0] = 0xff;
	packet[1] = 0xff;
	packet[2] = id;
	packet[3] = (uint8_t)len;
	packet[4] = (uint8_t)instruction;
	if (param_len > 0) {
		if (params == NULL) {
			return -EINVAL;
		}
		memcpy(&packet[5], params, param_len);
	}
	packet[5 + param_len] = bus_servo_checksum(&packet[2], 3 + param_len);
	*packet_len = param_len + BUS_SERVO_PACKET_OVERHEAD;

	return 0;
}

static int transact(int iface, uint8_t id, enum bus_servo_instruction instruction,
		    const uint8_t *params, size_t param_len, bool expect_response,
		    size_t expected_param_len, uint8_t *response_out)
{
	struct bus_servo_context *ctx = bus_servo_get_context(iface);
	uint8_t packet[BUS_SERVO_MAX_PACKET_SIZE];
	size_t packet_len;
	int rc;

	if (ctx == NULL) {
		return -EINVAL;
	}

	rc = build_packet(id, instruction, params, param_len, packet, &packet_len);
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);
	ctx->expected_id = id;
	ctx->status_error = 0;
	ctx->response_param_len = 0;
	rc = bus_servo_tx_rx(ctx, packet, packet_len, expect_response);
	if (rc != 0) {
		goto unlock;
	}
	if (!expect_response && id != BUS_SERVO_BROADCAST_ID) {
		/* A unicast write is acknowledged with a status packet even when
		 * the caller does not read it. Drain that reply so it cannot
		 * collide with the next transaction on the shared bus (which
		 * silently dropped back-to-back single-servo writes). Broadcast
		 * sync-writes are not acknowledged, so they skip this.
		 */
		bus_servo_drain_reply(ctx);
	}
	if (expect_response && ctx->status_error != 0) {
		rc = -EIO;
		goto unlock;
	}
	if (expected_param_len != BUS_SERVO_RESPONSE_LEN_ANY &&
	    ctx->response_param_len != expected_param_len) {
		rc = -EBADMSG;
		goto unlock;
	}
	if (response_out != NULL && expected_param_len != BUS_SERVO_RESPONSE_LEN_ANY) {
		memcpy(response_out, ctx->response_params, expected_param_len);
	}

unlock:
	k_mutex_unlock(&ctx->iface_lock);

	return rc;
}

int bus_servo_ping(int iface, uint8_t id)
{
	return transact(iface, id, BUS_SERVO_INST_PING, NULL, 0, true, 0, NULL);
}

static int bus_servo_read(int iface, uint8_t id, uint8_t addr, uint8_t len, uint8_t *out)
{
	uint8_t params[] = {addr, len};

	if (out == NULL || len == 0) {
		return -EINVAL;
	}

	return transact(iface, id, BUS_SERVO_INST_READ, params, sizeof(params), true, len, out);
}

int bus_servo_read_u8(int iface, uint8_t id, uint8_t addr, uint8_t *out)
{
	return bus_servo_read(iface, id, addr, sizeof(*out), out);
}

int bus_servo_read_u16(int iface, uint8_t id, uint8_t addr, uint16_t *out)
{
	uint8_t data[sizeof(*out)];
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}

	rc = bus_servo_read(iface, id, addr, sizeof(data), data);
	if (rc != 0) {
		return rc;
	}

	*out = sys_get_le16(data);
	return 0;
}

static int bus_servo_write(int iface, uint8_t id, uint8_t addr, const uint8_t *data, size_t len)
{
	uint8_t params[BUS_SERVO_MAX_PACKET_SIZE];

	if (data == NULL || len == 0 || len + 1 > sizeof(params)) {
		return -EINVAL;
	}

	params[0] = addr;
	memcpy(&params[1], data, len);

	return transact(iface, id, BUS_SERVO_INST_WRITE, params, len + 1, false,
			BUS_SERVO_RESPONSE_LEN_ANY, NULL);
}

int bus_servo_write_u8(int iface, uint8_t id, uint8_t addr, uint8_t value)
{
	return bus_servo_write(iface, id, addr, &value, sizeof(value));
}

int bus_servo_write_u16(int iface, uint8_t id, uint8_t addr, uint16_t value)
{
	uint8_t data[sizeof(value)];

	sys_put_le16(value, data);
	return bus_servo_write(iface, id, addr, data, sizeof(data));
}

static void put_position_ex_params(uint8_t data[BUS_SERVO_POSITION_EX_LEN], uint16_t position,
				   uint16_t speed, uint8_t accel)
{
	data[0] = accel;
	sys_put_le16(position, &data[1]);
	sys_put_le16(0, &data[3]);
	sys_put_le16(speed, &data[5]);
}

int bus_servo_write_position_ex(int iface, uint8_t id, uint16_t position, uint16_t speed,
				uint8_t accel)
{
	uint8_t data[BUS_SERVO_POSITION_EX_LEN];

	put_position_ex_params(data, position, speed, accel);
	return bus_servo_write(iface, id, BUS_SERVO_REG_GOAL_ACCEL, data, sizeof(data));
}

int bus_servo_sync_write_position_ex(int iface, const uint8_t ids[], const uint16_t positions[],
				     const uint16_t speeds[], const uint8_t accels[], size_t n)
{
	const size_t max_entries = (BUS_SERVO_MAX_PACKET_SIZE - BUS_SERVO_PACKET_OVERHEAD - 2) /
				   BUS_SERVO_SYNC_POSITION_EX_ENTRY_LEN;
	uint8_t params[BUS_SERVO_MAX_PACKET_SIZE - BUS_SERVO_PACKET_OVERHEAD];
	size_t offset = 2;

	if (ids == NULL || positions == NULL || speeds == NULL || accels == NULL || n == 0) {
		return -EINVAL;
	}
	if (n > max_entries) {
		return -EMSGSIZE;
	}

	params[0] = BUS_SERVO_REG_GOAL_ACCEL;
	params[1] = BUS_SERVO_POSITION_EX_LEN;
	for (size_t i = 0; i < n; i++) {
		params[offset] = ids[i];
		put_position_ex_params(&params[offset + 1], positions[i], speeds[i], accels[i]);
		offset += BUS_SERVO_SYNC_POSITION_EX_ENTRY_LEN;
	}

	return transact(iface, BUS_SERVO_BROADCAST_ID, BUS_SERVO_INST_SYNC_WRITE, params, offset,
			false, BUS_SERVO_RESPONSE_LEN_ANY, NULL);
}

#define BUS_SERVO_DEVICE_DEFINE(inst)                                                              \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, NULL, NULL, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(BUS_SERVO_DEVICE_DEFINE)

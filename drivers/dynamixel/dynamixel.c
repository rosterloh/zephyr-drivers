#define DT_DRV_COMPAT robotis_dynamixel

#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "dynamixel_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dynamixel, CONFIG_DYNAMIXEL_LOG_LEVEL);

#define DXL_DEFINE_GPIO_CFG(inst, prop)                                                            \
	static struct gpio_dt_spec prop##_cfg_##inst = {                                           \
		.port = DEVICE_DT_GET(DT_INST_PHANDLE(inst, prop)),                                \
		.pin = DT_INST_GPIO_PIN(inst, prop),                                               \
		.dt_flags = DT_INST_GPIO_FLAGS(inst, prop),                                        \
	};

#define DXL_DEFINE_GPIO_CFGS(inst)                                                                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, tx_en_gpios),                                      \
		    (DXL_DEFINE_GPIO_CFG(inst, tx_en_gpios)), ())

DT_INST_FOREACH_STATUS_OKAY(DXL_DEFINE_GPIO_CFGS)

#define DXL_ASSIGN_GPIO_CFG(inst, prop)                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop), (&prop##_cfg_##inst), (NULL))

#define DXL_DT_GET_SERIAL_DEV(inst)                                                                \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
		.tx_en = DXL_ASSIGN_GPIO_CFG(inst, tx_en_gpios),                                   \
	},

static struct dxl_serial_config dxl_serial_cfg[] = {
	DT_INST_FOREACH_STATUS_OKAY(DXL_DT_GET_SERIAL_DEV)};

#define DXL_DT_GET_DEV(inst)                                                                       \
	{                                                                                          \
		.iface_name = DEVICE_DT_NAME(DT_DRV_INST(inst)),                                   \
		.cfg = &dxl_serial_cfg[inst],                                                      \
	},

static struct dxl_context dxl_ctx_tbl[] = {DT_INST_FOREACH_STATUS_OKAY(DXL_DT_GET_DEV)};

static void dxl_rx_handler(struct k_work *item)
{
	struct dxl_context *ctx;

	ctx = CONTAINER_OF(item, struct dxl_context, handler_work);

	dxl_serial_rx_disable(ctx);
	ctx->rx_frame_err = dxl_serial_rx(ctx);

	k_sem_give(&ctx->wait_sem);
}

void dxl_tx(struct dxl_context *ctx)
{
	dxl_serial_tx(ctx);
}

int dxl_tx_wait_rx(struct dxl_context *ctx)
{
	dxl_tx(ctx);

	if (k_sem_take(&ctx->wait_sem, K_USEC(ctx->rxwait_to)) != 0) {
		LOG_WRN("Packet wait-for-RX timeout");
		return -ETIMEDOUT;
	}

	return ctx->rx_frame_err;
}

struct dxl_context *dxl_get_context(const uint8_t iface)
{
	struct dxl_context *ctx;

	if (iface >= ARRAY_SIZE(dxl_ctx_tbl)) {
		LOG_ERR("Interface %u not available", iface);
		return NULL;
	}

	ctx = &dxl_ctx_tbl[iface];

	if (!atomic_test_bit(&ctx->state, DXL_STATE_CONFIGURED)) {
		LOG_ERR("Interface not configured");
		return NULL;
	}

	return ctx;
}

int dxl_iface_get_by_ctx(const struct dxl_context *ctx)
{
	for (int i = 0; i < ARRAY_SIZE(dxl_ctx_tbl); i++) {
		if (&dxl_ctx_tbl[i] == ctx) {
			return i;
		}
	}

	return -ENODEV;
}

int dxl_iface_get_by_name(const char *iface_name)
{
	for (int i = 0; i < ARRAY_SIZE(dxl_ctx_tbl); i++) {
		if (strcmp(iface_name, dxl_ctx_tbl[i].iface_name) == 0) {
			return i;
		}
	}

	return -ENODEV;
}

static struct dxl_context *dxl_init_iface(const uint8_t iface)
{
	struct dxl_context *ctx;

	if (iface >= ARRAY_SIZE(dxl_ctx_tbl)) {
		LOG_ERR("Interface %u not available", iface);
		return NULL;
	}

	ctx = &dxl_ctx_tbl[iface];

	if (atomic_test_and_set_bit(&ctx->state, DXL_STATE_CONFIGURED)) {
		LOG_ERR("Interface already used");
		return NULL;
	}

	k_mutex_init(&ctx->iface_lock);
	k_sem_init(&ctx->wait_sem, 0, 1);
	k_work_init(&ctx->handler_work, dxl_rx_handler);

	return ctx;
}

int dxl_init(const int iface, struct dxl_iface_param param)
{
	struct dxl_context *ctx = NULL;
	int rc = 0;

	ctx = dxl_init_iface(iface);
	if (ctx == NULL) {
		rc = -EINVAL;
		goto init_error;
	}

	if (dxl_serial_init(ctx, param) != 0) {
		LOG_ERR("Failed to init DYNAMIXEL over serial line");
		rc = -EINVAL;
		goto init_error;
	}

	ctx->rxwait_to = param.rx_timeout;

	return 0;

init_error:
	if (ctx != NULL) {
		atomic_clear_bit(&ctx->state, DXL_STATE_CONFIGURED);
	}

	return rc;
}

int dxl_disable(const uint8_t iface)
{
	struct dxl_context *ctx;
	struct k_work_sync work_sync;

	ctx = dxl_get_context(iface);
	if (ctx == NULL) {
		LOG_ERR("Interface %u not initialised", iface);
		return -EINVAL;
	}

	dxl_serial_disable(ctx);

	k_work_cancel_sync(&ctx->handler_work, &work_sync);
	ctx->rxwait_to = 0;
	atomic_clear_bit(&ctx->state, DXL_STATE_CONFIGURED);

	LOG_INF("Dynamixel interface %u disabled", iface);

	return 0;
}
#define DT_DRV_COMPAT robotis_dynamixel

#include <errno.h>
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

#define DXL_MOTOR_ENTRY(motor_node, parent_inst)                                                   \
	{                                                                                          \
		.label = DT_PROP_OR(motor_node, label, NULL),                                      \
		.iface = parent_inst,                                                              \
		.id = COND_CODE_1(DT_NODE_HAS_PROP(motor_node, reg), (DT_REG_ADDR(motor_node)),    \
				  (DT_PROP(motor_node, id))),                                      \
	},

#define DXL_IFACE_MOTORS(inst) DT_INST_FOREACH_CHILD_VARGS(inst, DXL_MOTOR_ENTRY, inst)

static const struct dxl_motor dxl_motors[] = {DT_INST_FOREACH_STATUS_OKAY(DXL_IFACE_MOTORS)};

size_t dxl_motor_count(void)
{
	return ARRAY_SIZE(dxl_motors);
}

const struct dxl_motor *dxl_motor_get(size_t idx)
{
	if (idx >= ARRAY_SIZE(dxl_motors)) {
		return NULL;
	}
	return &dxl_motors[idx];
}

const struct dxl_motor *dxl_motor_get_by_label(const char *label)
{
	if (label == NULL) {
		return NULL;
	}
	for (size_t i = 0; i < ARRAY_SIZE(dxl_motors); i++) {
		if (dxl_motors[i].label == NULL) {
			continue;
		}
		if (strcmp(dxl_motors[i].label, label) == 0) {
			return &dxl_motors[i];
		}
	}
	return NULL;
}

static void dxl_rx_handler(struct k_work *item)
{
	struct dxl_context *ctx;

	ctx = CONTAINER_OF(item, struct dxl_context, handler_work);

	dxl_serial_rx_disable(ctx);
	ctx->rx_frame_err = dxl_serial_rx(ctx);

	if (ctx->rx_frame_err == -EBADMSG) {
		/* Wrong ID — discard silently and keep waiting for a valid reply. */
		dxl_serial_rx_enable(ctx);
		return;
	}

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

int dxl_disable(int iface)
{
	struct dxl_context *ctx;
	struct k_work_sync work_sync;

	if (iface < 0) {
		return -EINVAL;
	}

	ctx = dxl_get_context((uint8_t)iface);
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

/* Register a no-op Zephyr device per dynamixel instance. The driver itself
 * doesn't use the device API (clients call dxl_iface_get_by_name); the
 * device object is purely a linker artefact. zephyr,uart-emul builds a
 * compile-time emul_link_for_bus[] from its status=okay children using
 * DEVICE_DT_GET, which resolves to __device_dts_ord_N. Without a registered
 * device per dxl-bus child, that symbol is undefined and the test build
 * fails to link.
 */
#define DXL_DEVICE_DEFINE(inst)                                                                    \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, NULL, NULL, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(DXL_DEVICE_DEFINE)

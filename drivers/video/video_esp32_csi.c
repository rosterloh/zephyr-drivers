/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 MIPI CSI-2 camera receiver.
 *
 * Data path: sensor -> CSI D-PHY -> CSI host -> CSI bridge -> DW-GDMA -> memory.
 * The CSI bridge is the DMA flow controller; one DW-GDMA CONTIGUOUS block equals
 * one frame, so the DW-GDMA "full transfer done" interrupt is the end-of-frame
 * signal. There is no Zephyr DW-GDMA driver, so the channel is driven directly
 * through the vendored low-level (LL) header. Modelled on the ESP-IDF
 * esp_cam_ctlr_csi + dw_gdma drivers and the in-tree video_esp32_dvp.c.
 */

#define DT_DRV_COMPAT espressif_esp32p4_mipi_csi

#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hal/mipi_csi_hal.h>
#include <hal/mipi_csi_ll.h>
#include <hal/mipi_csi_brg_ll.h>
#include <hal/dw_gdma_ll.h>
#include <soc/reg_base.h>

#include "video_device.h"

LOG_MODULE_REGISTER(video_esp32_csi, CONFIG_VIDEO_LOG_LEVEL);

/* DW-GDMA transfers are 64-bit wide; a frame is one contiguous block. */
#define CSI_DMA_TRANS_WIDTH_BYTES 8
#define CSI_DMA_CHANNEL           0

/* MIPI CSI-2 data types (payload format identifiers). */
#define CSI_DT_RAW8  0x2a
#define CSI_DT_RAW10 0x2b

struct video_esp32_csi_config {
	const struct device *source_dev;
	uint8_t lane_nb;
	int irq_source;
};

struct video_esp32_csi_data {
	const struct device *dev;
	mipi_csi_hal_context_t hal;
	struct video_format fmt;
	struct video_buffer *active_vbuf;
	uint32_t trans_items; /* frame size expressed in 64-bit DMA items */
	bool streaming;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	intr_handle_t intr_handle;
#ifdef CONFIG_POLL
	struct k_poll_signal *sig_out;
#endif
};

#ifdef CONFIG_POLL
#define CSI_RAISE_SIG(data, res)                                                                   \
	do {                                                                                       \
		if ((data)->sig_out) {                                                             \
			k_poll_signal_raise((data)->sig_out, (res));                               \
		}                                                                                  \
	} while (0)
#else
#define CSI_RAISE_SIG(data, res)
#endif

/* Map a Bayer pixel format to its bits-per-pixel and MIPI CSI-2 data type. */
static int csi_pixfmt_info(uint32_t pixelformat, uint8_t *bpp, uint16_t *data_type)
{
	switch (pixelformat) {
	case VIDEO_PIX_FMT_SBGGR8:
		*bpp = 8;
		*data_type = CSI_DT_RAW8;
		return 0;
	case VIDEO_PIX_FMT_SBGGR10P:
		*bpp = 10;
		*data_type = CSI_DT_RAW10;
		return 0;
	default:
		return -ENOTSUP;
	}
}

/* Point the (already configured) DW-GDMA channel at a buffer and start it. */
static void csi_dma_arm(struct video_esp32_csi_data *data, struct video_buffer *vbuf)
{
	dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);

	/* Drop any stale/dirty cache lines so nothing is written back over the
	 * DMA data, and so the CPU re-reads from PSRAM once the frame lands.
	 */
	sys_cache_data_invd_range(vbuf->buffer, vbuf->bytesused);

	dw_gdma_ll_channel_set_src_addr(dev, CSI_DMA_CHANNEL, MIPI_CSI_BRG_MEM_BASE);
	dw_gdma_ll_channel_set_dst_addr(dev, CSI_DMA_CHANNEL, (uint32_t)(uintptr_t)vbuf->buffer);
	dw_gdma_ll_channel_set_trans_block_size(dev, CSI_DMA_CHANNEL, data->trans_items);
	dw_gdma_ll_channel_set_src_master_port(dev, CSI_DMA_CHANNEL, MIPI_CSI_BRG_MEM_BASE);
	dw_gdma_ll_channel_set_dst_master_port(dev, CSI_DMA_CHANNEL, (uintptr_t)vbuf->buffer);

	data->active_vbuf = vbuf;
	dw_gdma_ll_channel_enable(dev, CSI_DMA_CHANNEL, true);
}

static void video_esp32_csi_isr(void *arg)
{
	struct video_esp32_csi_data *data = arg;
	dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);
	uint32_t status = dw_gdma_ll_channel_get_intr_status(dev, CSI_DMA_CHANNEL);

	dw_gdma_ll_channel_clear_intr(dev, CSI_DMA_CHANNEL, status);

	if (status & DW_GDMA_LL_CHANNEL_EVENT_DMA_TFR_DONE) {
		struct video_buffer *done = data->active_vbuf;
		struct video_buffer *next;

		if (done != NULL) {
			sys_cache_data_invd_range(done->buffer, done->bytesused);
			done->timestamp = k_uptime_get_32();
			k_fifo_put(&data->fifo_out, done);
			CSI_RAISE_SIG(data, VIDEO_BUF_DONE);
		}

		next = k_fifo_get(&data->fifo_in, K_NO_WAIT);
		if (next != NULL) {
			csi_dma_arm(data, next);
		} else {
			data->active_vbuf = NULL;
			CSI_RAISE_SIG(data, VIDEO_BUF_ERROR);
		}
	} else if (status) {
		LOG_ERR("DW-GDMA error status 0x%08x", status);
		CSI_RAISE_SIG(data, VIDEO_BUF_ERROR);
	}
}

/* One-time DW-GDMA controller bring-up (bus clock, reset, enable, global IRQ). */
static void csi_dma_controller_init(void)
{
	dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);
	unsigned int key = irq_lock();

	dw_gdma_ll_enable_bus_clock(0, true);
	dw_gdma_ll_reset_register(0);
	irq_unlock(key);

	dw_gdma_ll_reset(dev);
	dw_gdma_ll_enable_controller(dev, true);
	dw_gdma_ll_enable_intr_global(dev, true);
}

/* Configure the DW-GDMA channel for a peripheral(CSI)->memory contiguous frame. */
static void csi_dma_channel_config(struct video_esp32_csi_data *data)
{
	dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);

	/* Peripheral (CSI bridge) -> memory, flow controlled by the source. */
	dw_gdma_ll_channel_set_trans_flow(dev, CSI_DMA_CHANNEL, DW_GDMA_ROLE_PERIPH_CSI,
					  DW_GDMA_ROLE_MEM, DW_GDMA_FLOW_CTRL_SRC);
	dw_gdma_ll_channel_set_src_multi_block_type(dev, CSI_DMA_CHANNEL,
						    DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
	dw_gdma_ll_channel_set_dst_multi_block_type(dev, CSI_DMA_CHANNEL,
						    DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
	dw_gdma_ll_channel_set_src_handshake_interface(dev, CSI_DMA_CHANNEL, DW_GDMA_HANDSHAKE_HW);
	dw_gdma_ll_channel_set_dst_handshake_interface(dev, CSI_DMA_CHANNEL, DW_GDMA_HANDSHAKE_HW);
	/* Only the peripheral side has a handshake; the LL aborts on ROLE_MEM. */
	dw_gdma_ll_channel_set_src_handshake_periph(dev, CSI_DMA_CHANNEL, DW_GDMA_ROLE_PERIPH_CSI);
	dw_gdma_ll_channel_set_priority(dev, CSI_DMA_CHANNEL, 1);
	dw_gdma_ll_channel_set_src_outstanding_limit(dev, CSI_DMA_CHANNEL, 5);
	dw_gdma_ll_channel_set_dst_outstanding_limit(dev, CSI_DMA_CHANNEL, 5);

	/* Transfer shape: read a fixed FIFO window, write linearly to PSRAM. */
	dw_gdma_ll_channel_set_src_trans_width(dev, CSI_DMA_CHANNEL, DW_GDMA_TRANS_WIDTH_64);
	dw_gdma_ll_channel_set_dst_trans_width(dev, CSI_DMA_CHANNEL, DW_GDMA_TRANS_WIDTH_64);
	dw_gdma_ll_channel_set_src_burst_items(dev, CSI_DMA_CHANNEL, DW_GDMA_BURST_ITEMS_512);
	dw_gdma_ll_channel_set_dst_burst_items(dev, CSI_DMA_CHANNEL, DW_GDMA_BURST_ITEMS_512);
	dw_gdma_ll_channel_set_src_burst_mode(dev, CSI_DMA_CHANNEL, DW_GDMA_BURST_MODE_FIXED);
	dw_gdma_ll_channel_set_dst_burst_mode(dev, CSI_DMA_CHANNEL, DW_GDMA_BURST_MODE_INCREMENT);
	dw_gdma_ll_channel_set_src_burst_len(dev, CSI_DMA_CHANNEL, 16);
	dw_gdma_ll_channel_set_dst_burst_len(dev, CSI_DMA_CHANNEL, 16);
	dw_gdma_ll_channel_enable_src_periph_status_write_back(dev, CSI_DMA_CHANNEL, false);
	dw_gdma_ll_channel_enable_dst_periph_status_write_back(dev, CSI_DMA_CHANNEL, false);

	dw_gdma_ll_channel_enable_intr_generation(dev, CSI_DMA_CHANNEL, UINT32_MAX, true);
	dw_gdma_ll_channel_enable_intr_propagation(dev, CSI_DMA_CHANNEL,
						   DW_GDMA_LL_CHANNEL_EVENT_DMA_TFR_DONE, true);
}

/* Bring up CSI clocks, D-PHY/host/bridge and the RAW data-type filter. */
static int csi_hw_start(const struct device *dev)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	mipi_csi_hal_config_t hal_cfg;
	uint8_t bpp;
	uint16_t data_type;
	int64_t link_freq;
	int ret;
	unsigned int key;

	ret = csi_pixfmt_info(data->fmt.pixelformat, &bpp, &data_type);
	if (ret < 0) {
		LOG_ERR("Unsupported pixel format 0x%08x", data->fmt.pixelformat);
		return ret;
	}

	link_freq = video_get_csi_link_freq(cfg->source_dev, bpp, cfg->lane_nb);
	if (link_freq <= 0) {
		LOG_ERR("Cannot get sensor link frequency (%lld)", link_freq);
		return (int)link_freq;
	}

	/* CSI-2 D-PHY is DDR, so the per-lane bit rate is twice the link freq. */
	hal_cfg = (mipi_csi_hal_config_t){
		.lanes_num = cfg->lane_nb,
		/* NOTE: mipi_csi_hal_init maps frame_height -> horizontal pixel
		 * count and frame_width -> vertical row count (inverted names).
		 */
		.frame_height = data->fmt.width,
		.frame_width = data->fmt.height,
		.in_bpp = bpp,
		.out_bpp = bpp,
		.byte_swap_en = false,
		.lane_bit_rate_mbps = (int)(2 * link_freq / 1000000),
	};

	/* Enable the CSI bridge + host + D-PHY config clocks before touching any
	 * CSI register. (Equivalent to esp_hw_support's mipi_csi_brg_claim, which
	 * we cannot use because it depends on FreeRTOS. We are the sole CSI user,
	 * so no bridge ref-counting is needed.)
	 */
	key = irq_lock();
	mipi_csi_ll_enable_brg_module_clock(0, true);
	mipi_csi_ll_reset_brg_module_clock(0);
	mipi_csi_ll_enable_host_bus_clock(0, true);
	mipi_csi_ll_reset_host_clock(0);
	mipi_csi_ll_set_phy_clock_source(0, MIPI_CSI_PHY_CLK_SRC_DEFAULT);
	mipi_csi_ll_enable_phy_config_clock(0, true);
	irq_unlock(key);
	mipi_csi_brg_ll_enable_clock(MIPI_CSI_BRG_LL_GET_HW(0), true);

	mipi_csi_hal_init(&data->hal, &hal_cfg);

	/* RAW passthrough: filter to the sensor's data type only, no colour
	 * conversion (bridge reset default), 512 x 64-bit DMA bursts.
	 */
	mipi_csi_brg_ll_set_burst_len(data->hal.bridge_dev, 512);
	mipi_csi_brg_ll_set_data_type_min(data->hal.bridge_dev, data_type);
	mipi_csi_brg_ll_set_data_type_max(data->hal.bridge_dev, data_type);

	return 0;
}

static int video_esp32_csi_set_stream(const struct device *dev, bool enable,
				      enum video_buf_type type)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	dw_gdma_dev_t *gdma = DW_GDMA_LL_GET_HW(0);
	struct video_buffer *first;
	uint32_t frame_bytes;
	int ret;

	if (!enable) {
		if (!data->streaming) {
			return 0;
		}
		if (video_stream_stop(cfg->source_dev, type)) {
			return -EIO;
		}
		mipi_csi_brg_ll_enable(data->hal.bridge_dev, false);
		dw_gdma_ll_channel_enable(gdma, CSI_DMA_CHANNEL, false);
		data->streaming = false;
		data->active_vbuf = NULL;
		return 0;
	}

	if (data->streaming) {
		return -EBUSY;
	}

	frame_bytes = (uint32_t)data->fmt.pitch * data->fmt.height;
	if (frame_bytes == 0 || (frame_bytes % CSI_DMA_TRANS_WIDTH_BYTES) != 0) {
		LOG_ERR("Frame size %u not a multiple of %u bytes", frame_bytes,
			CSI_DMA_TRANS_WIDTH_BYTES);
		return -EINVAL;
	}
	data->trans_items = frame_bytes / CSI_DMA_TRANS_WIDTH_BYTES;

	first = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	if (first == NULL) {
		LOG_ERR("No enqueued buffers");
		return -EAGAIN;
	}

	ret = csi_hw_start(dev);
	if (ret < 0) {
		k_fifo_put(&data->fifo_in, first);
		return ret;
	}

	csi_dma_channel_config(data);
	csi_dma_arm(data, first);
	mipi_csi_brg_ll_enable(data->hal.bridge_dev, true);

	if (video_stream_start(cfg->source_dev, type)) {
		mipi_csi_brg_ll_enable(data->hal.bridge_dev, false);
		dw_gdma_ll_channel_enable(gdma, CSI_DMA_CHANNEL, false);
		return -EIO;
	}

	data->streaming = true;
	return 0;
}

static int video_esp32_csi_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct video_esp32_csi_config *cfg = dev->config;

	caps->min_vbuf_count = 2;

	return video_get_caps(cfg->source_dev, caps);
}

static int video_esp32_csi_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	int ret;

	ret = video_get_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	return video_estimate_fmt_size(fmt);
}

static int video_esp32_csi_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	int ret;

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	data->fmt = *fmt;
	return 0;
}

static int video_esp32_csi_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct video_esp32_csi_data *data = dev->data;

	vbuf->bytesused = (uint32_t)data->fmt.pitch * data->fmt.height;
	vbuf->line_offset = 0;
	k_fifo_put(&data->fifo_in, vbuf);

	return 0;
}

static int video_esp32_csi_dequeue(const struct device *dev, struct video_buffer **vbuf,
				   k_timeout_t timeout)
{
	struct video_esp32_csi_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int video_esp32_csi_flush(const struct device *dev, bool cancel)
{
	struct video_esp32_csi_data *data = dev->data;
	struct video_buffer *vbuf;

	if (!cancel) {
		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_sleep(K_MSEC(1));
		}
		return 0;
	}

	if (data->active_vbuf) {
		k_fifo_put(&data->fifo_out, data->active_vbuf);
		data->active_vbuf = NULL;
	}
	while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT)) != NULL) {
		k_fifo_put(&data->fifo_out, vbuf);
		CSI_RAISE_SIG(data, VIDEO_BUF_ABORTED);
	}

	return 0;
}

static int video_esp32_csi_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_esp32_csi_config *cfg = dev->config;

	return video_set_frmival(cfg->source_dev, frmival);
}

static int video_esp32_csi_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_esp32_csi_config *cfg = dev->config;

	return video_get_frmival(cfg->source_dev, frmival);
}

#ifdef CONFIG_POLL
static int video_esp32_csi_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct video_esp32_csi_data *data = dev->data;

	data->sig_out = sig;
	return 0;
}
#endif

static int video_esp32_csi_init(const struct device *dev)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->source_dev)) {
		LOG_ERR("Source device %s not ready", cfg->source_dev->name);
		return -ENODEV;
	}

	data->dev = dev;
	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);

	csi_dma_controller_init();

	ret = esp_intr_alloc(cfg->irq_source, 0, video_esp32_csi_isr, data, &data->intr_handle);
	if (ret != 0) {
		LOG_ERR("Failed to allocate DW-GDMA interrupt (%d)", ret);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(video, video_esp32_csi_api) = {
	.set_format = video_esp32_csi_set_fmt,
	.get_format = video_esp32_csi_get_fmt,
	.set_stream = video_esp32_csi_set_stream,
	.get_caps = video_esp32_csi_get_caps,
	.enqueue = video_esp32_csi_enqueue,
	.dequeue = video_esp32_csi_dequeue,
	.flush = video_esp32_csi_flush,
	.set_frmival = video_esp32_csi_set_frmival,
	.get_frmival = video_esp32_csi_get_frmival,
#ifdef CONFIG_POLL
	.set_signal = video_esp32_csi_set_signal,
#endif
};

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(n, 0, 0)))

#define VIDEO_ESP32_CSI_INIT(n)                                                                    \
	static const struct video_esp32_csi_config video_esp32_csi_config_##n = {                  \
		.source_dev = SOURCE_DEV(n),                                                       \
		.lane_nb = DT_PROP_LEN(DT_INST_ENDPOINT_BY_ID(n, 0, 0), data_lanes),               \
		.irq_source = DT_INST_IRQN(n),                                                     \
	};                                                                                         \
	static struct video_esp32_csi_data video_esp32_csi_data_##n;                               \
	DEVICE_DT_INST_DEFINE(n, video_esp32_csi_init, NULL, &video_esp32_csi_data_##n,            \
			      &video_esp32_csi_config_##n, POST_KERNEL,                            \
			      CONFIG_VIDEO_INIT_PRIORITY, &video_esp32_csi_api);                   \
	VIDEO_DEVICE_DEFINE(video_esp32_csi_##n, DEVICE_DT_INST_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_ESP32_CSI_INIT)

/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 MIPI CSI-2 camera receiver.
 *
 * Data path: sensor -> CSI D-PHY -> CSI host -> ISP -> CSI bridge -> DW-GDMA ->
 * memory. The ISP is not optional even for a RAW passthrough: its
 * cntl.mipi_data_en gates the host-to-bridge link, and with it clear the host
 * receives valid packets while the bridge raises no event and the DMA never
 * fires, with no status bit anywhere pointing at the ISP. See csi_isp_start().
 * The CSI bridge is the DMA flow controller; one DW-GDMA CONTIGUOUS block equals
 * one frame, so the DW-GDMA "full transfer done" interrupt is the end-of-frame
 * signal. There is no Zephyr DW-GDMA driver, so the channel is driven directly
 * through the vendored low-level (LL) header. Modelled on the ESP-IDF
 * esp_cam_ctlr_csi + dw_gdma drivers and the in-tree video_esp32_dvp.c.
 */

#define DT_DRV_COMPAT espressif_esp_mipi_csi

#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <esp_intr_alloc.h>
#include <hal/clk_gate_ll.h>
#include <hal/dw_gdma_ll.h>
#include <hal/isp_ll.h>
#include <hal/mipi_csi_brg_ll.h>
#include <hal/mipi_csi_hal.h>
#include <hal/mipi_csi_ll.h>
#include <soc/isp_struct.h>
#include <soc/mipi_csi_bridge_struct.h>
#include <soc/mipi_csi_host_struct.h>
#include <soc/reg_base.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_esp32_csi, CONFIG_VIDEO_LOG_LEVEL);

/* DW-GDMA transfers are 64-bit wide; a frame is one contiguous block. */
#define CSI_DMA_TRANS_WIDTH_BYTES 8

/* The D-PHY on the ESP32-P4 has two data lanes. */
#define CSI_MAX_DATA_LANES 2

/* Bounded wait for the DW-GDMA bus clock to come up after ungating it. */
#define CSI_DMA_CLK_TIMEOUT_US 100

/*
 * The DMA destination must start on a cache line, or invalidating the head of
 * a frame discards dirty data belonging to whatever precedes it on the heap.
 * csi_cache_len() guards the tail.
 */
#if defined(CONFIG_DCACHE_LINE_SIZE) && (CONFIG_DCACHE_LINE_SIZE > 0)
BUILD_ASSERT(CONFIG_VIDEO_BUFFER_POOL_ALIGN >= CONFIG_DCACHE_LINE_SIZE,
	     "CONFIG_VIDEO_BUFFER_POOL_ALIGN must be at least CONFIG_DCACHE_LINE_SIZE");
#endif

/*
 * Channel events that are not faults and must not be logged as errors. The
 * DW-GDMA disables a channel automatically when a transfer finishes and again
 * whenever software disables it, so DISABLED (and SUSPENDED) arrive during
 * normal streaming and teardown. Reporting them as "DW-GDMA error status
 * 0x40000000" makes a silent CSI link look like a DMA fault, which is
 * actively misleading: a receiver that never gets any data leaves the channel
 * armed, and the only interrupt ever seen is the DISABLED from
 * video_stream_stop() long after the real failure.
 */
#define CSI_DMA_BENIGN_EVENTS                                                                      \
	(DW_GDMA_LL_CHANNEL_EVENT_DISABLED | DW_GDMA_LL_CHANNEL_EVENT_SUSPENDED |                  \
	 DW_GDMA_LL_CHANNEL_EVENT_SRC_SUSPENDED | DW_GDMA_LL_CHANNEL_EVENT_BLOCK_TFR_DONE |        \
	 DW_GDMA_LL_CHANNEL_EVENT_SRC_TRANSCOMP | DW_GDMA_LL_CHANNEL_EVENT_DST_TRANSCOMP)

struct video_esp32_csi_config {
	const struct device *source_dev;
	uint8_t lane_nb;
	uint8_t dma_channel;
	int irq_source;
	uint8_t irq_priority;
	uint8_t irq_flags;
};

struct video_esp32_csi_data {
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

/*
 * Map a pixel format to its bits-per-pixel and MIPI CSI-2 data type. Only RAW8,
 * RAW10 and RAW12 are captured: those are the depths the ISP stage below can
 * pass through untouched. Greyscale and Bayer share a data type at each depth -
 * the CSI-2 payload is identical, only the interpretation differs - so ToF
 * modules reporting Y10P land on the same RAW10 path as a Bayer sensor.
 */
static int csi_pixfmt_info(uint32_t pixelformat, uint8_t *bpp, uint8_t *data_type)
{
	uint8_t dt = video_mipi_data_type(pixelformat);

	switch (dt) {
	case VIDEO_MIPI_CSI2_DT_RAW8:
	case VIDEO_MIPI_CSI2_DT_RAW10:
	case VIDEO_MIPI_CSI2_DT_RAW12:
		*bpp = video_bits_per_pixel(pixelformat);
		*data_type = dt;
		return 0;
	default:
		return -ENOTSUP;
	}
}

/*
 * Length of the cache-maintenance window for a buffer.
 *
 * Cache ops must stay inside the lines the buffer actually owns. The DMA
 * destination is CONFIG_VIDEO_BUFFER_POOL_ALIGN-aligned, but the allocation
 * *length* is whatever the caller asked for -- a RAW10 frame is rarely a
 * multiple of the line size -- so invalidating bytesused would take out the
 * dirty half of the trailing line, which belongs to the next heap chunk. Its
 * header is then garbage and the next free() faults.
 *
 * Callers that round their allocation up to a cache line get the whole frame
 * invalidated. Ones that do not leave a stale tail of at most a line, which is
 * a far better failure mode than corrupting the heap.
 *
 * Coupled to CONFIG_VIDEO_BUFFER_POOL_ALIGN >= CONFIG_DCACHE_LINE_SIZE: this
 * guards the tail, POOL_ALIGN guards the head. Dropping POOL_ALIGN below the
 * line size reintroduces the same corruption at the start of the buffer.
 */
static inline size_t csi_cache_len(const struct video_buffer *vbuf)
{
	return ROUND_DOWN(vbuf->size, CONFIG_DCACHE_LINE_SIZE);
}

/*
 * ---------------------------------------------------------------------------
 * ISP stage
 *
 * On the ESP32-P4 the CSI host does not feed the bridge directly: the ISP sits
 * between them, and its cntl.mipi_data_en gates the whole path. With that bit
 * clear the host receives perfectly valid packets - no ECC, CRC, payload or
 * frame-boundary errors latch - while the bridge raises no event at all and the
 * DMA never fires. No status register anywhere points at the ISP, which is why
 * this is worth spelling out and why it lives next to the bridge setup that
 * appears to be the whole story.
 *
 * ESP-IDF hides the coupling: esp_video always creates an ISP processor, even
 * for a RAW passthrough where it sets bypass_isp.
 *
 * Deliberately only a wire, not a pipeline stage: no demosaic, no format
 * conversion, no statistics, no controls. That is why it is two static
 * functions here rather than its own video device - Zephyr has no ISP device
 * convention to conform to, and the block has exactly one consumer whose
 * lifetime it shares. Promote these to a separate device (following the split
 * ST uses for DCMIPP, which the board DTS already anticipates with its
 * csi_interface / csi_capture_port labels) when any of the following lands:
 * ISP-processed output formats, AE/AWB statistics, or a second consumer such
 * as the DVP driver needing arbitration.
 * ---------------------------------------------------------------------------
 */
static void csi_isp_start(struct video_esp32_csi_data *data, uint8_t bpp)
{
	isp_ll_enable_module_clock(true);
	isp_ll_reset_module_clock();
	isp_ll_init(&ISP);
	isp_ll_clk_enable(&ISP, true);
	isp_ll_set_input_data_source(&ISP, ISP_INPUT_DATA_SOURCE_CSI);

	/*
	 * Two ways through the ISP, and they configure different registers.
	 *
	 * RAW8 uses the processing path with demosaic off: RAW8 in, RAW8 out,
	 * Hsize in pixels. Verified streaming on hardware.
	 *
	 * RAW10/RAW12 cannot use it, because isp_ll_set_output_data_color_format()
	 * has no RAW10 or RAW12 output type - only RAW8, RGB and YUV - so the depth
	 * would not survive. Those depths need bypass, which per
	 * esp_driver_isp/src/isp_core.c means skipping both colour-format setters
	 * and re-reading the horizontal register: "when ISP bypass, input Hsize
	 * needs to be re-calculated according to input bits per pixel and IDI32.
	 * Hsize now stands for the number of 32-bit in one line."
	 *
	 * Verified on hardware with an Arducam ToF module: 240x180 Y12P, 64800
	 * bytes per frame. Note the two settings are not independent - bypass
	 * needs BOTH the word-based Hsize here and isp_en left clear below.
	 * Applying only the Hsize while the ISP stayed enabled is what made an
	 * earlier attempt look like "bypass breaks RAW8 too".
	 */
	if (bpp == 8) {
		isp_ll_set_input_data_color_format(&ISP, ISP_COLOR_RAW8);
		isp_ll_set_output_data_color_format(&ISP, ISP_COLOR_RAW8);
		isp_ll_set_intput_data_h_pixel_num(&ISP, data->fmt.width);
	} else {
		isp_ll_set_intput_data_h_pixel_num(&ISP, DIV_ROUND_UP(data->fmt.width * bpp, 32));
	}
	isp_ll_set_intput_data_v_row_num(&ISP, data->fmt.height);

	/*
	 * CSI-2 line start/end short packets. The ISP must be told whether the
	 * sensor sends them or the two ends disagree about line framing. There
	 * is no way to query a sensor for this through the video API, so follow
	 * what the sensors verified here do: their RAW8 modes send none, and
	 * Espressif's OV5647 RAW10 mode table (MIPI_CTRL00 = 0x34) turns line
	 * sync on, as esp_video expects at the deeper depths.
	 */
	isp_ll_enable_line_start_packet_exist(&ISP, bpp != 8);
	isp_ll_enable_line_end_packet_exist(&ISP, bpp != 8);

	/*
	 * Bypass is the ISP left disabled while cntl.mipi_data_en - set by
	 * isp_ll_set_input_data_source() above, independent of isp_en - holds the
	 * CSI -> bridge path open. ESP-IDF says the same as a guard:
	 * esp_isp_enable() refuses to run on a processor configured for bypass,
	 * so isp_en is never set on that path.
	 */
	isp_ll_enable(&ISP, bpp == 8);
}

static void csi_isp_stop(void)
{
	isp_ll_enable(&ISP, false);
}

/* Point the (already configured) DW-GDMA channel at a buffer and start it. */
static void csi_dma_arm(const struct device *dev, struct video_buffer *vbuf)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	dw_gdma_dev_t *dma = DW_GDMA_LL_GET_HW(0);
	uint8_t ch = cfg->dma_channel;

	/*
	 * Drop any stale/dirty cache lines so nothing is written back over the
	 * DMA data, and so the CPU re-reads from PSRAM once the frame lands.
	 */
	sys_cache_data_invd_range(vbuf->buffer, csi_cache_len(vbuf));

	dw_gdma_ll_channel_set_src_addr(dma, ch, MIPI_CSI_BRG_MEM_BASE);
	dw_gdma_ll_channel_set_dst_addr(dma, ch, (uint32_t)(uintptr_t)vbuf->buffer);
	dw_gdma_ll_channel_set_trans_block_size(dma, ch, data->trans_items);
	dw_gdma_ll_channel_set_src_master_port(dma, ch, MIPI_CSI_BRG_MEM_BASE);
	dw_gdma_ll_channel_set_dst_master_port(dma, ch, (uintptr_t)vbuf->buffer);

	data->active_vbuf = vbuf;
	dw_gdma_ll_channel_enable(dma, ch, true);
}

static void video_esp32_csi_isr(void *arg)
{
	const struct device *dev = arg;
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	dw_gdma_dev_t *dma = DW_GDMA_LL_GET_HW(0);
	uint32_t status = dw_gdma_ll_channel_get_intr_status(dma, cfg->dma_channel);

	dw_gdma_ll_channel_clear_intr(dma, cfg->dma_channel, status);

	if (status & DW_GDMA_LL_CHANNEL_EVENT_DMA_TFR_DONE) {
		struct video_buffer *done = data->active_vbuf;
		struct video_buffer *next;

		if (done != NULL) {
			sys_cache_data_invd_range(done->buffer, csi_cache_len(done));
			done->timestamp = k_uptime_get_32();
			k_fifo_put(&data->fifo_out, done);
			CSI_RAISE_SIG(data, VIDEO_BUF_DONE);
		}

		next = k_fifo_get(&data->fifo_in, K_NO_WAIT);
		if (next != NULL) {
			csi_dma_arm(dev, next);
		} else {
			data->active_vbuf = NULL;
			CSI_RAISE_SIG(data, VIDEO_BUF_ERROR);
		}
	} else if (status & ~CSI_DMA_BENIGN_EVENTS) {
		LOG_ERR("DW-GDMA error status 0x%08x", status);
		CSI_RAISE_SIG(data, VIDEO_BUF_ERROR);
	}
}

/*
 * DW-GDMA controller bring-up. The controller is shared: the MIPI DSI display
 * driver drives other channels of it, and there is no Zephyr DMA driver to
 * arbitrate. Only reset it if nobody has enabled it yet, so bringing up the
 * camera cannot wipe a channel the display is already scanning out on.
 */
static int csi_dma_controller_init(void)
{
	dw_gdma_dev_t *dma = DW_GDMA_LL_GET_HW(0);
	unsigned int key;

	if (!dw_gdma_ll_is_bus_clock_enabled(0)) {
		key = irq_lock();
		dw_gdma_ll_enable_bus_clock(0, true);
		irq_unlock(key);
		if (!WAIT_FOR(dw_gdma_ll_is_bus_clock_enabled(0), CSI_DMA_CLK_TIMEOUT_US,
			      k_busy_wait(1))) {
			LOG_ERR("DW-GDMA bus clock did not come up");
			return -ETIMEDOUT;
		}
	}

	if (dma->cfg0.dmac_en == 0) {
		key = irq_lock();
		dw_gdma_ll_reset_register(0);
		irq_unlock(key);
		dw_gdma_ll_reset(dma);
		dw_gdma_ll_enable_controller(dma, true);
	}
	dw_gdma_ll_enable_intr_global(dma, true);

	return 0;
}

/* Configure the DW-GDMA channel for a peripheral(CSI)->memory contiguous frame. */
static void csi_dma_channel_config(const struct device *dev)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	dw_gdma_dev_t *dma = DW_GDMA_LL_GET_HW(0);
	uint8_t ch = cfg->dma_channel;

	/* Peripheral (CSI bridge) -> memory, flow controlled by the source. */
	dw_gdma_ll_channel_set_trans_flow(dma, ch, DW_GDMA_ROLE_PERIPH_CSI, DW_GDMA_ROLE_MEM,
					  DW_GDMA_FLOW_CTRL_SRC);
	dw_gdma_ll_channel_set_src_multi_block_type(dma, ch, DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
	dw_gdma_ll_channel_set_dst_multi_block_type(dma, ch, DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
	dw_gdma_ll_channel_set_src_handshake_interface(dma, ch, DW_GDMA_HANDSHAKE_HW);
	dw_gdma_ll_channel_set_dst_handshake_interface(dma, ch, DW_GDMA_HANDSHAKE_HW);
	/* Only the peripheral side has a handshake; the LL aborts on ROLE_MEM. */
	dw_gdma_ll_channel_set_src_handshake_periph(dma, ch, DW_GDMA_ROLE_PERIPH_CSI);
	dw_gdma_ll_channel_set_priority(dma, ch, 1);
	dw_gdma_ll_channel_set_src_outstanding_limit(dma, ch, 5);
	dw_gdma_ll_channel_set_dst_outstanding_limit(dma, ch, 5);

	/* Transfer shape: read a fixed FIFO window, write linearly to PSRAM. */
	dw_gdma_ll_channel_set_src_trans_width(dma, ch, DW_GDMA_TRANS_WIDTH_64);
	dw_gdma_ll_channel_set_dst_trans_width(dma, ch, DW_GDMA_TRANS_WIDTH_64);
	dw_gdma_ll_channel_set_src_burst_items(dma, ch, DW_GDMA_BURST_ITEMS_512);
	dw_gdma_ll_channel_set_dst_burst_items(dma, ch, DW_GDMA_BURST_ITEMS_512);
	dw_gdma_ll_channel_set_src_burst_mode(dma, ch, DW_GDMA_BURST_MODE_FIXED);
	dw_gdma_ll_channel_set_dst_burst_mode(dma, ch, DW_GDMA_BURST_MODE_INCREMENT);
	dw_gdma_ll_channel_set_src_burst_len(dma, ch, 16);
	dw_gdma_ll_channel_set_dst_burst_len(dma, ch, 16);
	dw_gdma_ll_channel_enable_src_periph_status_write_back(dma, ch, false);
	dw_gdma_ll_channel_enable_dst_periph_status_write_back(dma, ch, false);

	dw_gdma_ll_channel_enable_intr_generation(dma, ch, UINT32_MAX, true);
	dw_gdma_ll_channel_enable_intr_propagation(dma, ch, DW_GDMA_LL_CHANNEL_EVENT_DMA_TFR_DONE,
						   true);
}

/* Bring up CSI clocks, D-PHY/host/bridge and the RAW data-type filter. */
static int csi_hw_start(const struct device *dev)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	mipi_csi_hal_config_t hal_cfg;
	uint8_t bpp;
	uint8_t data_type;
	int64_t link_freq;
	int ret;
	unsigned int key;

	ret = csi_pixfmt_info(data->fmt.pixelformat, &bpp, &data_type);
	if (ret < 0) {
		LOG_ERR("Unsupported pixel format %s", VIDEO_FOURCC_TO_STR(data->fmt.pixelformat));
		return ret;
	}

	link_freq = video_get_csi_link_freq(cfg->source_dev, bpp, cfg->lane_nb);
	if (link_freq <= 0) {
		LOG_ERR("Cannot get sensor link frequency (%d)", (int)link_freq);
		/*
		 * Not "return (int)link_freq": zero is a failure here (the source
		 * advertised neither LINK_FREQ nor a usable PIXEL_RATE) but would
		 * be returned to the caller as success, leaving mipi_csi_hal_init()
		 * never run and the first bridge access faulting on a NULL
		 * hal.bridge_dev.
		 */
		return link_freq < 0 ? (int)link_freq : -EINVAL;
	}

	/* CSI-2 D-PHY is DDR, so the per-lane bit rate is twice the link freq. */
	hal_cfg = (mipi_csi_hal_config_t){
		.lanes_num = cfg->lane_nb,
		/*
		 * mipi_csi_hal_init() maps frame_height to the horizontal count
		 * and frame_width to the vertical row count (inverted names).
		 *
		 * The bridge counts BYTES per line, not pixels. Passing the pixel
		 * width works only while bits-per-pixel is 8; for packed RAW10 a
		 * 1920-pixel line is 2400 bytes, the bridge runs past the count it
		 * was given, and rows come up short - it reports
		 * vadr_num_gt_real ("reg_vadr_num is greater than real") in
		 * int_raw and never completes a frame. pitch is exactly this
		 * byte count, and equals width for every 8-bit format.
		 */
		.frame_height = data->fmt.pitch,
		.frame_width = data->fmt.height,
		.in_bpp = bpp,
		.out_bpp = bpp,
		.byte_swap_en = false,
		.lane_bit_rate_mbps = (int)(2 * link_freq / 1000000),
	};

	/*
	 * Enable the CSI bridge, host and D-PHY config clocks before touching
	 * any CSI register. This is what esp_hw_support's mipi_csi_brg_claim()
	 * does, which cannot be used here because it depends on FreeRTOS. This
	 * driver is the sole CSI user, so no bridge ref-counting is needed.
	 *
	 * Ungate the D-PHY reference clock before selecting it. On the P4
	 * MIPI_CSI_PHY_CLK_SRC_DEFAULT is PLL_F20M, which is gated at boot.
	 *
	 * Do NOT reach for esp_clk_tree_enable_src() here: on the P4 its derived-
	 * clock table (esp_clk_tree_get_derived_clk_desc) only knows PLL_F50M, so
	 * for PLL_F20M it returns NULL and the call succeeds without ungating
	 * anything. The D-PHY is then left with no reference, which is invisible in
	 * every status register the host exposes -- shutdownz/rstz read released,
	 * the PHY test interface still reads back what hal_init wrote (it runs off
	 * the config clock), and the only symptom is phy_stopstate and
	 * phy_rxclkactivehs stuck at 0 with no error bit anywhere. Gate the 20 MHz
	 * reference directly instead; the _clk_gate_ll_ prefix is the lock-free
	 * variant, which is what the clk_tree descriptors themselves use, and we
	 * are already inside irq_lock().
	 */
	key = irq_lock();
	_clk_gate_ll_ref_20m_clk_en(true);
	mipi_csi_ll_enable_brg_module_clock(0, true);
	mipi_csi_ll_reset_brg_module_clock(0);
	/*
	 * Pulse the host bus clock off then on before resetting, as
	 * esp_cam_ctlr_csi's s_csi_claim_controller() does.
	 */
	mipi_csi_ll_enable_host_bus_clock(0, false);
	mipi_csi_ll_enable_host_bus_clock(0, true);
	mipi_csi_ll_reset_host_clock(0);
	mipi_csi_ll_set_phy_clock_source(0, MIPI_CSI_PHY_CLK_SRC_DEFAULT);
	/*
	 * Pulse the PHY config clock rather than just enabling it, as
	 * esp_cam_ctlr_csi does. mipi_csi_hal_init() programs hs_freq_sel through
	 * the PHY test interface, which is clocked by this clock; if it was left
	 * running in a stale state the write may not latch and the PHY PLL never
	 * locks.
	 */
	mipi_csi_ll_enable_phy_config_clock(0, false);
	mipi_csi_ll_enable_phy_config_clock(0, true);
	irq_unlock(key);
	mipi_csi_brg_ll_enable_clock(MIPI_CSI_BRG_LL_GET_HW(0), true);

	mipi_csi_hal_init(&data->hal, &hal_cfg);

	/*
	 * Unmask every host error source. On this IP INT_ST_* only latches an event
	 * whose INT_MSK_* bit is set, and both the HAL and reset leave the masks at
	 * 0 -- so an all-zero error dump means "nothing was being recorded", not
	 * "nothing went wrong". Without this a receiver failing to sync to the
	 * sensor's HS bursts is indistinguishable from a silent link.
	 */
	data->hal.host_dev->int_msk_phy_fatal.val = UINT32_MAX;
	data->hal.host_dev->int_msk_pkt_fatal.val = UINT32_MAX;
	data->hal.host_dev->int_msk_phy.val = UINT32_MAX;
	data->hal.host_dev->int_msk_bndry_frame_fatal.val = UINT32_MAX;
	data->hal.host_dev->int_msk_seq_frame_fatal.val = UINT32_MAX;
	data->hal.host_dev->int_msk_crc_frame_fatal.val = UINT32_MAX;
	data->hal.host_dev->int_msk_pld_crc_fatal.val = UINT32_MAX;
	data->hal.host_dev->int_msk_data_id.val = UINT32_MAX;
	data->hal.host_dev->int_msk_ecc_corrected.val = UINT32_MAX;

	/*
	 * RAW passthrough: filter to the sensor's data type only, no colour
	 * conversion (bridge reset default), 512 x 64-bit DMA bursts.
	 */
	mipi_csi_brg_ll_set_burst_len(data->hal.bridge_dev, 512);
	mipi_csi_brg_ll_set_data_type_min(data->hal.bridge_dev, data_type);
	mipi_csi_brg_ll_set_data_type_max(data->hal.bridge_dev, data_type);

	/*
	 * Arm the bridge's row-count check. It is off by default, which means the
	 * vadr_num_gt/lt interrupts never fire and a bridge receiving the wrong
	 * number of rows looks identical to one receiving none at all.
	 */
	data->hal.bridge_dev->frame_cfg.vadr_num_check = 1;

	csi_isp_start(data, bpp);

	return 0;
}

static int video_esp32_csi_set_stream(const struct device *dev, bool enable,
				      enum video_buf_type type)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	dw_gdma_dev_t *dma = DW_GDMA_LL_GET_HW(0);
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
		dw_gdma_ll_channel_enable(dma, cfg->dma_channel, false);
		/* Close the ISP's MIPI gate again; csi_hw_start() reopens it. */
		csi_isp_stop();
		data->streaming = false;
		/*
		 * Leave active_vbuf alone: video_stream_stop() calls flush(cancel)
		 * straight after this, and that is what hands the armed buffer back
		 * through fifo_out. Clearing it here dropped the buffer on the floor --
		 * with a two-buffer pool, one aborted stream permanently halved it and
		 * the next capture failed to allocate. The DMA channel is already
		 * disabled above, so nothing else can touch it in between.
		 */
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

	csi_dma_channel_config(dev);
	csi_dma_arm(dev, first);
	mipi_csi_brg_ll_enable(data->hal.bridge_dev, true);

	if (video_stream_start(cfg->source_dev, type)) {
		mipi_csi_brg_ll_enable(data->hal.bridge_dev, false);
		dw_gdma_ll_channel_enable(dma, cfg->dma_channel, false);
		/*
		 * csi_dma_arm() checked the buffer out into active_vbuf. streaming
		 * stays false here, so the teardown path that normally returns it via
		 * flush(cancel) never runs and the next set_stream would overwrite the
		 * pointer. Hand it back explicitly.
		 */
		k_fifo_put(&data->fifo_in, data->active_vbuf);
		data->active_vbuf = NULL;
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

/*
 * Round the stride up to a 32-bit word.
 *
 * In bypass the ISP is configured in words - its Hsize is
 * DIV_ROUND_UP(width * bpp, 32) - so it emits whole words per line and pads a
 * line that does not divide evenly. The bridge and the DMA are sized from
 * fmt.pitch, and if that is the unpadded byte count the two ends disagree
 * about line length: the ISP delivers more than the buffer was sized for and
 * the DMA writes past its end, into the next heap chunk.
 *
 * Measured on hardware, IMX219 RAW10 at 1640x1232: pitch 2050 is 512.5 words,
 * the ISP rounds to 513 (2052 bytes), and the DMA finished 1848 bytes beyond
 * the buffer. Every geometry whose pitch is already a multiple of 4 - RAW10 at
 * 1600/640/320, RAW8 at 1640, RAW12 at 240 - overran by exactly zero.
 *
 * This must be applied on BOTH set_fmt() and get_fmt(). get_fmt() does not
 * return the stored format, it re-queries the source and recomputes the size,
 * so padding only in set_fmt() leaves every consumer allocating the unpadded
 * length while the DMA writes the padded one.
 */
static void csi_pad_pitch(struct video_format *fmt)
{
	fmt->pitch = ROUND_UP(fmt->pitch, sizeof(uint32_t));
}

static int video_esp32_csi_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	int ret;

	ret = video_get_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	csi_pad_pitch(fmt);

	return 0;
}

static int video_esp32_csi_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_esp32_csi_config *cfg = dev->config;
	struct video_esp32_csi_data *data = dev->data;
	uint8_t bpp;
	uint8_t data_type;
	int ret;

	if (data->streaming) {
		return -EBUSY;
	}

	ret = csi_pixfmt_info(fmt->pixelformat, &bpp, &data_type);
	if (ret < 0) {
		LOG_ERR("Unsupported pixel format %s", VIDEO_FOURCC_TO_STR(fmt->pixelformat));
		return ret;
	}

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	csi_pad_pitch(fmt);

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

static int video_esp32_csi_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	const struct video_esp32_csi_config *cfg = dev->config;

	return video_enum_frmival(cfg->source_dev, fie);
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

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);

	ret = csi_dma_controller_init();
	if (ret < 0) {
		return ret;
	}

	/*
	 * The DW-GDMA raises one interrupt for all of its channels, which the
	 * MIPI DSI display driver also listens on, so share it and only take
	 * the calls for this channel.
	 */
	ret = esp_intr_alloc_intrstatus(
		cfg->irq_source,
		ESP_INTR_FLAG_SHARED | ESP_PRIO_TO_FLAGS(cfg->irq_priority) |
			ESP_INT_FLAGS_CHECK(cfg->irq_flags),
		(uint32_t)dw_gdma_ll_get_intr_status_reg(DW_GDMA_LL_GET_HW(0)),
		DW_GDMA_LL_CHANNEL_EVENT_MASK(cfg->dma_channel), video_esp32_csi_isr, (void *)dev,
		&data->intr_handle);
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
	.enum_frmival = video_esp32_csi_enum_frmival,
#ifdef CONFIG_POLL
	.set_signal = video_esp32_csi_set_signal,
#endif
};

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(n, 0, 0)))
#define LANE_NB(n)    DT_PROP_LEN(DT_INST_ENDPOINT_BY_ID(n, 0, 0), data_lanes)

#define VIDEO_ESP32_CSI_INIT(n)                                                                    \
	BUILD_ASSERT(LANE_NB(n) >= 1 && LANE_NB(n) <= CSI_MAX_DATA_LANES,                          \
		     "ESP32-P4 MIPI CSI supports 1 or 2 data lanes");                              \
	static const struct video_esp32_csi_config video_esp32_csi_config_##n = {                  \
		.source_dev = SOURCE_DEV(n),                                                       \
		.lane_nb = LANE_NB(n),                                                             \
		.dma_channel = DT_INST_PROP(n, dma_channel),                                       \
		.irq_source = DT_INST_IRQ_BY_IDX(n, 0, irq),                                       \
		.irq_priority = DT_INST_IRQ_BY_IDX(n, 0, priority),                                \
		.irq_flags = DT_INST_IRQ_BY_IDX(n, 0, flags),                                      \
	};                                                                                         \
	static struct video_esp32_csi_data video_esp32_csi_data_##n;                               \
	DEVICE_DT_INST_DEFINE(n, video_esp32_csi_init, NULL, &video_esp32_csi_data_##n,            \
			      &video_esp32_csi_config_##n, POST_KERNEL,                            \
			      CONFIG_VIDEO_INIT_PRIORITY, &video_esp32_csi_api);                   \
	VIDEO_DEVICE_DEFINE(video_esp32_csi_##n, DEVICE_DT_INST_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_ESP32_CSI_INIT)

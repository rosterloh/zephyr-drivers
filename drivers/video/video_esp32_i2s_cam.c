/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32 (original) I2S0 camera-mode DVP receiver, JPEG only.
 *
 * Register sequence and sampling mode follow Espressif's esp32-camera
 * (target/esp32/ll_cam.c, Apache-2.0). I2S0 DMAs into a circular ring in
 * internal RAM and raises in_suc_eof every half-ring-slot (4 KiB); a thread
 * strips the FIFO padding into the application's buffer, which may be in
 * PSRAM. VSYNC edges (a GPIO interrupt) delimit frames.
 */

#define DT_DRV_COMPAT espressif_esp32_i2s_cam

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/video/video.h>

#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/i2s_struct.h>
#include <soc/lldesc.h>

#include "esp32_i2s_cam_jpeg.h"
#include "video_common.h"

LOG_MODULE_REGISTER(video_esp32_i2s_cam, CONFIG_VIDEO_LOG_LEVEL);

#define I2S_NODE DT_INST_PHANDLE(0, i2s)

#define RING_SIZE  CONFIG_VIDEO_ESP32_I2S_CAM_DMA_RING_SIZE
#define NODE_SIZE  2048
#define HALF_SIZE  4096
#define HALF_COUNT (RING_SIZE / HALF_SIZE)
#define HALF_OUT   (HALF_SIZE / 4) /* camera bytes per half */

BUILD_ASSERT(RING_SIZE % HALF_SIZE == 0 && HALF_COUNT >= 2,
	     "DMA ring must be at least two 4 KiB halves");

/* rx_fifo_mod: one camera byte per 32-bit FIFO word ("00 s1 00 00") */
#define SM_0A00_0B00 3

enum cam_event {
	EV_VSYNC,
	EV_EOF,
	EV_STOP,
};

struct cam_config {
	const struct device *source_dev;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec vsync;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	int irq_source;
	int irq_priority;
	int irq_flags;
};

struct cam_data {
	struct video_format fmt;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct k_msgq evq;
	uint8_t evq_buf[16];
	struct k_sem stopped;
	struct gpio_callback vsync_cb;
	atomic_t overrun;
	bool streaming;
#ifdef CONFIG_POLL
	struct k_poll_signal *signal_out;
#endif
	/* Owned by the copy thread */
	struct video_buffer *vbuf;
	size_t len;
	size_t cnt;
	bool reading;
};

static lldesc_t cam_desc[RING_SIZE / NODE_SIZE] __aligned(4);
static uint8_t cam_ring[RING_SIZE] __aligned(4);

static const struct pwm_dt_spec cam_xclk = PWM_DT_SPEC_INST_GET_BY_NAME(0, xclk);

K_THREAD_STACK_DEFINE(cam_stack, CONFIG_VIDEO_ESP32_I2S_CAM_THREAD_STACK_SIZE);
static struct k_thread cam_thread;

static void cam_raise(struct cam_data *data, enum video_signal_result r)
{
#ifdef CONFIG_POLL
	if (data->signal_out) {
		k_poll_signal_raise(data->signal_out, r);
	}
#endif
}

static void cam_post(struct cam_data *data, uint8_t ev)
{
	if (k_msgq_put(&data->evq, &ev, K_NO_WAIT) != 0) {
		atomic_set(&data->overrun, 1);
	}
}

static void cam_i2s_isr(void *arg)
{
	struct cam_data *data = arg;
	typeof(I2S0.int_st) st = {.val = I2S0.int_st.val};

	I2S0.int_clr.val = st.val;
	if (st.in_suc_eof) {
		cam_post(data, EV_EOF);
	}
}

static void cam_vsync_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	cam_post(CONTAINER_OF(cb, struct cam_data, vsync_cb), EV_VSYNC);
}

static void cam_i2s_reset(void)
{
	I2S0.conf.rx_reset = 1;
	I2S0.conf.rx_reset = 0;
	I2S0.conf.rx_fifo_reset = 1;
	I2S0.conf.rx_fifo_reset = 0;
	I2S0.lc_conf.in_rst = 1;
	I2S0.lc_conf.in_rst = 0;
	I2S0.lc_conf.ahbm_fifo_rst = 1;
	I2S0.lc_conf.ahbm_fifo_rst = 0;
	I2S0.lc_conf.ahbm_rst = 1;
	I2S0.lc_conf.ahbm_rst = 0;
}

static void cam_dma_start(void)
{
	I2S0.conf.rx_start = 0;
	I2S0.int_clr.in_suc_eof = 1;
	I2S0.int_ena.in_suc_eof = 1;
	cam_i2s_reset();
	I2S0.rx_eof_num = HALF_SIZE / sizeof(uint32_t);
	I2S0.in_link.addr = (uint32_t)&cam_desc[0] & 0xfffff;
	I2S0.in_link.start = 1;
	I2S0.conf.rx_start = 1;
}

static void cam_dma_stop(void)
{
	I2S0.conf.rx_start = 0;
	I2S0.int_ena.in_suc_eof = 0;
	I2S0.int_clr.in_suc_eof = 1;
	I2S0.in_link.stop = 1;
}

static void cam_i2s_config(void)
{
	cam_i2s_reset();

	I2S0.conf.rx_slave_mod = 1;
	I2S0.conf.rx_right_first = 0;
	I2S0.conf.rx_msb_right = 0;
	I2S0.conf.rx_msb_shift = 0;
	I2S0.conf.rx_mono = 0;
	I2S0.conf.rx_short_sync = 0;

	I2S0.conf2.lcd_en = 1;
	I2S0.conf2.camera_en = 1;

	I2S0.clkm_conf.clkm_div_a = 0;
	I2S0.clkm_conf.clkm_div_b = 0;
	I2S0.clkm_conf.clkm_div_num = 2;

	I2S0.fifo_conf.dscr_en = 1;
	I2S0.fifo_conf.rx_fifo_mod = SM_0A00_0B00;
	I2S0.fifo_conf.rx_fifo_mod_force_en = 1;

	I2S0.conf_chan.rx_chan_mod = 1;
	I2S0.sample_rate_conf.rx_bits_mod = 0;
	I2S0.timing.val = 0;
	I2S0.timing.rx_dsync_sw = 1;

	for (size_t i = 0; i < ARRAY_SIZE(cam_desc); i++) {
		memset(&cam_desc[i], 0, sizeof(cam_desc[i]));
		cam_desc[i].size = NODE_SIZE;
		cam_desc[i].owner = 1;
		cam_desc[i].buf = &cam_ring[i * NODE_SIZE];
		cam_desc[i].empty = (uint32_t)&cam_desc[(i + 1) % ARRAY_SIZE(cam_desc)];
	}
}

static bool cam_begin_frame(struct cam_data *data)
{
	if (data->vbuf == NULL) {
		data->vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	}
	if (data->vbuf == NULL) {
		data->reading = false;
		return false;
	}
	data->len = 0;
	data->cnt = 0;
	data->reading = true;
	cam_dma_start();
	return true;
}

static void cam_abort_frame(struct cam_data *data)
{
	cam_dma_stop();
	data->reading = false; /* keep vbuf for the next frame */
}

/* Copy the half the DMA just finished (or, at VSYNC, is filling). */
static bool cam_copy_half(struct cam_data *data)
{
	const uint8_t *src = &cam_ring[(data->cnt % HALF_COUNT) * HALF_SIZE];

	if (data->len + HALF_OUT > data->vbuf->size) {
		LOG_WRN("Frame larger than buffer (%u B)", data->vbuf->size);
		return false;
	}
	data->len += esp32_i2s_cam_unpack(data->vbuf->buffer + data->len, src, HALF_SIZE);
	data->cnt++;
	return true;
}

static void cam_end_frame(struct cam_data *data)
{
	ssize_t end;

	cam_dma_stop();

	if (data->cnt == 0 || !cam_copy_half(data)) {
		cam_begin_frame(data);
		return;
	}

	/* EOI is in the last two halves; the final one may end in stale bytes. */
	end = esp32_i2s_cam_find_eoi(data->vbuf->buffer, data->len,
				     data->len > 2 * HALF_OUT ? data->len - 2 * HALF_OUT : 0);
	if (end < 0) {
		LOG_DBG("No EOI, frame dropped");
	} else {
		data->vbuf->bytesused = end;
		data->vbuf->timestamp = k_uptime_get_32();
		k_fifo_put(&data->fifo_out, data->vbuf);
		data->vbuf = NULL;
		cam_raise(data, VIDEO_BUF_DONE);
	}

	cam_begin_frame(data);
}

static void cam_thread_fn(void *p1, void *p2, void *p3)
{
	struct cam_data *data = p1;
	uint8_t ev;

	while (true) {
		k_msgq_get(&data->evq, &ev, K_FOREVER);

		if (ev == EV_STOP) {
			cam_dma_stop();
			data->reading = false;
			if (data->vbuf) {
				k_fifo_put(&data->fifo_in, data->vbuf);
				data->vbuf = NULL;
			}
			k_msgq_purge(&data->evq);
			atomic_clear(&data->overrun);
			k_sem_give(&data->stopped);
			continue;
		}

		if (atomic_clear(&data->overrun) && data->reading) {
			LOG_WRN("Copy thread overrun, frame dropped");
			cam_abort_frame(data);
			continue;
		}

		if (ev == EV_VSYNC) {
			if (data->reading) {
				cam_end_frame(data);
			} else {
				cam_begin_frame(data);
			}
		} else if (ev == EV_EOF && data->reading) {
			if (!cam_copy_half(data) ||
			    (data->cnt == 1 &&
			     !esp32_i2s_cam_has_soi(data->vbuf->buffer, data->len))) {
				cam_abort_frame(data);
			}
		}
	}
}

static void cam_fix_size(struct video_format *fmt)
{
	/* esp32-camera's JPEG budget; the generic w*h*2 estimate would not fit PSRAM. */
	fmt->pitch = 0;
	fmt->size = fmt->width * fmt->height / 5;
}

static int cam_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct cam_config *cfg = dev->config;
	struct cam_data *data = dev->data;
	int ret;

	/* shortcut: JPEG only; YUV/RGB need other sampling modes and filters (see ll_cam.c). */
	if (fmt->pixelformat != VIDEO_PIX_FMT_JPEG) {
		return -ENOTSUP;
	}

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	cam_fix_size(fmt);
	data->fmt = *fmt;
	return 0;
}

static int cam_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct cam_config *cfg = dev->config;
	int ret = video_get_format(cfg->source_dev, fmt);

	if (ret < 0) {
		return ret;
	}
	cam_fix_size(fmt);
	return 0;
}

static int cam_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct cam_config *cfg = dev->config;

	caps->min_vbuf_count = 1;
	return video_get_caps(cfg->source_dev, caps);
}

static int cam_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	const struct cam_config *cfg = dev->config;
	struct cam_data *data = dev->data;
	int ret;

	if (enable == data->streaming) {
		return 0;
	}

	if (!enable) {
		gpio_pin_interrupt_configure_dt(&cfg->vsync, GPIO_INT_DISABLE);
		k_sem_reset(&data->stopped);
		cam_post(data, EV_STOP);
		k_sem_take(&data->stopped, K_FOREVER);
		data->streaming = false;
		return video_stream_stop(cfg->source_dev, type);
	}

	ret = video_stream_start(cfg->source_dev, type);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->vsync, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		video_stream_stop(cfg->source_dev, type);
		return ret;
	}

	data->streaming = true;
	return 0;
}

static int cam_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct cam_data *data = dev->data;

	vbuf->bytesused = 0;
	vbuf->line_offset = 0;
	k_fifo_put(&data->fifo_in, vbuf);
	return 0;
}

static int cam_dequeue(const struct device *dev, struct video_buffer **vbuf, k_timeout_t timeout)
{
	struct cam_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	return *vbuf ? 0 : -EAGAIN;
}

static int cam_flush(const struct device *dev, bool cancel)
{
	struct cam_data *data = dev->data;
	struct video_buffer *vbuf;

	if (!cancel) {
		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_sleep(K_MSEC(1));
		}
		return 0;
	}

	if (data->streaming) {
		return -EBUSY; /* the thread may own a buffer; stop the stream first */
	}
	while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT)) != NULL) {
		k_fifo_put(&data->fifo_out, vbuf);
		cam_raise(data, VIDEO_BUF_ABORTED);
	}
	return 0;
}

#ifdef CONFIG_POLL
static int cam_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct cam_data *data = dev->data;

	data->signal_out = sig;
	return 0;
}
#endif

static int cam_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct cam_config *cfg = dev->config;

	return video_set_frmival(cfg->source_dev, frmival);
}

static int cam_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct cam_config *cfg = dev->config;

	return video_get_frmival(cfg->source_dev, frmival);
}

static int cam_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	const struct cam_config *cfg = dev->config;

	return video_enum_frmival(cfg->source_dev, fie);
}

static int cam_init(const struct device *dev)
{
	const struct cam_config *cfg = dev->config;
	struct cam_data *data = dev->data;
	int ret;

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	k_msgq_init(&data->evq, data->evq_buf, sizeof(uint8_t), sizeof(data->evq_buf));
	k_sem_init(&data->stopped, 0, 1);

	if (!device_is_ready(cfg->clock_dev) || !gpio_is_ready_dt(&cfg->vsync)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}
	/* HREF gating: tie the I2S "H enable" input high. */
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, I2S0I_H_ENABLE_IDX, false);

	ret = gpio_pin_configure_dt(&cfg->vsync, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}
	gpio_init_callback(&data->vsync_cb, cam_vsync_isr, BIT(cfg->vsync.pin));
	ret = gpio_add_callback_dt(&cfg->vsync, &data->vsync_cb);
	if (ret < 0) {
		return ret;
	}

	cam_i2s_config();

	ret = esp_intr_alloc(cfg->irq_source,
			     ESP_PRIO_TO_FLAGS(cfg->irq_priority) |
				     ESP_INT_FLAGS_CHECK(cfg->irq_flags),
			     cam_i2s_isr, data, NULL);
	if (ret < 0) {
		LOG_ERR("Could not allocate I2S0 interrupt (%d)", ret);
		return ret;
	}

	k_thread_create(&cam_thread, cam_stack, K_THREAD_STACK_SIZEOF(cam_stack), cam_thread_fn,
			data, NULL, NULL, CONFIG_VIDEO_ESP32_I2S_CAM_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&cam_thread, "esp32_i2s_cam");

	return 0;
}

/* The sensor needs XCLK to answer on SCCB in its own init, which runs first. */
static int cam_xclk_init(void)
{
	if (!pwm_is_ready_dt(&cam_xclk)) {
		return -ENODEV;
	}
	return pwm_set_dt(&cam_xclk, cam_xclk.period, cam_xclk.period / 2);
}

SYS_INIT(cam_xclk_init, POST_KERNEL, CONFIG_VIDEO_ESP32_I2S_CAM_XCLK_INIT_PRIORITY);

BUILD_ASSERT(CONFIG_VIDEO_ESP32_I2S_CAM_XCLK_INIT_PRIORITY > CONFIG_PWM_INIT_PRIORITY &&
		     CONFIG_VIDEO_ESP32_I2S_CAM_XCLK_INIT_PRIORITY < CONFIG_VIDEO_INIT_PRIORITY,
	     "XCLK must start after PWM and before the sensor");

static DEVICE_API(video, cam_api) = {
	.set_format = cam_set_fmt,
	.get_format = cam_get_fmt,
	.set_stream = cam_set_stream,
	.get_caps = cam_get_caps,
	.enqueue = cam_enqueue,
	.dequeue = cam_dequeue,
	.flush = cam_flush,
	.set_frmival = cam_set_frmival,
	.get_frmival = cam_get_frmival,
	.enum_frmival = cam_enum_frmival,
#ifdef CONFIG_POLL
	.set_signal = cam_set_signal,
#endif
};

PINCTRL_DT_INST_DEFINE(0);

#define SOURCE_DEV DEVICE_DT_GET(DT_INST_PHANDLE(0, source))

static const struct cam_config cam_cfg = {
	.source_dev = SOURCE_DEV,
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.vsync = GPIO_DT_SPEC_INST_GET(0, vsync_gpios),
	.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(I2S_NODE)),
	.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(I2S_NODE, offset),
	.irq_source = DT_IRQ_BY_NAME(I2S_NODE, rx, irq),
	.irq_priority = DT_IRQ_BY_NAME(I2S_NODE, rx, priority),
	.irq_flags = DT_IRQ_BY_NAME(I2S_NODE, rx, flags),
};

static struct cam_data cam_data;

DEVICE_DT_INST_DEFINE(0, cam_init, NULL, &cam_data, &cam_cfg, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, &cam_api);

VIDEO_DEVICE_DEFINE(esp32_i2s_cam, DEVICE_DT_INST_GET(0), SOURCE_DEV);

/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * OmniVision OV3660 3 MP image sensor on a DVP bus, JPEG output only.
 *
 * Register values are from Espressif's esp32-camera component
 * (sensors/ov3660.c, MIT, and sensors/private_include/ov3660_settings.h,
 * Apache-2.0).
 */

#define DT_DRV_COMPAT ovti_ov3660

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/video/video.h>

#include "video_common.h"

LOG_MODULE_REGISTER(ov3660, CONFIG_VIDEO_LOG_LEVEL);

#define OV3660_REG8(addr)  ((uint32_t)(addr) | VIDEO_REG_ADDR16_DATA8)
#define OV3660_REG16(addr) ((uint32_t)(addr) | VIDEO_REG_ADDR16_DATA16_BE)

#define OV3660_CHIP_ID     OV3660_REG16(0x300a)
#define OV3660_CHIP_ID_VAL 0x3660

#define OV3660_SYSTEM_CTRL0 OV3660_REG8(0x3008)
#define OV3660_SW_RESET     0x82
#define OV3660_SW_PWDN      0x42
#define OV3660_SW_PWUP      0x02

#define OV3660_PLL_CTRL0  OV3660_REG8(0x303a)
#define OV3660_PLL_CTRL1  OV3660_REG8(0x303b)
#define OV3660_PLL_CTRL2  OV3660_REG8(0x303c)
#define OV3660_PLL_CTRL3  OV3660_REG8(0x303d)
#define OV3660_PCLK_RATIO OV3660_REG8(0x3824)
#define OV3660_VFIFO_0C   OV3660_REG8(0x460c)

#define OV3660_AEC_PK_MANUAL OV3660_REG8(0x3503)
#define OV3660_AEC_MANUAL    BIT(0)
#define OV3660_AGC_MANUAL    BIT(1)

#define OV3660_X_ADDR_ST  OV3660_REG16(0x3800)
#define OV3660_Y_ADDR_ST  OV3660_REG16(0x3802)
#define OV3660_X_ADDR_END OV3660_REG16(0x3804)
#define OV3660_Y_ADDR_END OV3660_REG16(0x3806)
#define OV3660_X_OUT      OV3660_REG16(0x3808)
#define OV3660_Y_OUT      OV3660_REG16(0x380a)
#define OV3660_X_TOTAL    OV3660_REG16(0x380c)
#define OV3660_Y_TOTAL    OV3660_REG16(0x380e)
#define OV3660_X_OFFSET   OV3660_REG16(0x3810)
#define OV3660_Y_OFFSET   OV3660_REG16(0x3812)
#define OV3660_X_INC      OV3660_REG8(0x3814)
#define OV3660_Y_INC      OV3660_REG8(0x3815)

#define OV3660_TIMING_TC_REG20 OV3660_REG8(0x3820)
#define OV3660_TIMING_TC_REG21 OV3660_REG8(0x3821)
#define OV3660_REG_4514        OV3660_REG8(0x4514)
#define OV3660_REG_4520        OV3660_REG8(0x4520)

#define OV3660_JPEG_QS OV3660_REG8(0x4407)

#define OV3660_ISP_CTRL00   OV3660_REG8(0x5000)
#define OV3660_ISP_CTRL01   OV3660_REG8(0x5001)
#define OV3660_ISP_AWB      BIT(0)
#define OV3660_ISP_SCALE    BIT(5)
#define OV3660_PRE_ISP_TEST OV3660_REG8(0x503d)
#define OV3660_COLOR_BAR    0xc0

#define OV3660_SDE_CONTRAST   OV3660_REG8(0x5586)
#define OV3660_SDE_BRIGHTNESS OV3660_REG8(0x5587)
#define OV3660_SDE_SIGN       OV3660_REG8(0x5588)
#define OV3660_CMX1           0x5381

/* 20 MHz XCLK: PLL multiplier, PCLK divider */
#define OV3660_JPEG_PCLK_DIV_QXGA 8
#define OV3660_JPEG_PCLK_DIV      10

struct ov3660_window {
	uint16_t max_w, max_h;
	uint16_t start_x, start_y;
	uint16_t end_x, end_y;
	uint16_t total_x, total_y;
};

static const struct ov3660_window ov3660_win_4x3 = {2048, 1536, 0, 0, 2079, 1547, 2300, 1564};
static const struct ov3660_window ov3660_win_16x9 = {1920, 1080, 64, 242, 2015, 1333, 2172, 1322};
static const struct ov3660_window ov3660_win_5x4 = {1920, 1536, 64, 0, 2015, 1547, 2172, 1564};

struct ov3660_mode {
	uint16_t width;
	uint16_t height;
	const struct ov3660_window *win;
};

static const struct ov3660_mode ov3660_modes[] = {
	{320, 240, &ov3660_win_4x3},   {640, 480, &ov3660_win_4x3},
	{800, 600, &ov3660_win_4x3},   {1024, 768, &ov3660_win_4x3},
	{1280, 720, &ov3660_win_16x9}, {1280, 1024, &ov3660_win_5x4},
	{1600, 1200, &ov3660_win_4x3}, {1920, 1080, &ov3660_win_16x9},
	{2048, 1536, &ov3660_win_4x3},
};

#define OV3660_FMT(w, h)                                                                           \
	{                                                                                          \
		.pixelformat = VIDEO_PIX_FMT_JPEG,                                                 \
		.width_min = (w),                                                                  \
		.width_max = (w),                                                                  \
		.height_min = (h),                                                                 \
		.height_max = (h),                                                                 \
	}

static const struct video_format_cap ov3660_fmts[] = {
	OV3660_FMT(320, 240),   OV3660_FMT(640, 480),
	OV3660_FMT(800, 600),   OV3660_FMT(1024, 768),
	OV3660_FMT(1280, 720),  OV3660_FMT(1280, 1024),
	OV3660_FMT(1600, 1200), OV3660_FMT(1920, 1080),
	OV3660_FMT(2048, 1536), {0},
};

BUILD_ASSERT(ARRAY_SIZE(ov3660_fmts) == ARRAY_SIZE(ov3660_modes) + 1);

static const struct video_reg16 ov3660_init_regs[] = {
	{0x3103, 0x13},
	{0x3008, 0x42},
	{0x3017, 0xff},
	{0x3018, 0xff},
	{0x302c, 0xc3},
	{0x4740, 0x21},
	{0x3611, 0x01},
	{0x3612, 0x2d},
	{0x3032, 0x00},
	{0x3614, 0x80},
	{0x3618, 0x00},
	{0x3619, 0x75},
	{0x3622, 0x80},
	{0x3623, 0x00},
	{0x3624, 0x03},
	{0x3630, 0x52},
	{0x3632, 0x07},
	{0x3633, 0xd2},
	{0x3704, 0x80},
	{0x3708, 0x66},
	{0x3709, 0x12},
	{0x370b, 0x12},
	{0x3717, 0x00},
	{0x371b, 0x60},
	{0x371c, 0x00},
	{0x3901, 0x13},
	{0x3600, 0x08},
	{0x3620, 0x43},
	{0x3702, 0x20},
	{0x3739, 0x48},
	{0x3730, 0x20},
	{0x370c, 0x0c},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3000, 0x10},
	{0x3004, 0xef},
	{0x6700, 0x05},
	{0x6701, 0x19},
	{0x6702, 0xfd},
	{0x6703, 0xd1},
	{0x6704, 0xff},
	{0x6705, 0xff},
	{0x3c01, 0x80},
	{0x3c00, 0x04},
	{0x3a08, 0x00},
	{0x3a09, 0x62},
	{0x3a0e, 0x08},
	{0x3a0a, 0x00},
	{0x3a0b, 0x52},
	{0x3a0d, 0x09},
	{0x3a00, 0x3a},
	{0x3a14, 0x09},
	{0x3a15, 0x30},
	{0x3a02, 0x09},
	{0x3a03, 0x30},
	{0x440e, 0x08},
	{0x4520, 0x0b},
	{0x460b, 0x37},
	{0x4713, 0x02},
	{0x471c, 0xd0},
	{0x5086, 0x00},
	{0x5002, 0x00},
	{0x501f, 0x00},
	{0x3008, 0x02},
	/* AWB */
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x16},
	{0x5187, 0x16},
	{0x5188, 0x16},
	{0x5189, 0x68},
	{0x518a, 0x60},
	{0x518b, 0xe0},
	{0x518c, 0xb2},
	{0x518d, 0x42},
	{0x518e, 0x35},
	{0x518f, 0x56},
	{0x5190, 0x56},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	/* Colour matrix */
	{0x5381, 0x1d},
	{0x5382, 0x60},
	{0x5383, 0x03},
	{0x5384, 0x0c},
	{0x5385, 0x78},
	{0x5386, 0x84},
	{0x5387, 0x7d},
	{0x5388, 0x6b},
	{0x5389, 0x12},
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5480, 0x01},
	{0x5000, 0xa7},
	/* Lens correction */
	{0x5800, 0x0c},
	{0x5801, 0x09},
	{0x5802, 0x0c},
	{0x5803, 0x0c},
	{0x5804, 0x0d},
	{0x5805, 0x17},
	{0x5806, 0x06},
	{0x5807, 0x05},
	{0x5808, 0x04},
	{0x5809, 0x06},
	{0x580a, 0x09},
	{0x580b, 0x0e},
	{0x580c, 0x05},
	{0x580d, 0x01},
	{0x580e, 0x01},
	{0x580f, 0x01},
	{0x5810, 0x05},
	{0x5811, 0x0d},
	{0x5812, 0x05},
	{0x5813, 0x01},
	{0x5814, 0x01},
	{0x5815, 0x01},
	{0x5816, 0x05},
	{0x5817, 0x0d},
	{0x5818, 0x08},
	{0x5819, 0x06},
	{0x581a, 0x05},
	{0x581b, 0x07},
	{0x581c, 0x0b},
	{0x581d, 0x0d},
	{0x581e, 0x12},
	{0x581f, 0x0d},
	{0x5820, 0x0e},
	{0x5821, 0x10},
	{0x5822, 0x10},
	{0x5823, 0x1e},
	{0x5824, 0x53},
	{0x5825, 0x15},
	{0x5826, 0x05},
	{0x5827, 0x14},
	{0x5828, 0x54},
	{0x5829, 0x25},
	{0x582a, 0x33},
	{0x582b, 0x33},
	{0x582c, 0x34},
	{0x582d, 0x16},
	{0x582e, 0x24},
	{0x582f, 0x41},
	{0x5830, 0x50},
	{0x5831, 0x42},
	{0x5832, 0x15},
	{0x5833, 0x25},
	{0x5834, 0x34},
	{0x5835, 0x33},
	{0x5836, 0x24},
	{0x5837, 0x26},
	{0x5838, 0x54},
	{0x5839, 0x25},
	{0x583a, 0x15},
	{0x583b, 0x25},
	{0x583c, 0x53},
	{0x583d, 0xcf},
	/* AEC target for level 0, then de-noise and sharpness */
	{0x3a0f, 0x3b},
	{0x3a10, 0x32},
	{0x3a1b, 0x3b},
	{0x3a1e, 0x32},
	{0x3a11, 0x76},
	{0x3a1f, 0x19},
	{0x5302, 0x28},
	{0x5303, 0x20},
	{0x5306, 0x1c},
	{0x5307, 0x28},
	{0x4002, 0xc5},
	{0x4003, 0x81},
	{0x4005, 0x12},
	{0x5688, 0x11},
	{0x5689, 0x11},
	{0x568a, 0x11},
	{0x568b, 0x11},
	{0x568c, 0x11},
	{0x568d, 0x11},
	{0x568e, 0x11},
	{0x568f, 0x11},
	{0x5580, 0x06},
	{0x5588, 0x00},
	{0x5583, 0x40},
	{0x5584, 0x2c},
	{0x5001, 0x83},
	/* JPEG out of the YUV422 pipeline */
	{0x501f, 0x00},
	{0x4300, 0x30},
	{0x3002, 0x00},
	{0x3006, 0xff},
	{0x471c, 0x50},
};

/* Colour matrix 0x5381..0x538b per saturation level -4..4 */
static const uint8_t ov3660_saturation[9][11] = {
	{0x1d, 0x60, 0x03, 0x07, 0x48, 0x4f, 0x4b, 0x40, 0x0b, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x08, 0x54, 0x5c, 0x58, 0x4b, 0x0d, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x0a, 0x60, 0x6a, 0x64, 0x56, 0x0e, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x0b, 0x6c, 0x77, 0x70, 0x60, 0x10, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x0c, 0x78, 0x84, 0x7d, 0x6b, 0x12, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x0d, 0x84, 0x91, 0x8a, 0x76, 0x14, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x0e, 0x90, 0x9e, 0x96, 0x80, 0x16, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x10, 0x9c, 0xac, 0xa2, 0x8b, 0x17, 0x01, 0x98},
	{0x1d, 0x60, 0x03, 0x11, 0xa8, 0xb9, 0xaf, 0x96, 0x19, 0x01, 0x98},
};

struct ov3660_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec pwdn_gpio;
};

struct ov3660_ctrls {
	struct video_ctrl hflip;
	struct video_ctrl vflip;
	struct video_ctrl ae;
	struct video_ctrl awb;
	struct video_ctrl autogain;
	struct video_ctrl brightness;
	struct video_ctrl contrast;
	struct video_ctrl saturation;
	struct video_ctrl quality;
	struct video_ctrl test_pattern;
};

struct ov3660_data {
	struct ov3660_ctrls ctrls;
	struct video_format fmt;
	bool binning;
};

static int ov3660_write_pll(const struct i2c_dt_spec *i2c, uint8_t mult, uint8_t pclk_div)
{
	/* Not bypassed, sys_div 1, pre_div 3 (/2), seld5 0, PCLK divider manual */
	const struct video_reg regs[] = {
		{OV3660_PLL_CTRL0, 0x00}, {OV3660_PLL_CTRL1, mult & 0x1f}, {OV3660_PLL_CTRL2, 0x11},
		{OV3660_PLL_CTRL3, 0x30}, {OV3660_PCLK_RATIO, pclk_div},   {OV3660_VFIFO_0C, 0x22},
	};

	return video_write_cci_multiregs(i2c, regs, ARRAY_SIZE(regs));
}

/* Timing registers that combine compression, binning, flip and mirror. */
static int ov3660_write_image_options(const struct device *dev)
{
	const struct ov3660_config *cfg = dev->config;
	struct ov3660_data *data = dev->data;
	/* 0x4514 by (binning, vflip, hmirror), from the vendor table */
	static const uint8_t reg4514[8] = {0x88, 0x88, 0xbb, 0xbb, 0xaa, 0xbb, 0xbb, 0xaa};
	uint8_t reg20 = data->binning ? 0x01 : 0x40;
	uint8_t reg21 = 0x20 | (data->binning ? 0x01 : 0x00);
	uint8_t inc = data->binning ? 0x31 : 0x11;
	uint8_t idx = 0;

	if (data->ctrls.vflip.val != 0) {
		reg20 |= 0x06;
		idx |= 1;
	}
	if (data->ctrls.hflip.val != 0) {
		reg21 |= 0x06;
		idx |= 2;
	}
	if (data->binning) {
		idx |= 4;
	}

	const struct video_reg regs[] = {
		{OV3660_TIMING_TC_REG20, reg20},
		{OV3660_TIMING_TC_REG21, reg21},
		{OV3660_REG_4514, reg4514[idx]},
		{OV3660_REG_4520, data->binning ? 0x0b : 0xb0},
		{OV3660_X_INC, inc},
		{OV3660_Y_INC, inc},
	};

	return video_write_cci_multiregs(&cfg->i2c, regs, ARRAY_SIZE(regs));
}

static int ov3660_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct ov3660_config *cfg = dev->config;
	struct ov3660_data *data = dev->data;
	const struct ov3660_window *win;
	bool scale;
	size_t i;
	int ret;

	ret = video_format_caps_index(ov3660_fmts, fmt, &i);
	if (ret < 0) {
		LOG_ERR("Unsupported format %s %ux%u", VIDEO_FOURCC_TO_STR(fmt->pixelformat),
			fmt->width, fmt->height);
		return ret;
	}

	win = ov3660_modes[i].win;
	data->binning = fmt->width <= win->max_w / 2 && fmt->height <= win->max_h / 2;
	scale = !((fmt->width == win->max_w && fmt->height == win->max_h) ||
		  (fmt->width == win->max_w / 2 && fmt->height == win->max_h / 2));

	const struct video_reg regs[] = {
		{OV3660_X_ADDR_ST, win->start_x},
		{OV3660_Y_ADDR_ST, win->start_y},
		{OV3660_X_ADDR_END, win->end_x},
		{OV3660_Y_ADDR_END, win->end_y},
		{OV3660_X_OUT, fmt->width},
		{OV3660_Y_OUT, fmt->height},
		{OV3660_X_TOTAL, win->total_x},
		{OV3660_Y_TOTAL, data->binning ? win->total_y / 2 + 1 : win->total_y},
		{OV3660_X_OFFSET, data->binning ? 8 : 16},
		{OV3660_Y_OFFSET, data->binning ? 2 : 6},
	};

	ret = video_write_cci_multiregs(&cfg->i2c, regs, ARRAY_SIZE(regs));
	if (ret < 0) {
		return ret;
	}

	ret = video_modify_cci_reg(&cfg->i2c, OV3660_ISP_CTRL01, OV3660_ISP_SCALE,
				   scale ? OV3660_ISP_SCALE : 0);
	if (ret < 0) {
		return ret;
	}

	ret = ov3660_write_image_options(dev);
	if (ret < 0) {
		return ret;
	}

	/* 40 MHz SYSCLK at QXGA, 50 MHz below; PCLK 10 MHz either way */
	if (fmt->width == 2048) {
		ret = ov3660_write_pll(&cfg->i2c, 24, OV3660_JPEG_PCLK_DIV_QXGA);
	} else {
		ret = ov3660_write_pll(&cfg->i2c, 30, OV3660_JPEG_PCLK_DIV);
	}
	if (ret < 0) {
		return ret;
	}

	data->fmt = *fmt;
	return 0;
}

static int ov3660_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct ov3660_data *data = dev->data;

	*fmt = data->fmt;
	return 0;
}

static int ov3660_get_caps(const struct device *dev, struct video_caps *caps)
{
	caps->format_caps = ov3660_fmts;
	return 0;
}

static int ov3660_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	const struct ov3660_config *cfg = dev->config;

	return video_write_cci_reg(&cfg->i2c, OV3660_SYSTEM_CTRL0,
				   enable ? OV3660_SW_PWUP : OV3660_SW_PWDN);
}

static int ov3660_set_saturation(const struct i2c_dt_spec *i2c, int level)
{
	const uint8_t *row = ov3660_saturation[level + 4];
	int ret;

	for (size_t i = 0; i < ARRAY_SIZE(ov3660_saturation[0]); i++) {
		ret = video_write_cci_reg(i2c, OV3660_REG8(OV3660_CMX1 + i), row[i]);
		if (ret < 0) {
			return ret;
		}
	}
	return 0;
}

static int ov3660_set_brightness(const struct i2c_dt_spec *i2c, int level)
{
	int ret;

	ret = video_write_cci_reg(i2c, OV3660_SDE_BRIGHTNESS, (uint32_t)abs(level) << 4);
	if (ret < 0) {
		return ret;
	}
	return video_modify_cci_reg(i2c, OV3660_SDE_SIGN, BIT(3), level < 0 ? BIT(3) : 0);
}

static int ov3660_set_ctrl(const struct device *dev, uint32_t id)
{
	const struct ov3660_config *cfg = dev->config;
	struct ov3660_ctrls *ctrls = &((struct ov3660_data *)dev->data)->ctrls;

	switch (id) {
	case VIDEO_CID_HFLIP:
	case VIDEO_CID_VFLIP:
		return ov3660_write_image_options(dev);
	case VIDEO_CID_EXPOSURE_AUTO:
		if (ctrls->ae.val != VIDEO_EXPOSURE_AUTO &&
		    ctrls->ae.val != VIDEO_EXPOSURE_MANUAL) {
			return -ENOTSUP;
		}
		return video_modify_cci_reg(
			&cfg->i2c, OV3660_AEC_PK_MANUAL, OV3660_AEC_MANUAL,
			ctrls->ae.val == VIDEO_EXPOSURE_MANUAL ? OV3660_AEC_MANUAL : 0);
	case VIDEO_CID_AUTOGAIN:
		return video_modify_cci_reg(&cfg->i2c, OV3660_AEC_PK_MANUAL, OV3660_AGC_MANUAL,
					    ctrls->autogain.val != 0 ? 0 : OV3660_AGC_MANUAL);
	case VIDEO_CID_AUTO_WHITE_BALANCE:
		return video_modify_cci_reg(&cfg->i2c, OV3660_ISP_CTRL01, OV3660_ISP_AWB,
					    ctrls->awb.val != 0 ? OV3660_ISP_AWB : 0);
	case VIDEO_CID_BRIGHTNESS:
		return ov3660_set_brightness(&cfg->i2c, ctrls->brightness.val);
	case VIDEO_CID_CONTRAST:
		return video_write_cci_reg(&cfg->i2c, OV3660_SDE_CONTRAST,
					   (ctrls->contrast.val + 4) << 3);
	case VIDEO_CID_SATURATION:
		return ov3660_set_saturation(&cfg->i2c, ctrls->saturation.val);
	case VIDEO_CID_JPEG_COMPRESSION_QUALITY:
		/* QS 2..63, lower is finer */
		return video_write_cci_reg(&cfg->i2c, OV3660_JPEG_QS,
					   CLAMP((100 - ctrls->quality.val) * 63 / 100, 2, 63));
	case VIDEO_CID_TEST_PATTERN:
		return video_modify_cci_reg(&cfg->i2c, OV3660_PRE_ISP_TEST, OV3660_COLOR_BAR,
					    ctrls->test_pattern.val != 0 ? OV3660_COLOR_BAR : 0);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(video, ov3660_driver_api) = {
	.set_format = ov3660_set_fmt,
	.get_format = ov3660_get_fmt,
	.get_caps = ov3660_get_caps,
	.set_stream = ov3660_set_stream,
	.set_ctrl = ov3660_set_ctrl,
};

static int ov3660_init_controls(const struct device *dev)
{
	struct ov3660_ctrls *ctrls = &((struct ov3660_data *)dev->data)->ctrls;
	const struct {
		struct video_ctrl *ctrl;
		uint32_t id;
		struct video_ctrl_range range;
	} bools_and_levels[] = {
		{&ctrls->hflip, VIDEO_CID_HFLIP, {.min = 0, .max = 1, .step = 1, .def = 0}},
		{&ctrls->vflip, VIDEO_CID_VFLIP, {.min = 0, .max = 1, .step = 1, .def = 0}},
		{&ctrls->awb,
		 VIDEO_CID_AUTO_WHITE_BALANCE,
		 {.min = 0, .max = 1, .step = 1, .def = 1}},
		{&ctrls->autogain, VIDEO_CID_AUTOGAIN, {.min = 0, .max = 1, .step = 1, .def = 1}},
		{&ctrls->brightness,
		 VIDEO_CID_BRIGHTNESS,
		 {.min = -3, .max = 3, .step = 1, .def = 0}},
		{&ctrls->contrast, VIDEO_CID_CONTRAST, {.min = -3, .max = 3, .step = 1, .def = 0}},
		{&ctrls->saturation,
		 VIDEO_CID_SATURATION,
		 {.min = -4, .max = 4, .step = 1, .def = 0}},
		{&ctrls->quality,
		 VIDEO_CID_JPEG_COMPRESSION_QUALITY,
		 {.min = 1, .max = 100, .step = 1, .def = 80}},
		{&ctrls->test_pattern,
		 VIDEO_CID_TEST_PATTERN,
		 {.min = 0, .max = 1, .step = 1, .def = 0}},
	};
	int ret;

	for (size_t i = 0; i < ARRAY_SIZE(bools_and_levels); i++) {
		ret = video_init_ctrl(bools_and_levels[i].ctrl, dev, bools_and_levels[i].id,
				      bools_and_levels[i].range);
		if (ret < 0) {
			return ret;
		}
	}

	return video_init_menu_ctrl(&ctrls->ae, dev, VIDEO_CID_EXPOSURE_AUTO, VIDEO_EXPOSURE_AUTO,
				    NULL);
}

static int ov3660_init(const struct device *dev)
{
	const struct ov3660_config *cfg = dev->config;
	struct video_format fmt = {
		.pixelformat = VIDEO_PIX_FMT_JPEG,
		.width = 800,
		.height = 600,
	};
	uint32_t chip_id;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	if (cfg->pwdn_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&cfg->pwdn_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	if (cfg->reset_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return ret;
		}
		k_msleep(1);
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
	}

	k_msleep(20);

	ret = video_read_cci_reg(&cfg->i2c, OV3660_CHIP_ID, &chip_id);
	if (ret < 0) {
		LOG_ERR("Unable to read chip ID: %d", ret);
		return ret;
	}
	if (chip_id != OV3660_CHIP_ID_VAL) {
		LOG_ERR("Wrong chip ID 0x%04x", chip_id);
		return -ENODEV;
	}

	ret = video_write_cci_reg(&cfg->i2c, OV3660_SYSTEM_CTRL0, OV3660_SW_RESET);
	if (ret < 0) {
		return ret;
	}
	k_msleep(100);

	ret = video_write_cci_multiregs16(&cfg->i2c, ov3660_init_regs,
					  ARRAY_SIZE(ov3660_init_regs));
	if (ret < 0) {
		LOG_ERR("Unable to initialize the sensor: %d", ret);
		return ret;
	}
	k_msleep(100);

	ret = ov3660_init_controls(dev);
	if (ret < 0) {
		return ret;
	}

	ret = ov3660_set_ctrl(dev, VIDEO_CID_JPEG_COMPRESSION_QUALITY);
	if (ret < 0) {
		return ret;
	}

	ret = ov3660_set_fmt(dev, &fmt);
	if (ret < 0) {
		return ret;
	}

	/* Stream only on request */
	return ov3660_set_stream(dev, false, VIDEO_BUF_TYPE_OUTPUT);
}

#define OV3660_DEFINE(n)                                                                           \
	static struct ov3660_data ov3660_data_##n;                                                 \
	static const struct ov3660_config ov3660_cfg_##n = {                                       \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                       \
		.pwdn_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pwdn_gpios, {0}),                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ov3660_init, NULL, &ov3660_data_##n, &ov3660_cfg_##n,             \
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &ov3660_driver_api);        \
	VIDEO_DEVICE_DEFINE(ov3660_##n, DEVICE_DT_INST_GET(n), NULL);

DT_INST_FOREACH_STATUS_OKAY(OV3660_DEFINE)

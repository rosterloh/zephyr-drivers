/*
 * Copyright (c) 2026 Richard Osterloh
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * OmniVision OV5647 MIPI CSI-2 image sensor (Raspberry Pi Camera Module v1).
 *
 * The mode register table is ported from Espressif's esp_cam_sensor component
 * (sensors/ov5647, Apache-2.0), which was verified streaming from this exact
 * sensor on an ESP32-P4 before this driver was written. The constants there are
 * built from macros; they are expanded to literals here so the table reads as
 * the register values actually written. Deviating from these numbers means
 * re-deriving the sensor's PLL, which is what the IMX219 fork in this directory
 * exists to fix - do not "tidy" them.
 */

#define DT_DRV_COMPAT ovti_ov5647

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#endif
#include <zephyr/sys/byteorder.h>
#include <zephyr/video/video.h>

#include "video_common.h"

LOG_MODULE_REGISTER(ov5647, CONFIG_VIDEO_LOG_LEVEL);

#define OV5647_REG8(addr)  ((uint32_t)(addr) | VIDEO_REG_ADDR16_DATA8)
#define OV5647_REG16(addr) ((uint32_t)(addr) | VIDEO_REG_ADDR16_DATA16_BE)
#define OV5647_REG24(addr) ((uint32_t)(addr) | VIDEO_REG_ADDR16_DATA24_BE)

/* 0x300a/0x300b read back as one 16-bit big-endian pair. */
#define OV5647_CCI_CHIP_ID OV5647_REG16(0x300a)
#define OV5647_CHIP_ID     0x5647

#define OV5647_CCI_SW_STANDBY  OV5647_REG8(0x0100)
#define OV5647_STANDBY         0x00
#define OV5647_STREAMING       0x01
#define OV5647_CCI_SW_RESET    OV5647_REG8(0x0103)
#define OV5647_CCI_MIPI_CTRL00 OV5647_REG8(0x4800)

/*
 * MIPI_CTRL00 bits.
 *
 * CLOCK_LANE_GATE puts the clock lane in non-continuous mode. Espressif's
 * driver builds a value that includes it, then discards that value and writes a
 * literal 0x14 (BUS_IDLE | LINE_SYNC_ENABLE) - the computed `val` is dead code.
 * Writing the gated value instead is not a harmless tidy-up: with BIT(5) set,
 * this driver enumerated correctly and the sensor acknowledged every register
 * write, but no frame ever reached the receiver's DMA. The ESP32-P4 CSI host
 * here needs the continuous clock.
 */
#define OV5647_MIPI_CTRL00_CLOCK_LANE_GATE  BIT(5)
#define OV5647_MIPI_CTRL00_LINE_SYNC_ENABLE BIT(4)
#define OV5647_MIPI_CTRL00_BUS_IDLE         BIT(2)
#define OV5647_MIPI_CTRL00_CLOCK_LANE_DIS   BIT(0)

/*
 * BUS_IDLE only - deliberately NOT Espressif's 0x14.
 *
 * LINE_SYNC_ENABLE makes the sensor emit CSI-2 line start/end short packets.
 * Espressif enables them because esp_video tells its ISP to expect them. The
 * ESP32-P4 receiver in this tree configures the ISP with
 * isp_ll_enable_line_start/end_packet_exist(false), matching the IMX219 it was
 * brought up against, so enabling them here would put the two ends out of step.
 *
 * Measured: frames arrive complete either way (800/800 lines, 640000/640000
 * bytes), so this is a consistency choice rather than a fix for an observed
 * fault. Keep the sensor silent on line sync unless the receiver learns to ask.
 */
#define OV5647_MIPI_CTRL00_STREAMING (OV5647_MIPI_CTRL00_BUS_IDLE)

/* 20-bit exposure in 1/16 line units, spread over 0x3500..0x3502. */
#define OV5647_CCI_EXPOSURE      OV5647_REG24(0x3500)
#define OV5647_EXPOSURE_MAX      0xfffff
#define OV5647_EXPOSURE_DEFAULT  0x000800
/* 10-bit analogue gain, 0x350a..0x350b; 0x80 = 1x. */
#define OV5647_CCI_ANALOGUE_GAIN OV5647_REG16(0x350a)
#define OV5647_GAIN_MAX          0x3ff
/*
 * 2x analogue gain, paired with the exposure above. Tuned on hardware against a
 * lit indoor scene, measuring the mean of the RAW8 frame:
 *
 *   sensor default (AEC left auto, nothing written)   mean   6  - looks dead
 *   exposure 0x3000 + gain 0x200 (4x)                 mean 231  - clipped
 *   exposure 0x3000 + gain 0x100 (2x)                 mean 223  - still clipped
 *   exposure 0x0800 + gain 0x100 (2x)                 mean  83  - full range
 *
 * Note the middle two: halving the gain barely moved the mean, because the
 * exposure was the term that was clipping. Reach for exposure first.
 *
 * This is a starting point for an average indoor scene, not a calibration. A
 * real deployment should run an AE loop over VIDEO_CID_EXPOSURE and
 * VIDEO_CID_ANALOGUE_GAIN, both of which this driver exposes.
 */
#define OV5647_GAIN_DEFAULT      0x0100
#define OV5647_CCI_TESTPATTERN   OV5647_REG8(0x503d)
/* 0x3503: bit0 = manual AGC, bit1 = manual AEC. */
#define OV5647_CCI_AEC_MANUAL    OV5647_REG8(0x3503)

/*
 * 800x800 RAW8 at 50 fps, 2 lanes, 24 MHz input clock: 400 Mbps per lane.
 * VIDEO_CID_LINK_FREQ is the D-PHY clock, which is half the bit rate because
 * CSI-2 D-PHY is DDR - video_esp32_csi.c doubles it back to program the HAL.
 * Getting this wrong programs the receiver's PHY for the wrong range and the
 * link goes silent with no error anywhere.
 */
#define OV5647_LANE_BIT_RATE_RAW8 400000000 /* IDI 100 MHz * 4 */

enum {
	OV5647_LINK_FREQ_RAW8_IDX,
};

static const int64_t ov5647_link_frequency[] = {
	[OV5647_LINK_FREQ_RAW8_IDX] = OV5647_LANE_BIT_RATE_RAW8 / 2,
};

struct ov5647_ctrls {
	struct video_ctrl exposure;
	struct video_ctrl analogue_gain;
	struct video_ctrl linkfreq;
	struct video_ctrl test_pattern;
};

struct ov5647_data {
	struct ov5647_ctrls ctrls;
	struct video_format fmt;
	uint8_t mode_idx;
};

struct ov5647_config {
	struct i2c_dt_spec i2c;
	uint32_t input_clk_hz;
};

/*
 * Put the sensor in a known state and hold the clock lane in LP-11 so the
 * receiver's D-PHY can see an idle link before streaming starts.
 */
static const struct video_reg ov5647_reset_regs[] = {
	{OV5647_CCI_SW_STANDBY, OV5647_STANDBY},
	{OV5647_CCI_SW_RESET, 0x01},
};

/*
 * MIPI_2lane_24Minput_RAW8_800x800_50fps, expanded from
 * ov5647_mipi_2lane_24Minput_800x800_raw8_50fps[]:
 *   0x3034 = OV5647_8BIT_MODE                                      -> 0x18
 *   0x3036 = (100000000 * 8 * 4) / 25000000                        -> 0x80
 *   0x4837 = (1000000000 / 100000000) * 2                          -> 0x14
 * Crop window is x 500..2623, y 0..1953 scaled 2x2 to an 800x800 output, which
 * is why 0x3814/0x3815 are 0x31 (odd/even pixel increment) and the Bayer order
 * comes out GBRG rather than the sensor's native BGGR.
 */
/* clang-format off */
static const struct video_reg ov5647_mode_800x800_raw8[] = {
	{OV5647_REG8(0x3034), 0x18}, /* RAW8 output */
	{OV5647_REG8(0x3035), 0x41}, /* system clock divider */
	{OV5647_REG8(0x3036), 0x80}, /* PLL multiplier */
	{OV5647_REG8(0x303c), 0x11},
	{OV5647_REG8(0x3106), 0xf5},
	{OV5647_REG8(0x3821), 0x03},
	{OV5647_REG8(0x3820), 0x41},
	{OV5647_REG8(0x3827), 0xec},
	{OV5647_REG8(0x370c), 0x0f},
	{OV5647_REG8(0x3612), 0x59},
	{OV5647_REG8(0x3618), 0x00},
	{OV5647_REG8(0x5000), 0xff},
	{OV5647_REG8(0x583e), 0xf0}, /* lens shading max gain */
	{OV5647_REG8(0x583f), 0x20}, /* lens shading min gain */
	{OV5647_REG8(0x5002), 0x41},
	{OV5647_REG8(0x5003), 0x08},
	{OV5647_REG8(0x5a00), 0x08},
	{OV5647_REG8(0x3000), 0x00},
	{OV5647_REG8(0x3001), 0x00},
	{OV5647_REG8(0x3002), 0x00},
	{OV5647_REG8(0x3016), 0x08},
	{OV5647_REG8(0x3017), 0xe0},
	{OV5647_REG8(0x3018), 0x44},
	{OV5647_REG8(0x301c), 0xf8},
	{OV5647_REG8(0x301d), 0xf0},
	{OV5647_REG8(0x3a18), 0x00},
	{OV5647_REG8(0x3a19), 0xf8},
	{OV5647_REG8(0x3c01), 0x80},
	{OV5647_REG8(0x3c00), 0x40},
	{OV5647_REG8(0x3b07), 0x0c},
	{OV5647_REG16(0x380c), 0x0768}, /* HTS = 1896 */
	{OV5647_REG16(0x380e), 0x03d8}, /* VTS = 984 */
	{OV5647_REG8(0x3814), 0x31}, /* horizontal 2x subsample */
	{OV5647_REG8(0x3815), 0x31}, /* vertical 2x subsample */
	{OV5647_REG8(0x3708), 0x64},
	{OV5647_REG8(0x3709), 0x52},
	{OV5647_REG16(0x3800), 0x01f4}, /* X start 500 */
	{OV5647_REG16(0x3802), 0x0000}, /* Y start 0 */
	{OV5647_REG16(0x3804), 0x0a3f}, /* X end 2623 */
	{OV5647_REG16(0x3806), 0x07a1}, /* Y end 1953 */
	{OV5647_REG16(0x3808), 0x0320}, /* output width 800 */
	/* 0x380a output height is written per mode from ov5647_set_fmt(). */
	{OV5647_REG16(0x3810), 0x0008}, /* H offset 8 */
	{OV5647_REG16(0x3812), 0x0000}, /* V offset 0 */
	{OV5647_REG8(0x3630), 0x2e},
	{OV5647_REG8(0x3632), 0xe2},
	{OV5647_REG8(0x3633), 0x23},
	{OV5647_REG8(0x3634), 0x44},
	{OV5647_REG8(0x3636), 0x06},
	{OV5647_REG8(0x3620), 0x64},
	{OV5647_REG8(0x3621), 0xe0},
	{OV5647_REG8(0x3600), 0x37},
	{OV5647_REG8(0x3704), 0xa0},
	{OV5647_REG8(0x3703), 0x5a},
	{OV5647_REG8(0x3715), 0x78},
	{OV5647_REG8(0x3717), 0x01},
	{OV5647_REG8(0x3731), 0x02},
	{OV5647_REG8(0x370b), 0x60},
	{OV5647_REG8(0x3705), 0x1a},
	{OV5647_REG8(0x3f05), 0x02},
	{OV5647_REG8(0x3f06), 0x10},
	{OV5647_REG8(0x3f01), 0x0a},
	{OV5647_REG8(0x3a08), 0x01},
	{OV5647_REG8(0x3a09), 0x27},
	{OV5647_REG8(0x3a0a), 0x00},
	{OV5647_REG8(0x3a0b), 0xf6},
	{OV5647_REG8(0x3a0d), 0x04},
	{OV5647_REG8(0x3a0e), 0x03},
	{OV5647_REG8(0x3a0f), 0x58},
	{OV5647_REG8(0x3a10), 0x50},
	{OV5647_REG8(0x3a1b), 0x58},
	{OV5647_REG8(0x3a1e), 0x50},
	{OV5647_REG8(0x3a11), 0x60},
	{OV5647_REG8(0x3a1f), 0x28},
	{OV5647_REG8(0x4001), 0x02},
	{OV5647_REG8(0x4004), 0x02},
	{OV5647_REG8(0x4000), 0x09},
	{OV5647_REG8(0x4837), 0x14}, /* MIPI pclk period */
	{OV5647_REG8(0x4050), 0x6e},
	{OV5647_REG8(0x4051), 0x8f},
};
/* clang-format on */

/*
 * RAW8 only, and deliberately so. A 1920x1080 RAW10 mode was implemented from
 * Espressif's table and tested on this hardware: the sensor transmits, the CSI
 * host reports no errors, but the bridge latches vadr_num_gt_real ("reg_vadr_num
 * is greater than real") and never completes a frame - rows arrive short through
 * the ISP's bypass path. Per-mode link frequency, per-mode MIPI_CTRL00 line-sync
 * and the bridge byte-count fix were all tried and none of them changed it.
 * Advertising a format cap the driver cannot deliver is worse than not offering
 * it: an application picking the highest bit depth would get silence. It was
 * removed rather than shipped broken; see the git history for the table.
 *
 * Only the modes below are implemented, because they are the ones verified on
 * hardware. esp_cam_sensor also ships 800x640 / 800x1280 RAW8 (same PLL, only
 * the crop, HTS/VTS and output-size registers differ) and 1280x960 / 1920x1080
 * RAW10 (different PLL and link rate). Adding one is a new table plus an entry
 * in ov5647_fmts[] and ov5647_modes[].
 *
 * The 2x2 subsampled readout makes the Bayer phase GBRG, not the sensor's
 * native BGGR - esp_cam_sensor reports ESP_CAM_SENSOR_BAYER_GBRG for every
 * mode. Getting this wrong swaps red and blue in whatever debayers the frame.
 */
static const struct video_format_cap ov5647_fmts[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_SGBRG8,
		.width_min = 800,
		.width_max = 800,
		.width_step = 0,
		.height_min = 640,
		.height_max = 640,
		.height_step = 0,
	},
	{
		.pixelformat = VIDEO_PIX_FMT_SGBRG8,
		.width_min = 800,
		.width_max = 800,
		.width_step = 0,
		.height_min = 800,
		.height_max = 800,
		.height_step = 0,
	},
	{0},
};

/*
 * 800x640 and 800x800 share one register table. Espressif ships them as two
 * files, but the only difference between
 * ov5647_mipi_2lane_24Minput_800x640_raw8_50fps[] and the 800x800 table is the
 * output height at 0x380a - identical HTS (1896), VTS (984), crop window
 * (x 500..2623, y 0..1953) and 2x2 subsample. Writing the height from the
 * requested format keeps them as one table instead of two near-copies.
 */
static const struct {
	const struct video_reg *regs;
	size_t num_regs;
	uint32_t fps;
	uint8_t link_freq_idx;
	/* MIPI_CTRL00 while streaming; must agree with what the receiver tells
	 * the ISP about line-sync short packets.
	 */
	uint8_t mipi_ctrl00;
} ov5647_modes[] = {
	{ov5647_mode_800x800_raw8, ARRAY_SIZE(ov5647_mode_800x800_raw8), 50,
	 OV5647_LINK_FREQ_RAW8_IDX, OV5647_MIPI_CTRL00_BUS_IDLE},
	{ov5647_mode_800x800_raw8, ARRAY_SIZE(ov5647_mode_800x800_raw8), 50,
	 OV5647_LINK_FREQ_RAW8_IDX, OV5647_MIPI_CTRL00_BUS_IDLE},
};

static int ov5647_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct ov5647_config *cfg = dev->config;
	struct ov5647_data *drv_data = dev->data;
	size_t idx;
	int ret;

	ret = video_format_caps_index(ov5647_fmts, fmt, &idx);
	if (ret < 0) {
		LOG_ERR("Format '%s' %ux%u not supported", VIDEO_FOURCC_TO_STR(fmt->pixelformat),
			fmt->width, fmt->height);
		return -ENOTSUP;
	}

	ret = video_write_cci_multiregs(&cfg->i2c, ov5647_modes[idx].regs,
					ov5647_modes[idx].num_regs);
	if (ret < 0) {
		return ret;
	}

	/* Output height (0x380a/0x380b); see the comment on ov5647_modes[]. */
	ret = video_write_cci_reg(&cfg->i2c, OV5647_REG16(0x380a), fmt->height);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Each mode family runs its own lane rate, and the receiver programs its
	 * D-PHY from VIDEO_CID_LINK_FREQ. Point the read-only menu control at
	 * this mode's entry, or the PHY is configured for the wrong range and the
	 * link goes silent with no error reported.
	 */
	drv_data->ctrls.linkfreq.val = ov5647_modes[idx].link_freq_idx;
	drv_data->mode_idx = idx;

	/*
	 * The receiver takes the frame stride from the source's format, so the
	 * sensor has to fill it in - video_set_format() does not. Leaving it at
	 * zero makes video_esp32_csi.c compute a zero-byte frame and refuse to
	 * arm its DMA ("Frame size 0 not a multiple of 8 bytes"), which looks
	 * like a receiver fault rather than a missing field here.
	 */
	fmt->pitch = fmt->width * video_bits_per_pixel(fmt->pixelformat) / BITS_PER_BYTE;

	drv_data->fmt = *fmt;

	return 0;
}

static int ov5647_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct ov5647_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

static int ov5647_get_caps(const struct device *dev, struct video_caps *caps)
{
	if (caps->type != VIDEO_BUF_TYPE_OUTPUT) {
		LOG_ERR("Only output buffers supported");
		return -EINVAL;
	}

	caps->format_caps = ov5647_fmts;

	return 0;
}

static int ov5647_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(ov5647_modes)) {
		return -EINVAL;
	}

	fie->type = VIDEO_FRMIVAL_TYPE_DISCRETE;
	fie->discrete.numerator = 1;
	fie->discrete.denominator = ov5647_modes[fie->index].fps;

	return 0;
}

/*
 * The frame rate is fixed by the mode's PLL, HTS and VTS, so there is nothing
 * to program: report what the current mode actually runs at. Accepting a
 * request and silently ignoring it would be worse than refusing, but refusing
 * outright breaks callers that set a frame interval before streaming.
 */
static int ov5647_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	frmival->numerator = 1;
	frmival->denominator = ov5647_modes[0].fps;

	return 0;
}

static int ov5647_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	return ov5647_get_frmival(dev, frmival);
}

static int ov5647_set_stream(const struct device *dev, bool on, enum video_buf_type type)
{
	const struct ov5647_config *cfg = dev->config;
	struct ov5647_data *drv_data = dev->data;
	int ret;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		LOG_ERR("Only output buffers supported");
		return -EINVAL;
	}

	ret = video_write_cci_reg(&cfg->i2c, OV5647_CCI_MIPI_CTRL00,
				  ov5647_modes[drv_data->mode_idx].mipi_ctrl00);
	if (ret < 0) {
		return ret;
	}

	return video_write_cci_reg(&cfg->i2c, OV5647_CCI_SW_STANDBY,
				   on ? OV5647_STREAMING : OV5647_STANDBY);
}

static int ov5647_set_ctrl(const struct device *dev, unsigned int cid)
{
	const struct ov5647_config *cfg = dev->config;
	struct ov5647_data *drv_data = dev->data;
	struct ov5647_ctrls *ctrls = &drv_data->ctrls;

	switch (cid) {
	case VIDEO_CID_EXPOSURE:
		return video_write_cci_reg(&cfg->i2c, OV5647_CCI_EXPOSURE, ctrls->exposure.val);
	case VIDEO_CID_ANALOGUE_GAIN:
		return video_write_cci_reg(&cfg->i2c, OV5647_CCI_ANALOGUE_GAIN,
					   ctrls->analogue_gain.val);
	case VIDEO_CID_TEST_PATTERN:
		/*
		 * 0x503d bit 7 enables the generator; the pattern index sits in
		 * the low bits, so "off" is 0 and each menu entry n maps to
		 * 0x80 | (n - 1).
		 */
		return video_write_cci_reg(
			&cfg->i2c, OV5647_CCI_TESTPATTERN,
			ctrls->test_pattern.val == 0 ? 0x00 : 0x80 | (ctrls->test_pattern.val - 1));
	default:
		LOG_WRN("Control 0x%x not supported", cid);
		return -ENOTSUP;
	}
}

static DEVICE_API(video, ov5647_driver_api) = {
	.set_stream = ov5647_set_stream,
	.set_ctrl = ov5647_set_ctrl,
	.set_format = ov5647_set_fmt,
	.get_format = ov5647_get_fmt,
	.get_caps = ov5647_get_caps,
	.set_frmival = ov5647_set_frmival,
	.get_frmival = ov5647_get_frmival,
	.enum_frmival = ov5647_enum_frmival,
};

static const char *const ov5647_test_pattern_menu[] = {
	"Off", "Colour bars", "Colour squares", "Random data", NULL,
};

static int ov5647_init_ctrls(const struct device *dev)
{
	struct ov5647_data *drv_data = dev->data;
	struct ov5647_ctrls *ctrls = &drv_data->ctrls;
	int ret;

	ret = video_init_ctrl(&ctrls->exposure, dev, VIDEO_CID_EXPOSURE,
			      (struct video_ctrl_range){.min = 0,
							.max = OV5647_EXPOSURE_MAX,
							.step = 1,
							.def = OV5647_EXPOSURE_DEFAULT});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(
		&ctrls->analogue_gain, dev, VIDEO_CID_ANALOGUE_GAIN,
		(struct video_ctrl_range){
			.min = 0, .max = OV5647_GAIN_MAX, .step = 1, .def = OV5647_GAIN_DEFAULT});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_int_menu_ctrl(&ctrls->linkfreq, dev, VIDEO_CID_LINK_FREQ, 0,
				       ov5647_link_frequency, ARRAY_SIZE(ov5647_link_frequency));
	if (ret < 0) {
		return ret;
	}

	ctrls->linkfreq.flags |= VIDEO_CTRL_FLAG_READ_ONLY;

	return video_init_menu_ctrl(&ctrls->test_pattern, dev, VIDEO_CID_TEST_PATTERN, 0,
				    ov5647_test_pattern_menu);
}

static int ov5647_init(const struct device *dev)
{
	const struct ov5647_config *cfg = dev->config;
	struct video_format fmt = {
		.width = ov5647_fmts[0].width_min,
		.height = ov5647_fmts[0].height_min,
		.pixelformat = ov5647_fmts[0].pixelformat,
		.type = VIDEO_BUF_TYPE_OUTPUT,
	};
	uint32_t chip_id;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus %s is not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	/*
	 * The register tables assume a 24 MHz input clock; every PLL constant
	 * above is derived from it. A module with a different oscillator would
	 * stream at the wrong line rate and the receiver would see nothing, so
	 * refuse rather than produce a silent link.
	 */
	if (cfg->input_clk_hz != MHZ(24)) {
		LOG_ERR("Unsupported input clock %u Hz, this driver's modes need 24 MHz",
			cfg->input_clk_hz);
		return -ENOTSUP;
	}

	k_sleep(K_MSEC(1));

	ret = video_write_cci_multiregs(&cfg->i2c, ov5647_reset_regs,
					ARRAY_SIZE(ov5647_reset_regs));
	if (ret < 0) {
		LOG_ERR("Failed to reset sensor (%d)", ret);
		return ret;
	}

	k_sleep(K_MSEC(10));

	/* Hold the clock lane in LP-11 until the receiver is armed. */
	ret = video_write_cci_reg(&cfg->i2c, OV5647_CCI_MIPI_CTRL00,
				  OV5647_MIPI_CTRL00_CLOCK_LANE_DIS);
	if (ret < 0) {
		return ret;
	}

	ret = video_read_cci_reg(&cfg->i2c, OV5647_CCI_CHIP_ID, &chip_id);
	if (ret < 0) {
		LOG_ERR("Failed to read chip ID (%d)", ret);
		return ret;
	}

	if (chip_id != OV5647_CHIP_ID) {
		LOG_ERR("Wrong chip ID 0x%04x instead of 0x%04x", chip_id, OV5647_CHIP_ID);
		return -ENODEV;
	}

	ret = ov5647_set_fmt(dev, &fmt);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Take manual control of exposure and gain, and apply the defaults.
	 *
	 * Zephyr has no image-processing agent: nothing here runs an auto-
	 * exposure loop the way ESP-IDF's esp_ipa does, and video_init_ctrl()
	 * only registers a default, it never writes one to the sensor. Left
	 * alone the OV5647 comes up with a very short integration time - frames
	 * arrive complete but peak around 10/255, which reads as a dead sensor
	 * rather than an unexposed one.
	 *
	 * 0x3503 bit 0 = manual AGC, bit 1 = manual AEC. An application that
	 * wants auto-exposure should drive VIDEO_CID_EXPOSURE and
	 * VIDEO_CID_ANALOGUE_GAIN from its own loop; both are exposed.
	 */
	ret = video_write_cci_reg(&cfg->i2c, OV5647_CCI_AEC_MANUAL, 0x03);
	if (ret < 0) {
		return ret;
	}

	ret = video_write_cci_reg(&cfg->i2c, OV5647_CCI_EXPOSURE, OV5647_EXPOSURE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = video_write_cci_reg(&cfg->i2c, OV5647_CCI_ANALOGUE_GAIN, OV5647_GAIN_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	return ov5647_init_ctrls(dev);
}

#if defined(CONFIG_SHELL)
/*
 * Read back what the sensor actually holds, to separate "the sensor is not
 * transmitting" from "the receiver cannot see it". Everything else in CSI
 * bring-up is inferred; these are the sensor's own registers. Mirrors the
 * imx219_regs command next door.
 */
static int cmd_ov5647_regs(const struct shell *sh, size_t argc, char **argv)
{
	static const struct i2c_dt_spec i2c = I2C_DT_SPEC_GET(DT_NODELABEL(ov5647));
	static const struct {
		const char *name;
		uint32_t reg;
		const char *note;
	} regs[] = {
		{"CHIP_ID", OV5647_CCI_CHIP_ID, "expect 0x5647"},
		{"SW_STANDBY", OV5647_CCI_SW_STANDBY, "1 = streaming, 0 = standby"},
		{"MIPI_CTRL00", OV5647_CCI_MIPI_CTRL00, "expect 0x14 while streaming"},
		{"PAD_OUT", OV5647_REG8(0x300d), "0 = MIPI output enabled"},
		{"FRAME_OFF_NUMBER", OV5647_REG8(0x4202), "0 = frames emitted, 0x0f = blanked"},
		{"FORMAT_CTRL", OV5647_REG8(0x3034), "expect 0x18 for RAW8"},
		{"SC_PLL_CTRL1", OV5647_REG8(0x3035), "expect 0x41"},
		{"SC_PLL_CTRL2", OV5647_REG8(0x3036), "expect 0x80"},
		{"X_OUTPUT_SIZE", OV5647_REG16(0x3808), "expect 800"},
		{"Y_OUTPUT_SIZE", OV5647_REG16(0x380a), "expect 800"},
		{"HTS", OV5647_REG16(0x380c), "expect 1896"},
		{"VTS", OV5647_REG16(0x380e), "expect 984"},
		{"MIPI_PCLK_PERIOD", OV5647_REG8(0x4837), "expect 0x14"},
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
		uint32_t val = 0;
		int ret = video_read_cci_reg(&i2c, regs[i].reg, &val);

		if (ret < 0) {
			shell_error(sh, "%-18s read failed (%d)", regs[i].name, ret);
		} else {
			shell_print(sh, "%-18s = 0x%04x (%u)   %s", regs[i].name, val, val,
				    regs[i].note);
		}
	}

	return 0;
}
SHELL_CMD_REGISTER(ov5647_regs, NULL, "Read back OV5647 configuration over I2C", cmd_ov5647_regs);
#endif /* CONFIG_SHELL */

#define OV5647_INIT(n)                                                                             \
	static struct ov5647_data ov5647_data_##n;                                                 \
                                                                                                   \
	static const struct ov5647_config ov5647_cfg_##n = {                                       \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.input_clk_hz = DT_INST_PROP_BY_PHANDLE(n, clocks, clock_frequency),               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &ov5647_init, NULL, &ov5647_data_##n, &ov5647_cfg_##n,            \
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &ov5647_driver_api);        \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(ov5647_##n, DEVICE_DT_INST_GET(n), NULL);

DT_INST_FOREACH_STATUS_OKAY(OV5647_INIT)

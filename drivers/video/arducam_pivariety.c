/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Arducam Pivariety camera driver.
 *
 * Pivariety modules are not bare sensors. An on-module MCU exposes a small
 * self-describing register interface, so one driver serves the whole family
 * without per-sensor register tables: formats, resolutions and controls are
 * enumerated at init by writing an index register and reading back attribute
 * registers until the NO_DATA_AVAILABLE sentinel.
 *
 * Reference: drivers/media/i2c/arducam-pivariety.{c,h} in the Raspberry Pi
 * kernel. Frames are delivered raw; depth or Bayer processing is the
 * application's problem.
 */

#define DT_DRV_COMPAT arducam_pivariety

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/video/video.h>

#include "video_common.h"

LOG_MODULE_REGISTER(arducam_pivariety, CONFIG_VIDEO_LOG_LEVEL);

#define DEVICE_REG_BASE    0x0100
#define PIXFORMAT_REG_BASE 0x0200
#define FORMAT_REG_BASE    0x0300
#define CTRL_REG_BASE      0x0400

#define MODE_SELECT_REG    (DEVICE_REG_BASE | 0x0000)
#define DEVICE_VERSION_REG (DEVICE_REG_BASE | 0x0001)
#define SENSOR_ID_REG      (DEVICE_REG_BASE | 0x0002)
#define DEVICE_ID_REG      (DEVICE_REG_BASE | 0x0003)
#define SYSTEM_IDLE_REG    (DEVICE_REG_BASE | 0x0007)

#define PIXFORMAT_INDEX_REG (PIXFORMAT_REG_BASE | 0x0000)
#define PIXFORMAT_TYPE_REG  (PIXFORMAT_REG_BASE | 0x0001)
#define PIXFORMAT_ORDER_REG (PIXFORMAT_REG_BASE | 0x0002)
#define MIPI_LANES_REG      (PIXFORMAT_REG_BASE | 0x0003)

#define RESOLUTION_INDEX_REG (FORMAT_REG_BASE | 0x0000)
#define FORMAT_WIDTH_REG     (FORMAT_REG_BASE | 0x0001)
#define FORMAT_HEIGHT_REG    (FORMAT_REG_BASE | 0x0002)

#define CTRL_INDEX_REG (CTRL_REG_BASE | 0x0000)
#define CTRL_ID_REG    (CTRL_REG_BASE | 0x0001)
#define CTRL_MIN_REG   (CTRL_REG_BASE | 0x0002)
#define CTRL_MAX_REG   (CTRL_REG_BASE | 0x0003)
#define CTRL_STEP_REG  (CTRL_REG_BASE | 0x0004)
#define CTRL_DEF_REG   (CTRL_REG_BASE | 0x0005)
#define CTRL_VALUE_REG (CTRL_REG_BASE | 0x0006)

#define PIV_MAX_GENERIC_CTRLS 8

#define NO_DATA_AVAILABLE 0xFFFFFFFEU

#define PIV_MODE_STANDBY   0x00
#define PIV_MODE_STREAMING 0x01

/* The kernel driver spins on SYSTEM_IDLE_REG without a bound. A wedged module
 * must return an error here rather than hanging whichever thread called us.
 */
#define PIV_IDLE_TIMEOUT_MS 100

struct pivariety_config {
	struct i2c_dt_spec i2c;
	int64_t link_freq_hz;
};

struct piv_cap_index {
	uint8_t fmt_idx;
	uint8_t res_idx;
};

struct pivariety_data {
	struct video_format fmt;
	struct video_format_cap caps[CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS + 1];
	struct piv_cap_index cap_index[CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS];
	uint8_t num_caps;

	struct video_ctrl linkfreq;
	struct video_ctrl generic[PIV_MAX_GENERIC_CTRLS];
	uint32_t generic_id[PIV_MAX_GENERIC_CTRLS];
	uint8_t num_generic;
	int64_t link_freq_menu[1];
};

/*
 * Pivariety is not a CCI device. One 16-bit address selects one 32-bit value,
 * transferred whole: a 6-byte write, or a 2-byte address write followed by a
 * 4-byte read.
 *
 * Zephyr's video_{read,write}_cci_reg() cannot express this. They implement
 * genuine CCI semantics, where a 32-bit value occupies four consecutive
 * byte-addressed registers, so reading SYSTEM_IDLE_REG through them would
 * issue four transactions against 0x0107..0x010a. Frame the transfer directly
 * instead, mirroring pivariety_{read,write}_reg() in the kernel driver.
 */
static int piv_read(const struct device *dev, uint16_t reg, uint32_t *val)
{
	const struct pivariety_config *cfg = dev->config;
	uint8_t addr[2];
	uint8_t buf[4];
	int ret;

	sys_put_be16(reg, addr);

	ret = i2c_write_read_dt(&cfg->i2c, addr, sizeof(addr), buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	*val = sys_get_be32(buf);

	return 0;
}

static int piv_write(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct pivariety_config *cfg = dev->config;
	uint8_t buf[6];

	sys_put_be16(reg, &buf[0]);
	sys_put_be32(val, &buf[2]);

	return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

/* The module is single-threaded internally: it must report idle before the next
 * command, or indices and attributes get out of step.
 */
static int piv_wait_idle(const struct device *dev)
{
	int64_t deadline = k_uptime_get() + PIV_IDLE_TIMEOUT_MS;
	uint32_t idle = 0;

	do {
		int ret = piv_read(dev, SYSTEM_IDLE_REG, &idle);

		if (ret < 0) {
			return ret;
		}
		if (idle == 0) {
			return 0;
		}
		k_msleep(1);
	} while (k_uptime_get() < deadline);

	LOG_ERR("Module never reported idle");
	return -ETIMEDOUT;
}

/* PIXFORMAT_TYPE_REG returns the MIPI CSI-2 data type directly, so the fourcc
 * follows from the (type, order) pair. Order 4 means greyscale, for sensors
 * whose output is not a Bayer mosaic.
 */
#define PIV_DT_RAW8  0x2a
#define PIV_DT_RAW10 0x2b
#define PIV_DT_RAW12 0x2c

#define PIV_ORDER_BGGR 0
#define PIV_ORDER_GBRG 1
#define PIV_ORDER_GRBG 2
#define PIV_ORDER_RGGB 3
#define PIV_ORDER_GRAY 4

static uint32_t piv_fourcc(uint32_t data_type, uint32_t order)
{
	static const uint32_t bayer[3][4] = {
		{VIDEO_PIX_FMT_SBGGR8, VIDEO_PIX_FMT_SGBRG8, VIDEO_PIX_FMT_SGRBG8,
		 VIDEO_PIX_FMT_SRGGB8},
		{VIDEO_PIX_FMT_SBGGR10P, VIDEO_PIX_FMT_SGBRG10P, VIDEO_PIX_FMT_SGRBG10P,
		 VIDEO_PIX_FMT_SRGGB10P},
		{VIDEO_PIX_FMT_SBGGR12P, VIDEO_PIX_FMT_SGBRG12P, VIDEO_PIX_FMT_SGRBG12P,
		 VIDEO_PIX_FMT_SRGGB12P},
	};
	static const uint32_t grey[3] = {VIDEO_PIX_FMT_GREY, VIDEO_PIX_FMT_Y10P,
					 VIDEO_PIX_FMT_Y12P};
	uint32_t depth;

	switch (data_type) {
	case PIV_DT_RAW8:
		depth = 0;
		break;
	case PIV_DT_RAW10:
		depth = 1;
		break;
	case PIV_DT_RAW12:
		depth = 2;
		break;
	default:
		return 0;
	}

	if (order == PIV_ORDER_GRAY) {
		return grey[depth];
	}
	if (order > PIV_ORDER_RGGB) {
		return 0;
	}

	return bayer[depth][order];
}

/* Walk formats, and each format's resolutions, into a flat caps array. One
 * video_format_cap entry is needed per (format, resolution) pair because each
 * entry carries its own width/height bounds; cap_index remembers which module
 * indices produced each entry so set_fmt can select it again.
 */
static int piv_enumerate(const struct device *dev)
{
	struct pivariety_data *data = dev->data;
	int ret;

	for (uint32_t f = 0; f < UINT8_MAX; f++) {
		uint32_t type = 0, order = 0, fourcc;

		ret = piv_wait_idle(dev);
		if (ret < 0) {
			return ret;
		}

		ret = piv_write(dev, PIXFORMAT_INDEX_REG, f);
		if (ret < 0) {
			return ret;
		}

		ret = piv_read(dev, PIXFORMAT_TYPE_REG, &type);
		if (ret < 0) {
			return ret;
		}

		if (type == NO_DATA_AVAILABLE) {
			break;
		}

		ret = piv_read(dev, PIXFORMAT_ORDER_REG, &order);
		if (ret < 0) {
			return ret;
		}

		fourcc = piv_fourcc(type, order);
		if (fourcc == 0) {
			LOG_DBG("Skipping unsupported format %u (dt 0x%02x order %u)", f, type,
				order);
			continue;
		}

		for (uint32_t r = 0; r < UINT8_MAX; r++) {
			uint32_t width = 0, height = 0;
			struct video_format_cap *cap;

			ret = piv_wait_idle(dev);
			if (ret < 0) {
				return ret;
			}

			ret = piv_write(dev, RESOLUTION_INDEX_REG, r);
			if (ret < 0) {
				return ret;
			}

			ret = piv_read(dev, FORMAT_WIDTH_REG, &width);
			if (ret < 0) {
				return ret;
			}

			if (width == NO_DATA_AVAILABLE) {
				break;
			}

			ret = piv_read(dev, FORMAT_HEIGHT_REG, &height);
			if (ret < 0) {
				return ret;
			}

			if (data->num_caps >= CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS) {
				LOG_WRN("Module reports more than %d (format, resolution) "
					"pairs; truncating. Raise "
					"CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS.",
					CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS);
				return 0;
			}

			cap = &data->caps[data->num_caps];
			cap->pixelformat = fourcc;
			cap->width_min = width;
			cap->width_max = width;
			cap->height_min = height;
			cap->height_max = height;
			cap->width_step = 0;
			cap->height_step = 0;

			data->cap_index[data->num_caps].fmt_idx = f;
			data->cap_index[data->num_caps].res_idx = r;
			data->num_caps++;

			LOG_DBG("cap %u: %ux%u fourcc 0x%08x", data->num_caps - 1, width, height,
				fourcc);
		}
	}

	return 0;
}

static int pivariety_get_caps(const struct device *dev, struct video_caps *caps)
{
	struct pivariety_data *data = dev->data;

	if (caps->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	caps->format_caps = data->caps;
	caps->min_vbuf_count = 1;

	return 0;
}

static int pivariety_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct pivariety_data *data = dev->data;
	size_t idx;
	int ret;

	if (fmt->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	/* The caps array is built in the same order as cap_index, so the index the
	 * subsystem's matcher returns is exactly the one that remembers which
	 * module format/resolution indices produced this entry.
	 */
	ret = video_format_caps_index(data->caps, fmt, &idx);
	if (ret < 0) {
		LOG_ERR("Format '%s' %ux%u not advertised by this module",
			VIDEO_FOURCC_TO_STR(fmt->pixelformat), fmt->width, fmt->height);
		return -ENOTSUP;
	}

	ret = piv_wait_idle(dev);
	if (ret < 0) {
		return ret;
	}

	ret = piv_write(dev, PIXFORMAT_INDEX_REG, data->cap_index[idx].fmt_idx);
	if (ret < 0) {
		LOG_ERR("Failed to select format %zu (%d)", idx, ret);
		return ret;
	}

	ret = piv_wait_idle(dev);
	if (ret < 0) {
		return ret;
	}

	ret = piv_write(dev, RESOLUTION_INDEX_REG, data->cap_index[idx].res_idx);
	if (ret < 0) {
		LOG_ERR("Failed to select resolution %zu (%d)", idx, ret);
		return ret;
	}

	/* The module has no register reporting pitch or frame size, so derive them
	 * from the fourcc.
	 */
	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	data->fmt = *fmt;

	return 0;
}

static int pivariety_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct pivariety_data *data = dev->data;

	if (fmt->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	*fmt = data->fmt;

	return 0;
}

static int pivariety_set_stream(const struct device *dev, bool on, enum video_buf_type type)
{
	int ret;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	ret = piv_wait_idle(dev);
	if (ret < 0) {
		return ret;
	}

	return piv_write(dev, MODE_SELECT_REG, on ? PIV_MODE_STREAMING : PIV_MODE_STANDBY);
}

/* The module reports raw V4L2 control IDs. Register the ones Zephyr also
 * defines and skip the rest -- including Arducam's vendor range -- rather than
 * failing probe over a control nobody asked for.
 */
static bool piv_ctrl_is_supported(uint32_t id)
{
	switch (id) {
	case VIDEO_CID_EXPOSURE:
	case VIDEO_CID_GAIN:
	case VIDEO_CID_ANALOGUE_GAIN:
	case VIDEO_CID_BRIGHTNESS:
	case VIDEO_CID_CONTRAST:
	case VIDEO_CID_SATURATION:
	case VIDEO_CID_HFLIP:
	case VIDEO_CID_VFLIP:
		return true;
	default:
		return false;
	}
}

static int piv_init_ctrls(const struct device *dev)
{
	const struct pivariety_config *cfg = dev->config;
	struct pivariety_data *data = dev->data;
	int ret;

	data->link_freq_menu[0] = cfg->link_freq_hz;

	ret = video_init_int_menu_ctrl(&data->linkfreq, dev, VIDEO_CID_LINK_FREQ, 0,
				       data->link_freq_menu, ARRAY_SIZE(data->link_freq_menu));
	if (ret < 0) {
		return ret;
	}
	data->linkfreq.flags |= VIDEO_CTRL_FLAG_READ_ONLY;

	for (uint32_t i = 0; i < UINT8_MAX; i++) {
		uint32_t id = 0, min = 0, max = 0, step = 0, def = 0;

		ret = piv_wait_idle(dev);
		if (ret < 0) {
			return ret;
		}

		ret = piv_write(dev, CTRL_INDEX_REG, i);
		if (ret < 0) {
			return ret;
		}

		ret = piv_read(dev, CTRL_ID_REG, &id);
		if (ret < 0) {
			return ret;
		}
		if (id == NO_DATA_AVAILABLE) {
			break;
		}

		if (!piv_ctrl_is_supported(id)) {
			LOG_DBG("Skipping control 0x%08x with no Zephyr equivalent", id);
			continue;
		}

		if (data->num_generic >= PIV_MAX_GENERIC_CTRLS) {
			LOG_WRN("More than %d mappable controls; ignoring the rest",
				PIV_MAX_GENERIC_CTRLS);
			break;
		}

		ret = piv_read(dev, CTRL_MIN_REG, &min);
		if (ret < 0) {
			return ret;
		}

		ret = piv_read(dev, CTRL_MAX_REG, &max);
		if (ret < 0) {
			return ret;
		}

		ret = piv_read(dev, CTRL_STEP_REG, &step);
		if (ret < 0) {
			return ret;
		}

		ret = piv_read(dev, CTRL_DEF_REG, &def);
		if (ret < 0) {
			return ret;
		}

		ret = video_init_ctrl(&data->generic[data->num_generic], dev, id,
				      (struct video_ctrl_range){
					      .min = min, .max = max, .step = step, .def = def});
		if (ret < 0) {
			return ret;
		}

		data->generic_id[data->num_generic] = id;
		data->num_generic++;
	}

	return 0;
}

static int pivariety_set_ctrl(const struct device *dev, unsigned int cid)
{
	struct pivariety_data *data = dev->data;
	int ret;

	for (uint8_t i = 0; i < data->num_generic; i++) {
		if (data->generic_id[i] != cid) {
			continue;
		}

		ret = piv_wait_idle(dev);
		if (ret < 0) {
			return ret;
		}

		/* ID and VALUE are written back-to-back with no idle wait between
		 * them: the module latches the pair, and the kernel driver
		 * (pivariety_s_ctrl) writes them the same way. The settle belongs
		 * after the pair rather than between -- that driver's comment notes
		 * controls set in batches fail without it.
		 */
		ret = piv_write(dev, CTRL_ID_REG, cid);
		if (ret < 0) {
			return ret;
		}

		ret = piv_write(dev, CTRL_VALUE_REG, data->generic[i].val);
		if (ret < 0) {
			return ret;
		}

		return piv_wait_idle(dev);
	}

	return -ENOTSUP;
}

static int pivariety_init(const struct device *dev)
{
	const struct pivariety_config *cfg = dev->config;
	struct pivariety_data *data = dev->data;
	uint32_t device_id = 0, sensor_id = 0, version = 0;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	ret = piv_wait_idle(dev);
	if (ret < 0) {
		return ret;
	}

	ret = piv_read(dev, DEVICE_ID_REG, &device_id);
	if (ret < 0) {
		LOG_ERR("Failed to read DEVICE_ID (%d)", ret);
		return -ENODEV;
	}

	ret = piv_read(dev, SENSOR_ID_REG, &sensor_id);
	if (ret < 0) {
		LOG_ERR("Failed to read SENSOR_ID (%d)", ret);
		return -ENODEV;
	}

	ret = piv_read(dev, DEVICE_VERSION_REG, &version);
	if (ret < 0) {
		LOG_ERR("Failed to read DEVICE_VERSION (%d)", ret);
		return -ENODEV;
	}

	if (device_id == NO_DATA_AVAILABLE) {
		LOG_ERR("No Pivariety module responding at 0x%02x", cfg->i2c.addr);
		return -ENODEV;
	}

	LOG_INF("Pivariety device 0x%04x sensor 0x%04x version 0x%04x, link %lld Hz", device_id,
		sensor_id, version, cfg->link_freq_hz);

	ret = piv_enumerate(dev);
	if (ret < 0) {
		LOG_ERR("Format enumeration failed (%d)", ret);
		return ret;
	}

	if (data->num_caps == 0) {
		LOG_ERR("Module advertised no supported formats");
		return -ENOTSUP;
	}

	LOG_INF("Enumerated %u (format, resolution) pairs", data->num_caps);

	data->fmt.type = VIDEO_BUF_TYPE_OUTPUT;
	data->fmt.pixelformat = data->caps[0].pixelformat;
	data->fmt.width = data->caps[0].width_min;
	data->fmt.height = data->caps[0].height_min;

	ret = pivariety_set_fmt(dev, &data->fmt);
	if (ret < 0) {
		return ret;
	}

	return piv_init_ctrls(dev);
}

static DEVICE_API(video, pivariety_driver_api) = {
	.get_caps = pivariety_get_caps,
	.set_format = pivariety_set_fmt,
	.get_format = pivariety_get_fmt,
	.set_stream = pivariety_set_stream,
	.set_ctrl = pivariety_set_ctrl,
};

#define PIV_ENDPOINT(n) DT_INST_ENDPOINT_BY_ID(n, 0, 0)

#define PIVARIETY_INIT(n)                                                                          \
	static struct pivariety_data pivariety_data_##n;                                           \
                                                                                                   \
	static const struct pivariety_config pivariety_cfg_##n = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.link_freq_hz = DT_PROP_BY_IDX(PIV_ENDPOINT(n), link_frequencies, 0),              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &pivariety_init, NULL, &pivariety_data_##n, &pivariety_cfg_##n,   \
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &pivariety_driver_api);     \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(pivariety_##n, DEVICE_DT_INST_GET(n), NULL);

DT_INST_FOREACH_STATUS_OKAY(PIVARIETY_INIT)

#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>

/* The analogue of imx219_regs: prove the I2C protocol works and show exactly
 * what the module advertised, before blaming the CSI receiver for anything.
 */
static int cmd_pivariety_regs(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct pivariety_config *cfg = dev->config;
	struct pivariety_data *data = dev->data;
	static const struct {
		const char *name;
		uint32_t reg;
	} regs[] = {
		{"DEVICE_ID", DEVICE_ID_REG},           {"SENSOR_ID", SENSOR_ID_REG},
		{"DEVICE_VERSION", DEVICE_VERSION_REG}, {"SYSTEM_IDLE", SYSTEM_IDLE_REG},
		{"MODE_SELECT", MODE_SELECT_REG},
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
		uint32_t val = 0;
		int ret = piv_read(dev, regs[i].reg, &val);

		if (ret < 0) {
			shell_error(sh, "%-16s read failed (%d)", regs[i].name, ret);
		} else {
			shell_print(sh, "%-16s = 0x%08x (%u)", regs[i].name, val, val);
		}
	}

	shell_print(sh, "link frequency  = %lld Hz", cfg->link_freq_hz);
	shell_print(sh, "enumerated caps = %u", data->num_caps);

	for (uint8_t i = 0; i < data->num_caps; i++) {
		shell_print(sh, "  [%u] %ux%u fourcc 0x%08x (module fmt %u res %u)", i,
			    data->caps[i].width_min, data->caps[i].height_min,
			    data->caps[i].pixelformat, data->cap_index[i].fmt_idx,
			    data->cap_index[i].res_idx);
	}

	return 0;
}
SHELL_CMD_REGISTER(pivariety_regs, NULL, "Dump Arducam Pivariety identity and enumerated formats",
		   cmd_pivariety_regs);
#endif /* CONFIG_SHELL */

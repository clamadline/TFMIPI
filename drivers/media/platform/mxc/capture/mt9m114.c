/*
 * drivers/media/video/mt9m114.c
 *
 * Aptina MT9M114 sensor driver
 *
 * Copyright (C) 2013 Aptina Imaging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>

#include <linux/fsl_devices.h>
#include <linux/ctype.h>
#include <mach/mipi_csi2.h>
#include <media/v4l2-int-device.h>
#include <media/v4l2-chip-ident.h>
#include "mxc_v4l2_capture.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

#define FUNCTION_DEBUG
#ifdef FUNCTION_DEBUG
#define LOG_FUNCTION_NAME printk("[%d] : %s : %s() ENTER\n", __LINE__, __FILE__, __FUNCTION__)
#define LOG_FUNCTION_NAME_EXIT printk("[%d] : %s : %s() EXIT\n", __LINE__, __FILE__,  __FUNCTION__)
#else
#define LOG_FUNCTION_NAME (0)
#define LOG_FUNCTION_NAME_EXIT (0)
#endif

#define MT9M114_DEBUG
#ifdef MT9M114_DEBUG
#define mt9m114_dbg printk
#else
#define mt9m114_dbg //
#endif

//#define NTSC
#define MT9M114_XCLK_MIN 						6000000
#define MT9M114_XCLK_MAX 						24000000

// Result Status codes
#define MT9M114_ENOERR		0x00 // No error - command was successful
#define MT9M114_ENOENT		0x01 // No such entity
#define MT9M114_EINTR		0x02 // Operation interrupted
#define MT9M114_EIO			0x03 // I/O failure
#define MT9M114_E2BIG		0x04 // Too big
#define MT9M114_EBADF		0x05 // Bad file/handle
#define MT9M114_EAGAIN		0x06 // Would-block, try again
#define MT9M114_ENOMEM		0x07 // Not enough memory/resource
#define MT9M114_EACCES		0x08 // Permission denied
#define MT9M114_EBUSY		0x09 // Entity busy, cannot support operation
#define MT9M114_EEXIST		0x0A // Entity exists
#define MT9M114_ENODEV		0x0B // Device not found
#define MT9M114_EINVAL		0x0C // Invalid argument
#define MT9M114_ENOSPC		0x0D // No space/resource to complete
#define MT9M114_ERANGE		0x0E // Parameter out of range
#define MT9M114_ENOSYS		0x0F // Operation not supported
#define MT9M114_EALREADY	0x10 // Already requested/exists

/* Sysctl registers */
#define MT9M114_CHIP_ID                 0x0000
#define MT9M114_COMMAND_REGISTER            0x0080
#define MT9M114_COMMAND_REGISTER_APPLY_PATCH        (1 << 0)
#define MT9M114_COMMAND_REGISTER_SET_STATE      (1 << 1)
#define MT9M114_COMMAND_REGISTER_REFRESH        (1 << 2)
#define MT9M114_COMMAND_REGISTER_WAIT_FOR_EVENT     (1 << 3)
#define MT9M114_COMMAND_REGISTER_OK         (1 << 15)
#define MT9M114_SOFT_RESET              0x001a
#define MT9M114_PAD_SLEW                0x001e
#define MT9M114_PAD_CONTROL             0x0032

/* XDMA registers */
#define MT9M114_ACCESS_CTL_STAT             0x0982
#define MT9M114_PHYSICAL_ADDRESS_ACCESS         0x098a
#define MT9M114_LOGICAL_ADDRESS_ACCESS          0x098e

/* Core registers */
#define MT9M114_RESET_REGISTER              0x301a
#define MT9M114_FLASH                   0x3046
#define MT9M114_CUSTOMER_REV                0x31fe


/*mask */
#define MT9M114_COMMAND_REGISTER_DOORBELL_MASK	(1 << 15)

/* Command parameters */
#define MT9M114_COMMAND_PARAMS_0	0xFC00
#define MT9M114_COMMAND_PARAMS_1	0xFC02

/* Camera Control registers */
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_START     0xc800
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_START     0xc802
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_END       0xc804
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_END       0xc806
#define MT9M114_CAM_SENSOR_CFG_PIXCLK           0xc808
#define MT9M114_CAM_SENSOR_CFG_ROW_SPEED        0xc80c
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN  0xc80e
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX  0xc810
#define MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES   0xc812
#define MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK      0xc814
#define MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION      0xc816
#define MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW       0xc818
#define MT9M114_CAM_SENSOR_CFG_REG_0_DATA       0xc826
#define MT9M114_CAM_SENSOR_CONTROL_READ_MODE        0xc834
#define MT9M114_CAM_CROP_WINDOW_XOFFSET         0xc854
#define MT9M114_CAM_CROP_WINDOW_YOFFSET         0xc856
#define MT9M114_CAM_CROP_WINDOW_WIDTH           0xc858
#define MT9M114_CAM_CROP_WINDOW_HEIGHT          0xc85a
#define MT9M114_CAM_CROP_CROPMODE           0xc85c
#define MT9M114_CAM_OUTPUT_WIDTH            0xc868
#define MT9M114_CAM_OUTPUT_HEIGHT           0xc86a
#define MT9M114_CAM_OUTPUT_FORMAT           0xc86c
#define MT9M114_CAM_AET_AEMODE              0xc878
#define MT9M114_CAM_AET_MAX_FRAME_RATE          0xc88c
#define MT9M114_CAM_AET_MIN_FRAME_RATE          0xc88e
#define MT9M114_CAM_AWB_AWB_XSCALE          0xc8f2
#define MT9M114_CAM_AWB_AWB_YSCALE          0xc8f3
#define MT9M114_CAM_AWB_AWB_XSHIFT_PRE_ADJ      0xc904
#define MT9M114_CAM_AWB_AWB_YSHIFT_PRE_ADJ      0xc906
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART     0xc914
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART     0xc916
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND       0xc918
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND       0xc91a
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART   0xc91c
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART   0xc91e
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND     0xc920
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND     0xc922
#define MT9M114_CAM_SYSCTL_PLL_ENABLE           0xc97e
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N      0xc980
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_P        0xc982
#define MT9M114_CAM_PORT_OUTPUT_CONTROL         0xc984





/* System Manager registers */
#define MT9M114_SYSMGR_NEXT_STATE           0xdc00
#define MT9M114_SYSMGR_CURRENT_STATE            0xdc01
#define MT9M114_SYSMGR_CMD_STATUS           0xdc02

/* SYS_STATE values (for SYSMGR_NEXT_STATE and SYSMGR_CURRENT_STATE) */
#define MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE   0x28
#define MT9M114_SYS_STATE_STREAMING				0x31
#define MT9M114_SYS_STATE_START_STREAMING       0x34
#define MT9M114_SYS_STATE_ENTER_SUSPEND         0x40
#define MT9M114_SYS_STATE_SUSPENDED				0x41
#define MT9M114_SYS_STATE_ENTER_STANDBY         0x50
#define MT9M114_SYS_STATE_STANDBY				0x52
#define MT9M114_SYS_STATE_LEAVE_STANDBY         0x54

/* Patch Loader registers */
#define MT9M114_PATCHLDR_LOADER_ADDRESS         0xe000
#define MT9M114_PATCHLDR_PATCH_ID           0xe002
#define MT9M114_PATCHLDR_FIRMWARE_ID            0xe004
#define MT9M114_PATCHLDR_APPLY_STATUS           0xe008
#define MT9M114_PATCHLDR_NUM_PATCHES            0xe009
#define MT9M114_PATCHLDR_PATCH_ID_0         0xe00a
#define MT9M114_PATCHLDR_PATCH_ID_1         0xe00c
#define MT9M114_PATCHLDR_PATCH_ID_2         0xe00e
#define MT9M114_PATCHLDR_PATCH_ID_3         0xe010
#define MT9M114_PATCHLDR_PATCH_ID_4         0xe012
#define MT9M114_PATCHLDR_PATCH_ID_5         0xe014
#define MT9M114_PATCHLDR_PATCH_ID_6         0xe016
#define MT9M114_PATCHLDR_PATCH_ID_7         0xe018

#define MAX_FRAME_RATE 30

enum mt9m114_mode {
	MT9M114_MODE_MIN = 0,
	MT9M114_MODE_720P = 0,
	MT9M114_MODE_VGA = 1,
	MT9M114_MODE_QVGA = 2,
	MT9M114_MODE_MAX = 0, /* we have to support */
	MT9M114_MODE_INIT = 0xff,/*only for sensor init*/
};

struct mt9m114_resolution {
	unsigned int width;
	unsigned int height;

};

static const struct mt9m114_resolution mt9m114_resolutions[] = {
        [MT9M114_MODE_VGA] = {
            .width  = 640,
            .height = 480,
        },
        [MT9M114_MODE_QVGA] = {
            .width  = 320,
            .height = 240,
        },
        [MT9M114_MODE_720P] = {
            .width  = 1280,
            .height = 720,
        },
};


#define MIN_FPS 10
#define MAX_FPS 30
#define DEFAULT_MODE MT9M114_MODE_720P

static int mt9m114_framerates[] = {
	[MT9M114_MODE_VGA] = 30,
	[MT9M114_MODE_QVGA] = 30,
	[MT9M114_MODE_720P] = 30,
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data mt9m114_data;
static struct fsl_mxc_camera_platform_data *camera_plat;

struct mt9m114_reg {
	u16 reg;
	u32 val;
	int width;
};

static const struct mt9m114_reg mt9m114_init[] = {
	{ MT9M114_RESET_REGISTER,                        0x0218, 2 },//-------

	/* PLL settings */
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x0000, 2 },
	{ MT9M114_CAM_SYSCTL_PLL_ENABLE,                 0x01,   1 },
	{ MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N,            0x0120, 2 }, // 96M
	{ MT9M114_CAM_SYSCTL_PLL_DIVIDER_P,              0x0700, 2 },
	{ MT9M114_CAM_SENSOR_CFG_PIXCLK,                 0x2DC6C00, 4 }, //48MHz

	/* MIPI settings */
	{ MT9M114_CAM_PORT_OUTPUT_CONTROL, 0x8041, 2},
	{ 0xC988, 0x0F00, 2},        //cam_port_mipi_timing_t_hs_zero = 3840
	{ 0xC98A, 0x0B07, 2},        //cam_port_mipi_timing_t_hs_exit_hs_trail = 2823
	{ 0xC98C, 0x0D01, 2},        //cam_port_mipi_timing_t_clk_post_clk_pre = 3329
	{ 0xC98E, 0x071D, 2},        //cam_port_mipi_timing_t_clk_trail_clk_zero = 1821
	{ 0xC990, 0x0006, 2},        //cam_port_mipi_timing_t_lpx = 6
	{ 0xC992, 0x0A0C, 2},        //cam_port_mipi_timing_init_timing = 2572

	/* Sensor optimization */
	{ 0x316A, 0x8270, 2},
	{ 0x316C, 0x8270, 2},
	{ 0x3ED0, 0x2305, 2},
	{ 0x3ED2, 0x77CF, 2},
	{ 0x316E, 0x8202, 2},
	{ 0x3180, 0x87FF, 2},
	{ 0x30D4, 0x6080, 2},
	{ 0xA802, 0x0008, 2},

	{ 0x3E14, 0xFF39, 2},

	/* APGA */
	{ 0xC95E, 0x0000, 2},

	/* Camera control module   */
	{ 0xC892, 0x0267, 2},
	{ 0xC894, 0xFF1A, 2},
	{ 0xC896, 0xFFB3, 2},
	{ 0xC898, 0xFF80, 2},
	{ 0xC89A, 0x0166, 2},
	{ 0xC89C, 0x0003, 2},
	{ 0xC89E, 0xFF9A, 2},
	{ 0xC8A0, 0xFEB4, 2},
	{ 0xC8A2, 0x024D, 2},
	{ 0xC8A4, 0x01BF, 2},
	{ 0xC8A6, 0xFF01, 2},
	{ 0xC8A8, 0xFFF3, 2},
	{ 0xC8AA, 0xFF75, 2},
	{ 0xC8AC, 0x0198, 2},
	{ 0xC8AE, 0xFFFD, 2},
	{ 0xC8B0, 0xFF9A, 2},
	{ 0xC8B2, 0xFEE7, 2},
	{ 0xC8B4, 0x02A8, 2},
	{ 0xC8B6, 0x01D9, 2},
	{ 0xC8B8, 0xFF26, 2},
	{ 0xC8BA, 0xFFF3, 2},
	{ 0xC8BC, 0xFFB3, 2},
	{ 0xC8BE, 0x0132, 2},
	{ 0xC8C0, 0xFFE8, 2},
	{ 0xC8C2, 0xFFDA, 2},
	{ 0xC8C4, 0xFECD, 2},
	{ 0xC8C6, 0x02C2, 2},
	{ 0xC8C8, 0x0075, 2},
	{ 0xC8CA, 0x011C, 2},
	{ 0xC8CC, 0x009A, 2},
	{ 0xC8CE, 0x0105, 2},
	{ 0xC8D0, 0x00A4, 2},
	{ 0xC8D2, 0x00AC, 2},
	{ 0xC8D4, 0x0A8C, 2},  
	{ 0xC8D6, 0x0F0A, 2},
	{ 0xC8D8, 0x1964, 2},

	/* Automatic White balance */
	{ MT9M114_CAM_AWB_AWB_XSHIFT_PRE_ADJ,            0x0033, 2 },
	{ MT9M114_CAM_AWB_AWB_YSHIFT_PRE_ADJ,            0x003C, 2 },
	{ MT9M114_CAM_AWB_AWB_XSCALE,                    0x03,   1 },
	{ MT9M114_CAM_AWB_AWB_YSCALE,                    0x02,   1 },
	{ 0xC8F4, 0x0000, 2},
	{ 0xC8F6, 0x0000, 2},
	{ 0xC8F8, 0x0000, 2},
	{ 0xC8FA, 0xE724, 2},
	{ 0xC8FC, 0x1583, 2},
	{ 0xC8FE, 0x2045, 2},
	{ 0xC900, 0x03FF, 2},
	{ 0xC902, 0x007C, 2},
	{ 0xC90C, 0x80,   1},
	{ 0xC90D, 0x80,   1},
	{ 0xC90E, 0x80,   1},
	{ 0xC90F, 0x88,   1},
	{ 0xC910, 0x80,   1},
	{ 0xC911, 0x80,   1},

	/* CPIPE Preference*/
	{ 0xC926, 0x0020, 2},
	{ 0xC928, 0x009A, 2},
	{ 0xC946, 0x0070, 2},
	{ 0xC948, 0x00F3, 2},
	{ 0xC944, 0x20,   1},
	{ 0xC945, 0x9A,   1},
	{ 0xC92A, 0x80,   1},
	{ 0xC92B, 0x4B,   1},
	{ 0xC92C, 0x00,   1},
	{ 0xC92D, 0xFF,   1},
	{ 0xC92E, 0x3C,   1},
	{ 0xC92F, 0x02,   1},
	{ 0xC930, 0x06,   1},
	{ 0xC931, 0x64,   1},
	{ 0xC932, 0x01,   1},
	{ 0xC933, 0x0C,   1},
	{ 0xC934, 0x3C,   1},
	{ 0xC935, 0x3C,   1},
	{ 0xC936, 0x3C,   1},
	{ 0xC937, 0x0F,   1},
	{ 0xC938, 0x64,   1},
	{ 0xC939, 0x64,   1},
	{ 0xC93A, 0x64,   1},
	{ 0xC93B, 0x32,   1},
	{ 0xC93C, 0x0020, 2},
	{ 0xC93E, 0x009A, 2},
	{ 0xC940, 0x00DC, 2},
	{ 0xC942, 0x38,   1},
	{ 0xC943, 0x30,   1},
	{ 0xC944, 0x50,   1},
	{ 0xC945, 0x19,   1},
	{ 0xC94A, 0x0230, 2},
	{ 0xC94C, 0x0010, 2},
	{ 0xC94E, 0x01CD, 2},
	{ 0xC950, 0x05,   1},
	{ 0xC951, 0x40,   1},
	{ 0xC87B, 0x1B,   1},
	{ MT9M114_CAM_AET_AEMODE, 0x0E, 1},
	{ 0xC890, 0x0080, 2},
	{ 0xC886, 0x0100, 2},
	{ 0xC87C, 0x005A, 2},
	{ 0xB42A, 0x05,   1},
	{ 0xA80A, 0x20,   1},
};

static const struct mt9m114_reg mt9m114_regs_qvga[] = {
            { MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
            { MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0000, 2 },
            { MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0000, 2 },
            { MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x03CD, 2 },
            { MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050D, 2 },
            { MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x01C3, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x03F7, 2 },
            { MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
            { MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x00E0, 2 },
            { MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
            { MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
            { MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
            { MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
            { MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0280, 2 },
			{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
			{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
            { MT9M114_CAM_OUTPUT_WIDTH,                      0x0140, 2 },
            { MT9M114_CAM_OUTPUT_HEIGHT,                     0x00F0, 2 },
            { MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x013F, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x00EF, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x003F, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x002F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_vga[] = {
            { MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
            { MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0000, 2 },
            { MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0000, 2 },
            { MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x03CD, 2 },
            { MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050D, 2 },
            { MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x01C3, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x03F7, 2 },
            { MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
            { MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x00E0, 2 },
            { MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
            { MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
            { MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
            { MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
            { MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0280, 2 },
            { MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
            { MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
            { MT9M114_CAM_OUTPUT_WIDTH,                      0x0280, 2 },
            { MT9M114_CAM_OUTPUT_HEIGHT,                     0x01E0, 2 },
            { MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x027F, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x01DF, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x007F, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x005F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_720p[] = {
            { MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
            { MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0004, 2 },
            { MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0004, 2 },
            { MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x02DB, 2 },
            { MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050B, 2 },
            { MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x00DB, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x05B3, 2 },
            { MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x03EE, 2 },
            { MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x0636, 2 },
            { MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x0060, 2 },
            { MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x02D3, 2 },
            { MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
            { MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
            { MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
            { MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0500, 2 },
            { MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x02D0, 2 },
            { MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
            { MT9M114_CAM_OUTPUT_WIDTH,                      0x0500, 2 },
            { MT9M114_CAM_OUTPUT_HEIGHT,                     0x02D0, 2 },
            { MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
//			{0xA404, 0x0003,2},//Adaptive Weighted AE for lowlights
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x04FF, 2 },
            { MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x02CF, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
			{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x00FF, 2 },
            { MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x008F, 2 },
};

static int mt9m114_read8(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	u8 rval;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = (u8 *)&reg,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 2,
			.buf    = (u8 *)&rval,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		v4l_err(client, "Failed to read register 0x%04x!\n", reg);
		return ret;
	}
	*val = rval;

	return 0;
}

static int mt9m114_write8(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	struct {
		u16 reg;
		u8 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 3,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = val;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_read16(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret;
	u16 rval;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = (u8 *)&reg,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 2,
			.buf    = (u8 *)&rval,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		v4l_err(client, "Failed to read register 0x%04x!\n", reg);
		return ret;
	}
	*val = swab16(rval);

	return 0;
}

static int mt9m114_write16(struct i2c_client *client, u16 reg, u16 val)
{
	int ret;
	struct {
		u16 reg;
		u16 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 4,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = swab16(val);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_write32(struct i2c_client *client, u16 reg, u32 val)
{
	int ret;
	struct {
		u16 reg;
		u32 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 6,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = swab32(val);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_read32(struct i2c_client *client, u16 reg, u32 *val)
{
	int ret;
	u32 rval;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = (u8 *)&reg,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 4,
			.buf    = (u8 *)&rval,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		v4l_err(client, "Failed to read register 0x%04x!\n", reg);
		return ret;
	}
	*val = swab32(rval);

	return 0;
}

static int mt9m114_writeregs(struct i2c_client *client,
		const struct mt9m114_reg *regs, int len)
{
	int i, ret;

	for (i = 0; i < len; i++) {
		switch (regs[i].width) {
			case 1:
				ret = mt9m114_write8(client,
						regs[i].reg, regs[i].val);
				break;
			case 2:
				ret = mt9m114_write16(client,
						regs[i].reg, regs[i].val);
				break;
			case 4:
				ret = mt9m114_write32(client,
						regs[i].reg, regs[i].val);
				break;
			default:
				ret = -EINVAL;
				break;
		}
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int mt9m114_set_state(struct i2c_client *client, u8 next_state)
{
    int timeout = 100, ret;
    u16 command;

  /* set the next desired state */
	 ret = mt9m114_write8(client, MT9M114_SYSMGR_NEXT_STATE, next_state);
	 if (ret < 0)
		 return ret;

 /* start state transition */
	 ret = mt9m114_write16(client, MT9M114_COMMAND_REGISTER, (MT9M114_COMMAND_REGISTER_OK | MT9M114_COMMAND_REGISTER_SET_STATE));
	 if (ret < 0)								       
		return ret;

   /* wait for the state transition to complete */
	  while (timeout) {
		   ret = mt9m114_read16(client,
				   MT9M114_COMMAND_REGISTER, &command);
		   if (ret < 0)
		      return ret;
	       if (!(command & MT9M114_COMMAND_REGISTER_SET_STATE))
			   break;
		   msleep(10);
		   timeout--;
	  }
	  if (!timeout) {
		  v4l_err(client, "Failed to poll command register\n");
           return -ETIMEDOUT;
 }
	   /* check if the command is successful */
	      ret = mt9m114_read16(client,
				  MT9M114_COMMAND_REGISTER, &command);
		 if (ret < 0)
		   return ret;
          if (command & MT9M114_COMMAND_REGISTER_OK)
		  return 0;
		
		 else
	      return -EFAULT;
}


static int mt9m114_set_res(struct i2c_client *client, u32 mode)
{
	int ret = 0;
    u16 read_mode;
	LOG_FUNCTION_NAME;
	v4l_info(client, "mt9m114_set_res: mode = %d\n", mode);
	switch (mode) {
		case MT9M114_MODE_VGA:
            mt9m114_writeregs(client, mt9m114_regs_vga, ARRAY_SIZE(mt9m114_regs_vga));
            mt9m114_read16(client,MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
            read_mode = (read_mode & 0xfccf) | 0x0330;
            mt9m114_write16(client, MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
			v4l_info(client, "set_res = MT9M114_MODE_VGA\n");
			break;

        case MT9M114_MODE_QVGA:
            mt9m114_writeregs(client, mt9m114_regs_qvga, ARRAY_SIZE(mt9m114_regs_qvga));
            mt9m114_read16(client, MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
            read_mode = (read_mode & 0xfccf) | 0x0330;
            mt9m114_write16(client, MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
			v4l_info(client, "set_res = MT9M114_MODE_QVGA\n");
			break;

        case MT9M114_MODE_720P:
            mt9m114_writeregs(client, mt9m114_regs_720p, ARRAY_SIZE(mt9m114_regs_720p));
            mt9m114_read16(client, MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
            read_mode = (read_mode & 0xfccf);
            mt9m114_write16(client, MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
			v4l_info(client, "set_res = MT9M114_MODE_720P\n");
			break;

		default:
			v4l_err(client, "\nSet_res parameter not valid\n");
			break;
	}

	/* start state transition */
	mt9m114_set_state(client, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	mt9m114_set_state(client, MT9M114_SYS_STATE_START_STREAMING);
	LOG_FUNCTION_NAME_EXIT;
	return ret;
}

/************************************************************************
  v4l2_ioctls
 ************************************************************************/
/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s, struct v4l2_frmsizeenum *fsize)
{
	LOG_FUNCTION_NAME;
	if (fsize->index > MT9M114_MODE_MAX)
		return -EINVAL;
	fsize->pixel_format = mt9m114_data.pix.pixelformat;
	fsize->discrete.width = mt9m114_resolutions[fsize->index].width;
	fsize->discrete.height = mt9m114_resolutions[fsize->index].height;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_enum_frameintervals - V4L2 sensor if handler for vidioc_int_enum_frameintervals
 * @s: pointer to standard V4L2 device structure
 * @frmi: pointer to standard V4L2 frameinterval enumeration structure
 *
 * Returns possible frameinterval numerator and denominator depending on choosen frame size
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
		struct v4l2_frmivalenum *fival)
{
	int i;

	LOG_FUNCTION_NAME;
	if (fival->index < 0 || fival->index > MT9M114_MODE_MAX)
		return -EINVAL;
	if (fival->pixel_format == 0 || fival->width == 0 || fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	for (i = 0; i < ARRAY_SIZE(mt9m114_resolutions); i++) {
		if (fival->pixel_format == mt9m114_data.pix.pixelformat
				&& fival->width == mt9m114_resolutions[i].width
				&& fival->height == mt9m114_resolutions[i].height) {
			fival->discrete.denominator =
				mt9m114_framerates[i];
			return 0;
		}
	}
	LOG_FUNCTION_NAME_EXIT;
	return -EINVAL;
}


/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;
	int ret, state;

	LOG_FUNCTION_NAME;

	if (on && !sensor->on) {
		v4l_info(mt9m114_data.i2c_client, "Power on!\n");
		// Make sure power on
		if (camera_plat->pwdn)
			camera_plat->pwdn(0);
		ret = mt9m114_set_state(mt9m114_data.i2c_client,
				MT9M114_SYS_STATE_START_STREAMING);
	

	} else if (!on && sensor->on) {
		v4l_info(mt9m114_data.i2c_client, "Power off!\n");
		if (camera_plat->pwdn)
			camera_plat->pwdn(1);
		ret = mt9m114_set_state(mt9m114_data.i2c_client,
				MT9M114_SYS_STATE_ENTER_SUSPEND);
		
	}
	v4l_dbg(1, debug, mt9m114_data.i2c_client,
			"Firmware State = %d\n", state);
	sensor->on = on;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	LOG_FUNCTION_NAME;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's ifparm
 *
 * Returns device's (sensor's) ifparm in p parameter
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	LOG_FUNCTION_NAME;

	if (s == NULL) {
		v4l_err(mt9m114_data.i2c_client, "   ERROR!! no slave device set!\n");
		return -1;
	}
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656;//V4L2_IF_TYPE_RAW;//V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	if (!mt9m114_data.mclk_source)
		p->u.bt656.clock_curr = mt9m114_data.mclk;
	else
		p->u.bt656.clock_curr = 0;
	v4l_info(mt9m114_data.i2c_client, "clock_curr = %d\n", p->u.bt656.clock_curr);
	p->u.bt656.clock_min = MT9M114_XCLK_MIN;
	p->u.bt656.clock_max = MT9M114_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
		struct v4l2_fmtdesc *fmt)
{
	LOG_FUNCTION_NAME;
	fmt->pixelformat = mt9m114_data.pix.pixelformat;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
		struct v4l2_format *f)
{
	LOG_FUNCTION_NAME;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
		struct v4l2_format *f)
{
	LOG_FUNCTION_NAME;
	f->fmt.pix = mt9m114_data.pix;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
		struct v4l2_format *f)
{
	LOG_FUNCTION_NAME;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
		struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	LOG_FUNCTION_NAME;
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}
	LOG_FUNCTION_NAME_EXIT;
	return ret;
}

/**
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 * ----->Note, this function is not active in this release.<------
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 new_mode = a->parm.capture.capturemode;
	u32 tgt_fps;	/* target frames per secound */
	int ret = 0;
	void *mipi_csi2_info;
	u32 mipi_reg;

	/* Make sure power on */
	if (camera_plat->pwdn)
		camera_plat->pwdn(0);

	switch (a->type) {
		/* This is the only case currently handled. */
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			/* Check that the new frame rate is allowed. */
			if ((timeperframe->numerator == 0) ||
					(timeperframe->denominator == 0)) {
				timeperframe->denominator = mt9m114_framerates[DEFAULT_MODE];
				timeperframe->numerator = 1;
			}
			tgt_fps = timeperframe->denominator /
				timeperframe->numerator;
			if (tgt_fps > MAX_FPS) {
				timeperframe->denominator = MAX_FPS;
				timeperframe->numerator = 1;
			} else if (tgt_fps < MIN_FPS) {
				timeperframe->denominator = MIN_FPS;
				timeperframe->numerator = 1;
			}
			/* Change the sensor mode */
			ret = mt9m114_set_res(mt9m114_data.i2c_client, new_mode);
			if (ret < 0)
				return ret;
			/* Check for MIPI DPHY Clock */
			mipi_csi2_info =  mipi_csi2_get_info();
			if (mipi_csi2_info) {
				unsigned int i;
				i = 0;
				/* wait for mipi sensor ready */
				mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
				while ((mipi_reg == 0x200) && (i < 10)) {
					mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
					i++;
					msleep(10);
				}
				printk("\nmipi_reg=%x",mipi_reg);
				if (i >= 10) {
					printk("no clock");
					pr_err("mipi csi2 can not receive sensor clk!\n");
					return -1;
				}
				i = 0;

				/* wait for mipi stable */
				mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
				while ((mipi_reg != 0x0) && (i < 10)) {
					mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
					i++;
					msleep(10);
				}
				if (i >= 10) {
					pr_err("mipi csi2 can not reveive data correctly!\n");
					return -1;
				}
			} else {
				ret = -1;
			}
			sensor->streamcap.timeperframe = *timeperframe;
			sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;
			break;

			/* These are all the possible cases. */
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		case V4L2_BUF_TYPE_VBI_CAPTURE:
		case V4L2_BUF_TYPE_VBI_OUTPUT:
		case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
		case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
			v4l_dbg(1, debug, mt9m114_data.i2c_client, "V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n", a->type);
			ret = -EINVAL;
			break;
		default:
			v4l_dbg(1, debug, mt9m114_data.i2c_client, "type is unknown - %d\n", a->type);
			ret = -EINVAL;
			break;
	}

	return ret;
}

/**
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 * ----->Note, this function is not active in this release.<------
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s,
		struct v4l2_control *vc)
{
	LOG_FUNCTION_NAME;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 * ----->Note, this function is not active in this release.<------
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
		struct v4l2_control  *ctrl)
{
	LOG_FUNCTION_NAME;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
		struct v4l2_queryctrl *qc)
{
	LOG_FUNCTION_NAME;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	int ret = 0;
	LOG_FUNCTION_NAME;
	void *mipi_csi2_info;

    v4l_info(mt9m114_data.i2c_client, "Setting mclk to %d MHz\n", mt9m114_data.mclk);
    set_mclk_rate(&mt9m114_data.mclk, mt9m114_data.mclk_source);

	 mipi_csi2_info = mipi_csi2_get_info();

	 /* enable mipi csi2 */
	 if (mipi_csi2_info)
		 mipi_csi2_enable(mipi_csi2_info);
	 else {
		 v4l_err(mt9m114_data.i2c_client, "Fail to get mipi_csi2_info!\n");
		 return -EPERM;
	 }
	 if (mipi_csi2_get_status(mipi_csi2_info)) {
		 mipi_csi2_set_lanes(mipi_csi2_info, 1);
		 if (mt9m114_data.pix.pixelformat == V4L2_PIX_FMT_YUYV)
			 mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);
		 else if (mt9m114_data.pix.pixelformat == V4L2_PIX_FMT_RGB565)
			 mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RGB565);
		 else
			 v4l_err(mt9m114_data.i2c_client,
					 "Currently this sensor format can not be supported!\n");
	 } else {
		 v4l_err(mt9m114_data.i2c_client, "Cannot enable mipi csi2 driver!\n");
		 return -1;
	 }

	 LOG_FUNCTION_NAME_EXIT;
	 return ret;
}


/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	LOG_FUNCTION_NAME;

	void *mipi_csi2_info;
	    mipi_csi2_info = mipi_csi2_get_info();

	    /* disable mipi csi2 */
	    if (mipi_csi2_info)
		    if (mipi_csi2_get_status(mipi_csi2_info))
			    mipi_csi2_disable(mipi_csi2_info);

	    return 0;

	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/*!
 * ioctl_init_num - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init_num(struct v4l2_int_device *s)
{
	LOG_FUNCTION_NAME;
	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	LOG_FUNCTION_NAME;
	((struct v4l2_dbg_chip_ident *)id)->match.type =
		V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "mt9m114_mipi");

	LOG_FUNCTION_NAME_EXIT;
	return 0;
}


static struct v4l2_int_ioctl_desc mt9m114_ioctl_desc[] =
{
	{ .num = vidioc_int_enum_framesizes_num, .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num, .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ .num = vidioc_int_s_power_num, .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num, .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_g_ifparm_num, .func = (v4l2_int_ioctl_func *)ioctl_g_ifparm },
	{ .num = vidioc_int_enum_fmt_cap_num, .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num, .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num, .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ .num = vidioc_int_s_fmt_cap_num, .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num, .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ .num = vidioc_int_s_parm_num,	.func = (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ .num = vidioc_int_g_ctrl_num,	.func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,	.func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ .num = vidioc_int_queryctrl_num, .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_dev_init_num, .func = (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ .num = vidioc_int_dev_exit_num, .func = (v4l2_int_ioctl_func *)ioctl_dev_exit },
	{ .num = vidioc_int_init_num, .func = (v4l2_int_ioctl_func *)ioctl_init_num },
	{ .num = vidioc_int_g_chip_ident_num,.func = (v4l2_int_ioctl_func *)ioctl_g_chip_ident },
};


static struct v4l2_int_slave mt9m114_slave = {
	.ioctls = mt9m114_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9m114_ioctl_desc),
};


static struct v4l2_int_device mt9m114_int_device = {
	.module = THIS_MODULE,
	.name = "mt9m114",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9m114_slave,
	},
};

static int mt9m114_probe(struct i2c_client *client,	const struct i2c_device_id *did)
{
	int retval;
	void *mipi_csi2_info;
	u32 mipi_reg;
	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;
	int ret;
	u16 chip_id;
	u16 output_control,command;
	int state;
	u32 reg32;
	u8 reg8;
	u16 reg16;
	struct sensor_data *sensor = &mt9m114_data; //~~C.LaMadline~~

	LOG_FUNCTION_NAME;

	if (!client->dev.platform_data)
	{
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	/* Set initial values for the sensor struct. */
	memset(&mt9m114_data, 0, sizeof(mt9m114_data));
	sensor->mipi_camera = 1; //~~C.LaMadline~~
	v4l_info(client, "mt9m114: mclk = %d\n", plat_data->mclk);
	mt9m114_data.mclk = plat_data->mclk; /* 6 - 54 MHz, typical 24MHz */
	mt9m114_data.mclk_source = plat_data->mclk_source;
	mt9m114_data.csi = plat_data->csi;
	mt9m114_data.io_init = plat_data->io_init;
	sensor->virtual_channel = sensor->csi | (sensor->ipu_id << 1);  //~~C.LaMadline~~
	mt9m114_data.i2c_client = client;
	mt9m114_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	mt9m114_data.pix.width = mt9m114_resolutions[DEFAULT_MODE].width;
	mt9m114_data.pix.height = mt9m114_resolutions[DEFAULT_MODE].height;
	mt9m114_data.pix.priv = 0;
	mt9m114_data.on = true;
	mt9m114_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |	V4L2_CAP_TIMEPERFRAME;
	mt9m114_data.streamcap.capturemode = DEFAULT_MODE;
	mt9m114_data.streamcap.timeperframe.denominator =
		mt9m114_framerates[DEFAULT_MODE];
	mt9m114_data.streamcap.timeperframe.numerator = 1;

	if (plat_data->io_init)
		plat_data->io_init();

	if (plat_data->pwdn)
		plat_data->pwdn(0);

	/* Verify Chip ID */
	ret = mt9m114_read16(client, MT9M114_CHIP_ID, &chip_id);
	if (ret < 0) {
		v4l_err(client, "Failed to get chip id\n");
		return -ENODEV;
	}
	if (chip_id != 0x2481) {
		v4l_err(client, "chip id 0x%04x mismatch\n", chip_id);
		return -ENODEV;
	}

	/*MIPI interface enable*/
	mipi_csi2_info =  mipi_csi2_get_info();
	/* initial mipi dphy */
	if (mipi_csi2_info) {
		if (!mipi_csi2_get_status(mipi_csi2_info)) {
			mipi_csi2_enable(mipi_csi2_info);	
		}
		if (mipi_csi2_get_status(mipi_csi2_info)) {
			mipi_csi2_set_lanes(mipi_csi2_info, 1);
			/*Only reset MIPI CSI2 HW at sensor initialize*/
			mipi_csi2_reset(mipi_csi2_info);
		}
		if (mt9m114_data.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
			mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);
		}
	} else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -1;
	}
	//Reset the sensor
	ret = mt9m114_write16(client, MT9M114_SOFT_RESET, 0x0001);
	mdelay(20);
	if (ret < 0) {
		v4l_err(client, "Failed to reset the sensor\n");
		return ret;
	}
	mt9m114_write16(client, MT9M114_SOFT_RESET, 0x0000);
	mdelay(500);

	do {
		ret = mt9m114_read16(client, MT9M114_COMMAND_REGISTER, &command);
		if (ret < 0)
			return ret;
	} while (command & MT9M114_COMMAND_REGISTER_SET_STATE);

	//load initial settings of sensor
	ret = mt9m114_writeregs(client, mt9m114_init, ARRAY_SIZE(mt9m114_init));
	if (ret < 0){
		v4l_err(client, "Failed to initialize the sensor\n");
		return ret;
	}

	mt9m114_set_state(client, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	mt9m114_set_state(client, MT9M114_SYS_STATE_ENTER_SUSPEND);

	/* Set Default Streaming Mode */
	//mt9m114_set_res(mt9m114_data.i2c_client, MT9M114_MODE_VGA);

	if (plat_data->pwdn)
		plat_data->pwdn(1);

	if (mipi_csi2_info) {
		unsigned int i;
		i = 0;
		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		while ((mipi_reg == 0x200) && (i < 10)) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			i++;
			msleep(10);
		}
		printk("\nmipi_reg=%x",mipi_reg);
		if (i >= 10) {
			printk("no clock");
			pr_err("mipi csi2 can not receive sensor clk!\n");
			return -1;
		}
		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}
		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			return -1;
		}
		mt9m114_read8(client,MT9M114_SYSMGR_CURRENT_STATE , &reg8);
		printk("MT9M114 Current sys state = %x\n",reg8);
	}
	// Keeping copy of plat_data
	camera_plat = plat_data;
	mt9m114_int_device.priv = &mt9m114_data;
	// registering with v4l2 int device
	retval = v4l2_int_device_register(&mt9m114_int_device);
	LOG_FUNCTION_NAME_EXIT;
	return retval;
}

/**
 * mt9m114_remove - remove the mt9m114 soc sensor driver module
 * @client: i2c client driver structure
 *
 * Upon the given i2c client, the sensor driver module is removed.
 */
static int mt9m114_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&mt9m114_int_device);
	return 0;
}

static const struct i2c_device_id mt9m114_id[] = {
	{ "mt9m114_mipi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9m114_id);

static struct i2c_driver mt9m114_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mt9m114_mipi",
	},
	.probe    = mt9m114_probe,
	.remove   = mt9m114_remove,
	.id_table = mt9m114_id,
};

/************************************************************************
  module function
 ************************************************************************/
static  __init int  mt9m114_module_init(void)
{
	u8 err;

	err =  i2c_add_driver(&mt9m114_i2c_driver);
	if(err!=0)
		pr_err("%s:driver registration failed, error = %d\n",__func__,err);
	return err;
}

static void __exit mt9m114_module_exit(void)
{
	i2c_del_driver(&mt9m114_i2c_driver);
}


module_init(mt9m114_module_init);
module_exit(mt9m114_module_exit);


MODULE_AUTHOR("Aptina Imaging");
MODULE_DESCRIPTION("Aptina Imaging, MT9M114 sensor driver");
MODULE_LICENSE("GPL v2");

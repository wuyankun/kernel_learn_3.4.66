/*
 * Copyright © 2007-2008 Intel Corporation
 *   Jesse Barnes <jesse.barnes@intel.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef __DRM_EDID_H__
#define __DRM_EDID_H__

#include <linux/types.h>

#define EDID_LENGTH 128 //EDID数据的长度，单位字节，EEDID表示大于128字节
#define DDC_ADDR 0x50//EDID读取时，i2c的默认地址，DDC通道

#define CEA_EXT	    0x02//扩展的每128个数据的类型
#define VTB_EXT	    0x10
#define DI_EXT	    0x40
#define LS_EXT	    0x50
#define MI_EXT	    0x60

struct est_timings {//标准的分辨率支持情况，每个bit代表一种分辨率类型
	u8 t1;/*Established Timing 1 0x23 720x400@70 640x480@60 640x480@75 800x600@60*/
	u8 t2;/*Established Timing 2 0x24 800x600@75 832x624@75 1024x768@60 1024x768@75 1280x1024@75*/
	u8 mfg_rsvd;//厂商保留的种类/*Manufactuer Reserved Timing  0x25 1152x870@75*/
} __attribute__((packed));

/* 00=16:10, 01=4:3, 10=5:4, 11=16:9 */
#define EDID_TIMING_ASPECT_SHIFT 6
#define EDID_TIMING_ASPECT_MASK  (0x3 << EDID_TIMING_ASPECT_SHIFT)

/* need to add 60 */
#define EDID_TIMING_VFREQ_SHIFT  0
#define EDID_TIMING_VFREQ_MASK   (0x3f << EDID_TIMING_VFREQ_SHIFT)

struct std_timing {//标准的时序支持情况，每组占两个字节，大小和刷新率
	u8 hsize; /* need to multiply by 8 then add 248 */
	u8 vfreq_aspect;
} __attribute__((packed));

#define DRM_EDID_PT_HSYNC_POSITIVE (1 << 1)
#define DRM_EDID_PT_VSYNC_POSITIVE (1 << 2)
#define DRM_EDID_PT_SEPARATE_SYNC  (3 << 3)
#define DRM_EDID_PT_STEREO         (1 << 5)
#define DRM_EDID_PT_INTERLACED     (1 << 7)

/* If detailed data is pixel timing */
struct detailed_pixel_timing { //详细时序的描述，共18个字节，可以参考Documentation/EDID/edid.S中的注释说明
	u8 hactive_lo;//lo:lower低8bit
	u8 hblank_lo;
	u8 hactive_hblank_hi;//hi:high高4bit，每个数据是12bit，可表示范围是0-4095
	u8 vactive_lo;
	u8 vblank_lo;
	u8 vactive_vblank_hi;//和上述两组数据分布相同，这是垂直方向
	u8 hsync_offset_lo;
	u8 hsync_pulse_width_lo;
	u8 vsync_offset_pulse_width_lo;
	u8 hsync_vsync_offset_pulse_width_hi;//共4组数据，有两组占用10个bit，有两组占用6个bit数据
	u8 width_mm_lo;
	u8 height_mm_lo;
	u8 width_height_mm_hi;//with，height 两组数据，单位为mm
	u8 hborder;
	u8 vborder;
	u8 misc;
} __attribute__((packed));

/* If it's not pixel timing, it'll be one of the below */
struct detailed_data_string {//字符串描述类型
	u8 str[13];
} __attribute__((packed));

struct detailed_data_monitor_range {//显示器刷新频率范围约束的结构体
	u8 min_vfreq;
	u8 max_vfreq;
	u8 min_hfreq_khz;
	u8 max_hfreq_khz;
	u8 pixel_clock_mhz; /* need to multiply by 10 */
	__le16 sec_gtf_toggle; /* A000=use above, 20=use below */
	u8 hfreq_start_khz; /* need to multiply by 2 */
	u8 c; /* need to divide by 2 */
	__le16 m;
	u8 k;
	u8 j; /* need to divide by 2 */
} __attribute__((packed));

struct detailed_data_wpindex {
	u8 white_yx_lo; /* Lower 2 bits each */
	u8 white_x_hi;
	u8 white_y_hi;
	u8 gamma; /* need to divide by 100 then add 1 */
} __attribute__((packed));

struct detailed_data_color_point {
	u8 windex1;
	u8 wpindex1[3];
	u8 windex2;
	u8 wpindex2[3];
} __attribute__((packed));

struct cvt_timing {
	u8 code[3];
} __attribute__((packed));

struct detailed_non_pixel {//非时钟细节描述的18个字节的结构体定义
	u8 pad1;
	u8 type; /* ff=serial, fe=string, fd=monitor range, fc=monitor name
		    fb=color point data, fa=standard timing data,
		    f9=undefined, f8=mfg. reserved */
	u8 pad2;
	union {
		struct detailed_data_string str;
		struct detailed_data_monitor_range range;
		struct detailed_data_wpindex color;
		struct std_timing timings[6];
		struct cvt_timing cvt[4];
	} data;
} __attribute__((packed));

//18个详细描述的中，用来标识该18个数据表示信息的类型种类，详细看标准文档
#define EDID_DETAIL_EST_TIMINGS 0xf7
#define EDID_DETAIL_CVT_3BYTE 0xf8
#define EDID_DETAIL_COLOR_MGMT_DATA 0xf9
#define EDID_DETAIL_STD_MODES 0xfa
#define EDID_DETAIL_MONITOR_CPDATA 0xfb
#define EDID_DETAIL_MONITOR_NAME 0xfc //Display Product Name Block Tag 用来描述显示器的名字信息，会被软件读出
#define EDID_DETAIL_MONITOR_RANGE 0xfd//Display Range Limits Block Tag 用来标识显示器的刷新频率范围//56-73Hz 31-68KHz
#define EDID_DETAIL_MONITOR_STRING 0xfe
#define EDID_DETAIL_MONITOR_SERIAL 0xff

struct detailed_timing {
	__le16 pixel_clock; /* need to multiply by 10 KHz */
	union {
		struct detailed_pixel_timing pixel_data;
		struct detailed_non_pixel other_data;
	} data;
} __attribute__((packed));

#define DRM_EDID_INPUT_SERRATION_VSYNC (1 << 0)
#define DRM_EDID_INPUT_SYNC_ON_GREEN   (1 << 1)
#define DRM_EDID_INPUT_COMPOSITE_SYNC  (1 << 2)
#define DRM_EDID_INPUT_SEPARATE_SYNCS  (1 << 3)
#define DRM_EDID_INPUT_BLANK_TO_BLACK  (1 << 4)
#define DRM_EDID_INPUT_VIDEO_LEVEL     (3 << 5)
#define DRM_EDID_INPUT_DIGITAL         (1 << 7)//数字输出BIT位
#define DRM_EDID_DIGITAL_DEPTH_MASK    (7 << 4)
#define DRM_EDID_DIGITAL_DEPTH_UNDEF   (0 << 4)
#define DRM_EDID_DIGITAL_DEPTH_6       (1 << 4)
#define DRM_EDID_DIGITAL_DEPTH_8       (2 << 4)
#define DRM_EDID_DIGITAL_DEPTH_10      (3 << 4)
#define DRM_EDID_DIGITAL_DEPTH_12      (4 << 4)
#define DRM_EDID_DIGITAL_DEPTH_14      (5 << 4)
#define DRM_EDID_DIGITAL_DEPTH_16      (6 << 4)
#define DRM_EDID_DIGITAL_DEPTH_RSVD    (7 << 4)
#define DRM_EDID_DIGITAL_TYPE_UNDEF    (0)//6种数字信号类型，包含未定义，DVI，HDMIA/B MDDI和DP
#define DRM_EDID_DIGITAL_TYPE_DVI      (1)
#define DRM_EDID_DIGITAL_TYPE_HDMI_A   (2)
#define DRM_EDID_DIGITAL_TYPE_HDMI_B   (3)
#define DRM_EDID_DIGITAL_TYPE_MDDI     (4)
#define DRM_EDID_DIGITAL_TYPE_DP       (5)

#define DRM_EDID_FEATURE_DEFAULT_GTF      (1 << 0)
#define DRM_EDID_FEATURE_PREFERRED_TIMING (1 << 1)
#define DRM_EDID_FEATURE_STANDARD_COLOR   (1 << 2)
/* If analog */
#define DRM_EDID_FEATURE_DISPLAY_TYPE     (3 << 3) /* 00=mono, 01=rgb, 10=non-rgb, 11=unknown */
/* If digital */
#define DRM_EDID_FEATURE_COLOR_MASK	  (3 << 3)
#define DRM_EDID_FEATURE_RGB		  (0 << 3)
#define DRM_EDID_FEATURE_RGB_YCRCB444	  (1 << 3)
#define DRM_EDID_FEATURE_RGB_YCRCB422	  (2 << 3)
#define DRM_EDID_FEATURE_RGB_YCRCB	  (3 << 3) /* both 4:4:4 and 4:2:2 */

#define DRM_EDID_FEATURE_PM_ACTIVE_OFF    (1 << 5)
#define DRM_EDID_FEATURE_PM_SUSPEND       (1 << 6)
#define DRM_EDID_FEATURE_PM_STANDBY       (1 << 7)

//EDID的数据结构定义，参照标准文档说明
struct edid {
	u8 header[8];//头部信息，数据固定为/*0x00 0xFF ... 0xFF  0x00*//*0x00~0x07*/
	/* Vendor & product info */
	u8 mfg_id[2];//厂商信息/*vendor name 0x08-0x09*/
	u8 prod_code[2];//pid占用两位/*pid,vid 0x0A-0x0B*/
	u32 serial; /* FIXME: byte order *///序列号，占用4个字节/*serial number 0x0C-0x0F*/
	u8 mfg_week;//一年中的第几周，占用1个字节/*week id,year id 0x10-0x11*/
	u8 mfg_year;//哪一年，占用1个字节，+1990得到真实的年份
	/* EDID version */
	u8 version;//支持的EDID协议的版本，大版本/*edid version revision 0x12-0x13*/
	u8 revision;//小版本
	/* Display info: */
	u8 input;//是否支持数字信号等信息/*Video Input Definition 0x14*/
	u8 width_cm;//可显示的最大宽度/*Max H Image Size (cm) 0x15*/
	u8 height_cm;//可显示的最大高度/*Max V Image Size (cm) 0x16*/
	u8 gamma;//支持的gamma的值/*Display Tranfer Gamma 0x17*/
	u8 features;//支持的属性信息/*Feature Support 0x18*/
	/* Color characteristics */
	u8 red_green_lo;/*Rx1 Rx0 Ry1 Ry0 Gx1 Gx0 Gy1 Gy0 0x19*/ //rgbw四色每种用10bit数据，这是第1bit数据
	u8 black_white_lo;/*Bx1 Bx0 By1 By0 Wx1 Wx0 Wy1 Wy0 0x1A*///rgbx四色每种分为xy两个属性
	u8 red_x;/*Red x[9-2] 0x1B*/
	u8 red_y;/*Red y[9-2] 0x1C*/
	u8 green_x;/*Green x[9-2] 0x1D*/
	u8 green_y;/*Green y[9-2] 0x1E*/
	u8 blue_x;/*Blue x[9-2] 0x1F*/
	u8 blue_y;/*Blue y[9-2] 0x20*/
	u8 white_x;/*White x[9-2] 0x21*/
	u8 white_y;/*White y[9-2] 0x22*/
	/* Est. timings and mfg rsvd timings*/
	struct est_timings established_timings;//3个字节，Established Timing 1  Established Timing 2   Manufactuer Reserved Timing
	/* Standard timings 1-8*///每组占用2个字节
	struct std_timing standard_timings[8];//8组标准的时序描述/*Standard Timing Identification #8 0x34-0x35*/
	/* Detailing timings 1-4 */
	struct detailed_timing detailed_timings[4];//4组时序细节描述信息，每组占用18个字节Detailed Timing Descriping #1 #2 #3 #4
	/* Number of 128 byte ext. blocks */
	u8 extensions;//扩展描述的个数，每个占用128个字节，一般为0或者1/*Extension Flag 0x7E*/
	/* Checksum */
	u8 checksum;//检查和/*Checksum 0x7F*/
} __attribute__((packed));

#define EDID_PRODUCT_ID(e) ((e)->prod_code[0] | ((e)->prod_code[1] << 8))

struct drm_encoder;//前向申明信息，为使得gcc可以正常编译，编码，解码，显示信息
struct drm_connector;
struct drm_display_mode;
void drm_edid_to_eld(struct drm_connector *connector, struct edid *edid);//edid转换为eld
int drm_av_sync_delay(struct drm_connector *connector,
		      struct drm_display_mode *mode);
struct drm_connector *drm_select_eld(struct drm_encoder *encoder,//选择其中的一个eld
				     struct drm_display_mode *mode);
int drm_load_edid_firmware(struct drm_connector *connector);//加载固件方法

#endif /* __DRM_EDID_H__ */

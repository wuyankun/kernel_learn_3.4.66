/*
   edid.S: EDID data template

   Copyright (C) 2012 Carsten Emde <C.Emde@osadl.org>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.
*/


/* Manufacturer */
#define MFG_LNX1 'L'//厂商名字，只能有3个字母，用2个字节表示
#define MFG_LNX2 'N'
#define MFG_LNX3 'X'
#define SERIAL 0//序列号为0，参考include/drm_edid.h 中的解释信息
#define YEAR 2012//年份和周信息，更详细信息查看drm_edid.h中的struct edid 中的描述
#define WEEK 5

/* EDID 1.3 standard definitions */ //在1.3规范中，定义的4种长高比类型
#define XY_RATIO_16_10	0b00
#define XY_RATIO_4_3	0b01
#define XY_RATIO_5_4	0b10
#define XY_RATIO_16_9	0b11

#define mfgname2id(v1,v2,v3) \
	((((v1-'@')&0x1f)<<10)+(((v2-'@')&0x1f)<<5)+((v3-'@')&0x1f))//由3个ASII码转换为2Byte数据
#define swap16(v1) ((v1>>8)+((v1&0xff)<<8))//将一个16bit的数据，高8bit和低8bit交换
#define msbs2(v1,v2) ((((v1>>8)&0x0f)<<4)+((v2>>8)&0x0f))
#define msbs4(v1,v2,v3,v4) \
	(((v1&0x03)>>2)+((v2&0x03)>>4)+((v3&0x03)>>6)+((v4&0x03)>>8))
#define pixdpi2mm(pix,dpi) ((pix*25)/dpi)//由像素和DPI数据，得到像素占用的尺寸信息,单位mm
#define xsize pixdpi2mm(XPIX,DPI)//分别得到长高的尺寸信息
#define ysize pixdpi2mm(YPIX,DPI)

		.data//数据字段

/* Fixed header pattern */
header:		.byte	0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00//固定的EDID的头部标识8个字节

mfg_id:		.word	swap16(mfgname2id(MFG_LNX1, MFG_LNX2, MFG_LNX3))//占用1个word的厂商名字信息

prod_code:	.word	0//pid和vid信息，占用两个字节，一个word

/* Serial number. 32 bits, little endian. *///序列号，4个字节，一个long型，小端模式
serial_number:	.long	SERIAL

/* Week of manufacture */
week:		.byte	WEEK//周数信息，一年中的第几周

/* Year of manufacture, less 1990. (1990-2245)//年份信息，从1990年开始，表示范围是1990-2245年
   If week=255, it is the model year instead */
year:		.byte	YEAR-1990//记录的是和1990年数字的差

version:	.byte	VERSION 	/* EDID version, usually 1 (for 1.3) *///EDID的主版本
revision:	.byte	REVISION	/* EDID revision, usually 3 (for 1.3) *///EDID的修订版本

/* If Bit 7=1	Digital input. If set, the following bit definitions apply:
     Bits 6-1	Reserved, must be 0
     Bit 0	Signal is compatible with VESA DFP 1.x TMDS CRGB,
		  1 pixel per clock, up to 8 bits per color, MSB aligned,
   If Bit 7=0	Analog input. If clear, the following bit definitions apply:
     Bits 6-5	Video white and sync levels, relative to blank
		  00=+0.7/-0.3 V; 01=+0.714/-0.286 V;
		  10=+1.0/-0.4 V; 11=+0.7/0 V
   Bit 4	Blank-to-black setup (pedestal) expected
   Bit 3	Separate sync supported
   Bit 2	Composite sync (on HSync) supported
   Bit 1	Sync on green supported
   Bit 0	VSync pulse must be serrated when somposite or
		  sync-on-green is used. */
video_parms:	.byte	0x6d//输入属性信息，分为模拟信号和数字信号，不同的bit代表特定含义

/* Maximum horizontal image size, in centimetres
   (max 292 cm/115 in at 16:9 aspect ratio) */
max_hor_size:	.byte	xsize/10//X水平方向的尺寸信息，单位cm,

/* Maximum vertical image size, in centimetres.
   If either byte is 0, undefined (e.g. projector) */
max_vert_size:	.byte	ysize/10//Y垂直方向的尺寸信息，单位cm

/* Display gamma, minus 1, times 100 (range 1.00-3.5 */
gamma:		.byte	120//gamma值*100得到，表示范围1.00-3.5，常见1.0 2.0 2.2

/* Bit 7	DPMS standby supported
   Bit 6	DPMS suspend supported
   Bit 5	DPMS active-off supported
   Bits 4-3	Display type: 00=monochrome; 01=RGB colour;//显示的类型信息
		  10=non-RGB multicolour; 11=undefined
   Bit 2	Standard sRGB colour space. Bytes 25-34 must contain
		  sRGB standard values.
   Bit 1	Preferred timing mode specified in descriptor block 1.//第一组详细描述，是最优的显示模式
   Bit 0	GTF supported with default parameter values. */
dsp_features:	.byte	0xea//DPMS的属性信息

/* Chromaticity coordinates. */
/* Red and green least-significant bits
   Bits 7-6	Red x value least-significant 2 bits
   Bits 5-4	Red y value least-significant 2 bits
   Bits 3-2	Green x value lst-significant 2 bits
   Bits 1-0	Green y value least-significant 2 bits */
red_green_lsb:	.byte	0x5e //RG的两个方向，各占2个bit，用于数据组合，R XY分别需要10bit

/* Blue and white least-significant 2 bits */
blue_white_lsb:	.byte	0xc0//BW的两个方向，各占2个bit

/* Red x value most significant 8 bits.
   0-255 encodes 0-0.996 (255/256); 0-0.999 (1023/1024) with lsbits */
red_x_msb:	.byte	0xa4//RX方向的高8bit数据，共10bit，可组成1024个数据

/* Red y value most significant 8 bits */
red_y_msb:	.byte	0x59

/* Green x and y value most significant 8 bits */
green_x_y_msb:	.byte	0x4a,0x98

/* Blue x and y value most significant 8 bits */
blue_x_y_msb:	.byte	0x25,0x20

/* Default white point x and y value most significant 8 bits */
white_x_y_msb:	.byte	0x50,0x54

/* Established timings */
/* Bit 7	720x400 @ 70 Hz
   Bit 6	720x400 @ 88 Hz
   Bit 5	640x480 @ 60 Hz
   Bit 4	640x480 @ 67 Hz
   Bit 3	640x480 @ 72 Hz
   Bit 2	640x480 @ 75 Hz
   Bit 1	800x600 @ 56 Hz
   Bit 0	800x600 @ 60 Hz */
estbl_timing1:	.byte	0x00//第一组标准时序，对应bit置为1，表示支持这种标准分辨率,不常用

/* Bit 7	800x600 @ 72 Hz
   Bit 6	800x600 @ 75 Hz
   Bit 5	832x624 @ 75 Hz
   Bit 4	1024x768 @ 87 Hz, interlaced (1024x768)
   Bit 3	1024x768 @ 60 Hz
   Bit 2	1024x768 @ 72 Hz
   Bit 1	1024x768 @ 75 Hz
   Bit 0	1280x1024 @ 75 Hz */
estbl_timing2:	.byte	ESTABLISHED_TIMINGS_BITS//第二组标准时序，常见的1024x768和1280x1024

/* Bit 7	1152x870 @ 75 Hz (Apple Macintosh II)
   Bits 6-0 	Other manufacturer-specific display mod */
estbl_timing3:	.byte	0x00//第三组标准时序，和其他厂商特殊的显示模式分辨率

/* Standard timing */
/* X resolution, less 31, divided by 8 (256-2288 pixels) */
std_xres:	.byte	(XPIX/8)-31//计算方法如上
/* Y resolution, X:Y pixel ratio
   Bits 7-6	X:Y pixel ratio: 00=16:10; 01=4:3; 10=5:4; 11=16:9.
   Bits 5-0	Vertical frequency, less 60 (60-123 Hz) */
std_vres:	.byte	(XY_RATIO<<6)+VFREQ-60//Y的计算方法，刷新频率加上长高比
		.fill	7,2,0x0101	/* Unused *///第2到第7bit填充0x0101

descriptor1://第一组描述信息，常常来描述优选分辨率信息
/* Pixel clock in 10 kHz units. (0.-655.35 MHz, little-endian) */
clock:		.word	CLOCK/10//像素时钟，用10KHZ作为一个单位

/* Horizontal active pixels 8 lsbits (0-4095) */
x_act_lsb:	.byte	XPIX&0xff//水平有效时钟的起始，低8bit,共12bit，可表示0-4095
/* Horizontal blanking pixels 8 lsbits (0-4095)
   End of active to start of next active. *///有效时钟结束，开始下一个单元
x_blk_lsb:	.byte	XBLANK&0xff
/* Bits 7-4 	Horizontal active pixels 4 msbits
   Bits 3-0	Horizontal blanking pixels 4 msbits */
x_msbs:		.byte	msbs2(XPIX,XBLANK)

/* Vertical active lines 8 lsbits (0-4095) */
y_act_lsb:	.byte	YPIX&0xff//垂直方向的3个字节
/* Vertical blanking lines 8 lsbits (0-4095) */
y_blk_lsb:	.byte	YBLANK&0xff
/* Bits 7-4 	Vertical active lines 4 msbits
   Bits 3-0 	Vertical blanking lines 4 msbits */
y_msbs:		.byte	msbs2(YPIX,YBLANK)

/* Horizontal sync offset pixels 8 lsbits (0-1023) From blanking start */
x_snc_off_lsb:	.byte	XOFFSET&0xff//水平同步偏移量
/* Horizontal sync pulse width pixels 8 lsbits (0-1023) *///水平同步结束，共占用10个bit
x_snc_pls_lsb:	.byte	XPULSE&0xff
/* Bits 7-4 	Vertical sync offset lines 4 lsbits -63)
   Bits 3-0 	Vertical sync pulse width lines 4 lsbits -63) */
y_snc_lsb:	.byte	((YOFFSET-63)<<4)+(YPULSE-63)//垂直方向的数据表示约定，各占6个bit
/* Bits 7-6 	Horizontal sync offset pixels 2 msbits
   Bits 5-4 	Horizontal sync pulse width pixels 2 msbits
   Bits 3-2 	Vertical sync offset lines 2 msbits
   Bits 1-0 	Vertical sync pulse width lines 2 msbits */
xy_snc_msbs:	.byte	msbs4(XOFFSET,XPULSE,YOFFSET,YPULSE)

/* Horizontal display size, mm, 8 lsbits (0-4095 mm, 161 in) */
x_dsp_size:	.byte	xsize&0xff//水平显示区域大小，占用14个bit

/* Vertical display size, mm, 8 lsbits (0-4095 mm, 161 in) */
y_dsp_size:	.byte	ysize&0xff//垂直显示区域大小，占用14个bit，单位mm

/* Bits 7-4 	Horizontal display size, mm, 4 msbits
   Bits 3-0 	Vertical display size, mm, 4 msbits */
dsp_size_mbsb:	.byte	msbs2(xsize,ysize)

/* Horizontal border pixels (each side; total is twice this) */
x_border:	.byte	0//水平和垂直的肩宽长度
/* Vertical border lines (each side; total is twice this) */
y_border:	.byte	0

/* Bit 7 	Interlaced
   Bits 6-5 	Stereo mode: 00=No stereo; other values depend on bit 0:
   Bit 0=0: 01=Field sequential, sync=1 during right; 10=similar,
     sync=1 during left; 11=4-way interleaved stereo
   Bit 0=1 2-way interleaved stereo: 01=Right image on even lines;
     10=Left image on even lines; 11=side-by-side
   Bits 4-3 	Sync type: 00=Analog composite; 01=Bipolar analog composite;
     10=Digital composite (on HSync); 11=Digital separate
   Bit 2 	If digital separate: Vertical sync polarity (1=positive)
   Other types: VSync serrated (HSync during VSync)
   Bit 1 	If analog sync: Sync on all 3 RGB lines (else green only)
   Digital: HSync polarity (1=positive)
   Bit 0 	2-way line-interleaved stereo, if bits 4-3 are not 00. */
features:	.byte	0x18+(VSYNC_POL<<2)+(HSYNC_POL<<1)//极性约束定义和同步方式约定

descriptor2:	.byte	0,0	/* Not a detailed timing descriptor *///第二组详细描述信息，非时序描述信息
		.byte	0	/* Must be zero */
		.byte	0xff	/* Descriptor is monitor serial number (text) *///该组描述序列号信息，类型标识位
		.byte	0	/* Must be zero */
start1:		.ascii	"Linux #0"//用ascii码来填充
end1:		.byte	0x0a	/* End marker *///结束的标识位
		.fill	12-(end1-start1), 1, 0x20 /* Padded spaces *///中间多出的部分，填充0x20
descriptor3:	.byte	0,0	/* Not a detailed timing descriptor *///第三组详细描述信息，非时序描述信息
		.byte	0	/* Must be zero */
		.byte	0xfd	/* Descriptor is monitor range limits *///该组描述刷新频率支持范围
		.byte	0	/* Must be zero */
start2:		.byte	VFREQ-1	/* Minimum vertical field rate (1-255 Hz) */
		.byte	VFREQ+1	/* Maximum vertical field rate (1-255 Hz) */
		.byte	(CLOCK/(XPIX+XBLANK))-1 /* Minimum horizontal line rate
						    (1-255 kHz) */
		.byte	(CLOCK/(XPIX+XBLANK))+1 /* Maximum horizontal line rate
						    (1-255 kHz) */
		.byte	(CLOCK/10000)+1	/* Maximum pixel clock rate, rounded up//向上取10的整数倍
					   to 10 MHz multiple (10-2550 MHz) */
		.byte	0	/* No extended timing information type */
end2:		.byte	0x0a	/* End marker */
		.fill	12-(end2-start2), 1, 0x20 /* Padded spaces *///多余空间填充0x20，代表\n
descriptor4:	.byte	0,0	/* Not a detailed timing descriptor *///第四组详细描述信息
		.byte	0	/* Must be zero */
		.byte	0xfc	/* Descriptor is text *///该组描述字符串信息
		.byte	0	/* Must be zero */
start3:		.ascii	TIMING_NAME
end3:		.byte	0x0a	/* End marker */
		.fill	12-(end3-start3), 1, 0x20 /* Padded spaces *///多余空间填充0x20
extensions:	.byte	0	/* Number of extensions to follow *///扩展描述信息的个数
checksum:	.byte	CRC	/* Sum of all bytes must be 0 *///所有的127个数据的检查和，相加得到

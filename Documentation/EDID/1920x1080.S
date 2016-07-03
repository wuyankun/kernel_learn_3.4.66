/*
   1920x1080.S: EDID data set for standard 1920x1080 60 Hz monitor

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

/* EDID */
#define VERSION 1
#define REVISION 3

/* Display */
#define CLOCK 148500 /* kHz *///像素时钟148.5MHZ
#define XPIX 1920
#define YPIX 1080
#define XY_RATIO XY_RATIO_16_9//长高比调整为16：9，准确数据
#define XBLANK 280
#define YBLANK 45
#define XOFFSET 88
#define XPULSE 44
#define YOFFSET (63+4)
#define YPULSE (63+5)
#define DPI 96//DPI的值和1680x1050的相同
#define VFREQ 60 /* Hz */
#define TIMING_NAME "Linux FHD"//这种分辨率统称为FHD，全高清 Full High Definition   HDMI(High-Definition Multimedia Interface)
#define ESTABLISHED_TIMINGS_BITS 0x00 /* none *///不是标准时序中的一种
#define HSYNC_POL 1#有效数据极性和校验和
#define VSYNC_POL 1
#define CRC 0x05

#include "edid.S"

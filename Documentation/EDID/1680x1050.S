/*
   1680x1050.S: EDID data set for standard 1680x1050 60 Hz monitor

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
#define CLOCK 146250 /* kHz */像素时钟频率146.25MHZ
#define XPIX 1680
#define YPIX 1050
#define XY_RATIO XY_RATIO_16_10//长高比大致统计为16：10
#define XBLANK 560//波形的详细细节
#define YBLANK 39
#define XOFFSET 104
#define XPULSE 176
#define YOFFSET (63+3)
#define YPULSE (63+6)
#define DPI 96//DPI提高到96
#define VFREQ 60 /* Hz */
#define TIMING_NAME "Linux WSXGA"//统称为WSXGA,WSXGA(1680x1050)-->SXGA(1280x1024)-->XGA(1024x768)
#define ESTABLISHED_TIMINGS_BITS 0x00 /* none *///不是标准时序中的某位表示
#define HSYNC_POL 1
#define VSYNC_POL 1
#define CRC 0x26//极性信息和校验和

#include "edid.S"

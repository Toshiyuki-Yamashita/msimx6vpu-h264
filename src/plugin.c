/*
mediastreamer2 V4L2 H.264 encoder/decoder plugin
Copyright (C) 2006-2012 Belledonne Communications, Grenoble

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
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#include "mediastreamer2/mscommon.h"

extern void libmsimx6vpu_h264_init_enc(void);
extern void libmsimx6vpu_h264_init_dec(void);

/******************************************************************************
 * Init routine																  *
 *****************************************************************************/

MS2_PUBLIC void libmsimx6vpu_h264_init(void) {
	libmsimx6vpu_h264_init_enc();
	libmsimx6vpu_h264_init_dec();
}
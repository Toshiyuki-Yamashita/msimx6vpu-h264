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
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include "mediastreamer2/msfactory.h"

extern void libmsimx6vpu_h264_init_enc(MSFactory *f);
extern void libmsimx6vpu_h264_init_dec(MSFactory *f);
extern void vpu_wrapper_init_class(void);

/******************************************************************************
 * Init routine																  *
 *****************************************************************************/

MS2_PUBLIC void libmsimx6vpu_h264_init(MSFactory *f) {
	vpu_wrapper_init_class();
	libmsimx6vpu_h264_init_enc(f);
	libmsimx6vpu_h264_init_dec(f);
}

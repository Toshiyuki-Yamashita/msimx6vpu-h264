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

#include "mediastreamer2/msfilter.h"
#include "mediastreamer2/msinterfaces.h"
#include "mediastreamer2/msticker.h"
#include "mediastreamer2/rfc3984.h"

static void msimx6vpu_h264_dec_init(MSFilter *f) {
	
}

static void msimx6vpu_h264_dec_preprocess(MSFilter *f) {
		
}

static void msimx6vpu_h264_dec_process(MSFilter *f) {
	
}

static void msimx6vpu_h264_dec_uninit(MSFilter *f) {
	
}

static int msimx6vpu_h264_reset_first_image(MSFilter *f, void *data) {
	return 0;
}

static MSFilterMethod msimx6vpu_h264_dec_methods[] = {
	{	MS_VIDEO_DECODER_RESET_FIRST_IMAGE_NOTIFICATION,	msimx6vpu_h264_reset_first_image	},
	{	0,							NULL				}
};

MSFilterDesc msimx6vpu_h264_dec_desc = {
	.id = MS_FILTER_PLUGIN_ID,
	.name = "MSIMX6VPU-H264Dec",
	.text = "A H264 decoder using Freescale's IMX6's VPU",
	.category = MS_FILTER_DECODER,
	.enc_fmt = "H264",
	.ninputs = 1,
	.noutputs = 1,
	.init = msimx6vpu_h264_dec_init,
	.preprocess = msimx6vpu_h264_dec_preprocess,
	.process = msimx6vpu_h264_dec_process,
	.postprocess = NULL,
	.uninit = msimx6vpu_h264_dec_uninit,
	.methods = msimx6vpu_h264_dec_methods,
	.flags = 0
};
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
#include "mediastreamer2/msvideo.h"
#include "mediastreamer2/rfc3984.h"

#include <vpu_lib.h>
#include <vpu_io.h>


/******************************************************************************
 * Implementation of the decoder                                              *
 *****************************************************************************/

static void msimx6vpu_h264_enc_init(MSFilter *f) {
	
}

static void msimx6vpu_h264_enc_preprocess(MSFilter *f) {
	
}

static void msimx6vpu_h264_enc_process(MSFilter *f) {
	
}

/*
static void msimx6vpu_h264_enc_postprocess(MSFilter *f) {
	
}
*/

static void msimx6vpu_h264_enc_uninit(MSFilter *f) {
	
}

/******************************************************************************
 * Methods to configure the encoder                                           *
 *****************************************************************************/

static MSFilterMethod msimx6vpu_h264_enc_methods[] = {
	{	0,							NULL					}
};

/******************************************************************************
 * Definition of the encoder                                                  *
 *****************************************************************************/

MSFilterDesc msimx6vpu_h264_enc_desc = {
	.id = MS_FILTER_PLUGIN_ID,
	.name = "MSIMX6VPU-H264Enc",
	.text = "A H264 encoder using Freescale's IMX6's VPU",
	.category = MS_FILTER_ENCODER,
	.enc_fmt = "H264",
	.ninputs = 1,
	.noutputs = 1,
	.init = msimx6vpu_h264_enc_init,
	.preprocess = msimx6vpu_h264_enc_preprocess,
	.process = msimx6vpu_h264_enc_process,
	.postprocess = NULL,//msimx6vpu_h264_enc_postprocess,
	.uninit = msimx6vpu_h264_enc_uninit,
	.methods = msimx6vpu_h264_enc_methods,
	.flags = 0
};

/******************************************************************************
 * Init routine								      *
 *****************************************************************************/

MS2_PUBLIC void libmsimx6vpu_h264_init_enc(void) {
        ms_filter_register(&msimx6vpu_h264_enc_desc);
        ms_message("msimx6vpu-h264 encoder plugin registered.");
}
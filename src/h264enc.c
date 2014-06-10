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

#define MS_H264_CONF(required_bitrate, bitrate_limit, resolution, fps) \
	{ required_bitrate, bitrate_limit, { MS_VIDEO_SIZE_ ## resolution ## _W, MS_VIDEO_SIZE_ ## resolution ## _H }, fps, NULL }

static const MSVideoConfiguration h264_conf_list[] = {
#if defined(ANDROID) || (TARGET_OS_IPHONE == 1) || defined(__arm__)
	MS_H264_CONF( 170000, 512000,  QVGA, 12),
	MS_H264_CONF( 128000,  170000, QCIF, 10),
	MS_H264_CONF(  64000,  128000, QCIF,  7),
	MS_H264_CONF(      0,   64000, QCIF,  5)
#else
	MS_H264_CONF(1024000, 1536000, SVGA, 25),
	MS_H264_CONF( 512000, 1024000,  VGA, 25),
	MS_H264_CONF( 256000,  512000,  VGA, 15),
	MS_H264_CONF( 170000,  256000, QVGA, 15),
	MS_H264_CONF( 128000,  170000, QCIF, 10),
	MS_H264_CONF(  64000,  128000, QCIF,  7),
	MS_H264_CONF(      0,   64000, QCIF,  5)
#endif
};

static const MSVideoConfiguration multicore_h264_conf_list[] = {
#if defined(ANDROID) || (TARGET_OS_IPHONE == 1) || defined(__arm__)
	MS_H264_CONF(2048000, 3072000,       UXGA, 15),
	MS_H264_CONF(1024000, 1536000, SXGA_MINUS, 15),
	MS_H264_CONF( 750000, 1024000,        XGA, 15),
	MS_H264_CONF( 500000,  750000,       SVGA, 15),
	MS_H264_CONF( 300000,  500000,        VGA, 12),
	MS_H264_CONF( 170000,  300000,       QVGA, 12),
	MS_H264_CONF( 128000,   170000,      QCIF, 10),
	MS_H264_CONF(  64000,   128000,      QCIF,  7),
	MS_H264_CONF(      0,    64000,      QCIF,  5)
#else
	MS_H264_CONF(1536000,  2560000, SXGA_MINUS, 15),
	MS_H264_CONF(1536000,  2560000,       720P, 15),
	MS_H264_CONF(1024000,  1536000,        XGA, 15),
	MS_H264_CONF( 512000,  1024000,       SVGA, 15),
	MS_H264_CONF( 256000,   512000,        VGA, 15),
	MS_H264_CONF( 170000,   256000,       QVGA, 15),
	MS_H264_CONF( 128000,   170000,       QCIF, 10),
	MS_H264_CONF(  64000,   128000,       QCIF,  7),
	MS_H264_CONF(      0,    64000,       QCIF,  5)
#endif
};

typedef struct _MSIMX6VPUH264EncData {
	DecHandle handle;
	vpu_mem_desc bitstream_mem;
	Rfc3984Context *packer;
	MSVideoConfiguration vconf;
	const MSVideoConfiguration *vconf_list;
	bool_t generate_keyframe;
	bool_t configure_done;
} MSIMX6VPUH264EncData;

#define STREAM_BUF_SIZE		0x200000
#define KBPS				1000

/******************************************************************************
 * VPU low level access functions                                            *
 *****************************************************************************/

static int msimx6vpu_h264_vpu_enc_open(MSIMX6VPUH264EncData *d) {
	RetCode ret;
	vpu_versioninfo version;
	DecHandle handle = {0};
	EncOpenParam oparam = {0};
	
	ret = vpu_Init(NULL);
	if (ret) {
		ms_error("[msimx6vpu_h264_enc] vpu_Init error: %d", ret);
		return -1;
	}

	ret = vpu_GetVersionInfo(&version);
	if (ret) {
		ms_error("[msimx6vpu_h264_enc] vpu_GetVersionInfo error: %d", ret);
		vpu_UnInit();
		return -1;
	}

	ms_message("[msimx6vpu_h264_enc] VPU firmware version: %d.%d.%d_r%d", version.fw_major, version.fw_minor, version.fw_release, version.fw_code);
	ms_message("[msimx6vpu_h264_enc] VPU library version: %d.%d.%d", version.lib_major, version.lib_minor, version.lib_release);
	
	d->bitstream_mem.size = STREAM_BUF_SIZE;
	ret = IOGetPhyMem(&d->bitstream_mem);
	if (ret) {
		ms_error("[msimx6vpu_h264_enc] IOGetPhyMem error: %d", ret);
		goto err;
	}

	ret = IOGetVirtMem(&d->bitstream_mem);
	if (ret <= 0) {
		ms_error("[msimx6vpu_h264_enc] IOGetVirtMem error: %d", ret);
		return -1;
	}
	
	oparam.bitstreamFormat = STD_AVC;
	oparam.bitstreamBuffer = d->bitstream_mem.phy_addr;
	oparam.bitstreamBufferSize = STREAM_BUF_SIZE;
	oparam.picWidth = d->vconf.vsize.width;
	oparam.picHeight = d->vconf.vsize.height;
	oparam.frameRateInfo = (int)d->vconf.fps;
	oparam.bitRate = d->vconf.required_bitrate / KBPS;
	oparam.ringBufferEnable = 1;
	
	ret = vpu_EncOpen(&handle, &oparam);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncOpen error: %d", ret);
		return -1;
	}

	d->handle = handle;
	return 0;
	
err:
	IOFreeVirtMem(&d->bitstream_mem);
	IOFreePhyMem(&d->bitstream_mem);
	vpu_UnInit();
	return -1;
}

static int msimx6vpu_h264_vpu_enc_init(MSIMX6VPUH264EncData *d) {
	RetCode ret;
	EncInitialInfo initinfo = {0};
	
	ret = vpu_EncGetInitialInfo(d->handle, &initinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncGetInitialInfo error: %d", ret);
		return -2;
	}
}

static int msimx6vpu_h264_vpu_enc_start(MSFilter *f) {
	RetCode ret;
}

static void msimx6vpu_h264_vpu_enc_close(MSIMX6VPUH264EncData *d) {
	RetCode ret;
	
	IOFreeVirtMem(&d->bitstream_mem);
	IOFreePhyMem(&d->bitstream_mem);
	
	ret = vpu_EncClose(d->handle);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncClose error: %d", ret);
	}
		
	vpu_UnInit();
}

/******************************************************************************
 * Implementation of the decoder                                              *
 *****************************************************************************/

static void msimx6vpu_h264_enc_init(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)ms_new(MSIMX6VPUH264EncData, 1);
	
	if (ms_get_cpu_count() > 1) {
		d->vconf_list = &multicore_h264_conf_list[0];
	} else { 
		d->vconf_list = &h264_conf_list[0];
	}
	d->vconf = ms_video_find_best_configuration_for_bitrate(d->vconf_list, 384000);
	
	d->handle = NULL;
	d->configure_done = FALSE;
	d->packer = NULL;
	f->data = d;
}

static void msimx6vpu_h264_enc_preprocess(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	
	if (!d->handle) {
		if (msimx6vpu_h264_vpu_enc_open(d) < 0) {
			ms_error("[msimx6vpu_h264_enc] failed to open the encoder");
		}
	}
	
	d->packer = rfc3984_new();
}

static void msimx6vpu_h264_enc_process(MSFilter *f) {
	
}


static void msimx6vpu_h264_enc_postprocess(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	
	rfc3984_destroy(d->packer);
	d->packer = NULL;
}


static void msimx6vpu_h264_enc_uninit(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	
	msimx6vpu_h264_vpu_enc_close(d);
	ms_free(d);
}

/******************************************************************************
 * Methods to configure the encoder                                           *
 *****************************************************************************/

static int msimx6vpu_h264_enc_get_br(MSFilter *f, void *arg){
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	*(int*)arg = d->vconf.required_bitrate;
	return 0;
}

static int msimx6vpu_h264_enc_set_configuration(MSFilter *f, void *arg) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	const MSVideoConfiguration *vconf = (const MSVideoConfiguration *)arg;
	if (vconf != &d->vconf) memcpy(&d->vconf, vconf, sizeof(MSVideoConfiguration));

	if (d->vconf.required_bitrate > d->vconf.bitrate_limit) {
		d->vconf.required_bitrate = d->vconf.bitrate_limit;
	}
	if (d->handle) {
		ms_filter_lock(f);
		d->configure_done = FALSE;
		msimx6vpu_h264_vpu_enc_close(d);
		msimx6vpu_h264_vpu_enc_open(d);
		ms_filter_unlock(f);
		return 0;
	}

	ms_message("[msimx6vpu_h264_enc] Video configuration set: bitrate=%dbits/s, fps=%f, vsize=%dx%d", d->vconf.required_bitrate, d->vconf.fps, d->vconf.vsize.width, d->vconf.vsize.height);
	return 0;
}

static int msimx6vpu_h264_enc_set_br(MSFilter *f, void *arg) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	int br = *(int *)arg;
	if (d->handle != NULL) {
		/* Encoding is already ongoing, do not change video size, only bitrate. */
		d->vconf.required_bitrate = br;
		msimx6vpu_h264_enc_set_configuration(f, &d->vconf);
	} else {
		MSVideoConfiguration best_vconf = ms_video_find_best_configuration_for_bitrate(d->vconf_list, br);
		msimx6vpu_h264_enc_set_configuration(f, &best_vconf);
	}
	return 0;
}

static int msimx6vpu_h264_enc_set_fps(MSFilter *f, void *arg){
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	d->vconf.fps = *(float*)arg;
	msimx6vpu_h264_enc_set_configuration(f, &d->vconf);
	return 0;
}

static int msimx6vpu_h264_enc_get_fps(MSFilter *f, void *arg){
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	*(float*)arg = d->vconf.fps;
	return 0;
}

static int msimx6vpu_h264_enc_get_vsize(MSFilter *f, void *arg){
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	*(MSVideoSize*)arg = d->vconf.vsize;
	return 0;
}

static int msimx6vpu_h264_enc_set_vsize(MSFilter *f, void *arg){
	MSVideoConfiguration best_vconf;
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	MSVideoSize *vs = (MSVideoSize *)arg;
	best_vconf = ms_video_find_best_configuration_for_size(d->vconf_list, *vs);
	d->vconf.vsize = *vs;
	d->vconf.fps = best_vconf.fps;
	d->vconf.bitrate_limit = best_vconf.bitrate_limit;
	msimx6vpu_h264_enc_set_configuration(f, &d->vconf);
	return 0;
}

static int msimx6vpu_h264_enc_req_vfu(MSFilter *f, void *arg){
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	d->generate_keyframe = TRUE;
	return 0;
}

static int msimx6vpu_h264_enc_get_configuration_list(MSFilter *f, void *data) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	const MSVideoConfiguration **vconf_list = (const MSVideoConfiguration **)data;
	*vconf_list = d->vconf_list;
	return 0;
}

static MSFilterMethod msimx6vpu_h264_enc_methods[] = {
	{ MS_FILTER_SET_FPS,						msimx6vpu_h264_enc_set_fps					},
	{ MS_FILTER_SET_BITRATE,					msimx6vpu_h264_enc_set_br					},
	{ MS_FILTER_GET_BITRATE,					msimx6vpu_h264_enc_get_br					},
	{ MS_FILTER_GET_FPS,						msimx6vpu_h264_enc_get_fps					},
	{ MS_FILTER_GET_VIDEO_SIZE,					msimx6vpu_h264_enc_get_vsize				},
	{ MS_FILTER_SET_VIDEO_SIZE,					msimx6vpu_h264_enc_set_vsize				},
	{ MS_FILTER_REQ_VFU,						msimx6vpu_h264_enc_req_vfu					},
	{ MS_VIDEO_ENCODER_REQ_VFU,					msimx6vpu_h264_enc_req_vfu					},
	{ MS_VIDEO_ENCODER_GET_CONFIGURATION_LIST, 	msimx6vpu_h264_enc_get_configuration_list	},
	{ 0,										NULL										}
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
	.postprocess = msimx6vpu_h264_enc_postprocess,
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
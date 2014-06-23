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

#include "vpu_wrapper.h"

#define MS_H264_CONF(required_bitrate, bitrate_limit, resolution, fps) \
	{ required_bitrate, bitrate_limit, { MS_VIDEO_SIZE_ ## resolution ## _W, MS_VIDEO_SIZE_ ## resolution ## _H }, fps, NULL }

static const MSVideoConfiguration h264_conf_list[] = {
#if defined(ANDROID) || (TARGET_OS_IPHONE == 1) || defined(__arm__)
	MS_H264_CONF( 170000,  512000, VGA,  15),
	MS_H264_CONF( 128000,  170000, QVGA, 12),
	MS_H264_CONF(  64000,  128000, QCIF, 10),
	MS_H264_CONF(      0,   64000, QCIF,  7)
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

void encoder_open_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	ms_message("[msimx6vpu_h264_enc] open callback with result %i", result);
}

void encoder_init_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	d->init_command_queued = FALSE;
	if (result == 0) {
		ms_filter_lock(d->filter);
		d->configure_done = TRUE;
		ms_filter_unlock(d->filter);
		ms_message("[msimx6vpu_h264_enc] encoder initialized");
	} else if (result != -3) {
		ms_error("[msimx6vpu_h264_enc] failed to initialize the encoder");
	}
}

void encoder_fill_buffer_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
}

void encoder_encode_frame_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	uint32_t ts = d->filter->ticker->time * 90LL;
	
	ms_filter_lock(d->filter);
	d->encode_frame_command_queued = FALSE;
	if (result >= 0) {
		if (!ms_queue_empty(&d->nalus)) {
			rfc3984_pack(d->packer, &d->nalus, d->filter->outputs[0], ts);
			if (d->packet_num == 0) {
				ms_message("[msimx6vpu_h264_enc] first frame encoded");
				ms_video_starter_first_frame(&d->starter, d->filter->ticker->time);
			}
			d->packet_num++;
		}
	}
	ms_filter_unlock(d->filter);
}

void encoder_close_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	ms_message("[msimx6vpu_h264_enc] close callback");
	d->configure_done = FALSE;
	
	ms_filter_lock(d->filter);
	ms_free(d->fbpool);
	ms_free(d->fbs);
	ms_filter_unlock(d->filter);
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
	
	if (!VpuWrapper::Instance()->IsVpuInitialized()) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_INIT, NULL, NULL, NULL));
	}
	
	d->handle = NULL;
	d->configure_done = FALSE;
	d->packet_num = 0;
	d->packer = NULL;
	d->src_buffer_index = -1;
	d->src_width = 0;
	d->src_height = 0;
	d->encode_frame_command_queued = FALSE;
	d->init_command_queued = FALSE;
	d->shutdown = FALSE;
	d->mode = 0;
	d->filter = f;
	f->data = d;
}

static void msimx6vpu_h264_enc_preprocess(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	
	if (!d->handle) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_open_callback, NULL));
	}
	
	d->packer = rfc3984_new();
	rfc3984_set_mode(d->packer, d->mode);
	rfc3984_enable_stap_a(d->packer, FALSE);
	ms_video_starter_init(&d->starter);
}

static void msimx6vpu_h264_enc_process(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	MSPicture pic;
	mblk_t *im;
	
	if (!d->handle) {
		ms_error("[msimx6vpu_h264_enc] encoder not openned yet");
		ms_queue_flush(f->inputs[0]);
		return;
	}
	
	ms_filter_lock(f);
	while ((im = ms_queue_get(f->inputs[0])) != NULL) {
		if (ms_yuv_buf_init_from_mblk(&pic, im) == 0) {
			/* send I frame 2 seconds and 4 seconds after the beginning */
			if (ms_video_starter_need_i_frame(&d->starter, f->ticker->time)) {
				ms_message("[msimx6vpu_h264_enc] asking encoder for keyframe");
				d->generate_keyframe = TRUE;
			}
			
			d->src_width = pic.w;
			d->src_height = pic.h;

			if (!d->configure_done && !d->init_command_queued) {
				VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(INIT_ENCODER, d, &encoder_init_callback, NULL));
				d->init_command_queued = TRUE;
			}
			
			if (d->configure_done && !d->encode_frame_command_queued) {
				d->src_pic = pic;
				VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(FILL_ENCODER_BUFFER, d, &encoder_fill_buffer_callback, &d->src_pic));
			}
		}
		freemsg(im);
	}
	
	if (d->configure_done && !d->encode_frame_command_queued) {
		d->encode_frame_command_queued = TRUE;
		ms_queue_init(&d->nalus);
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(ENCODE_FRAME, d, &encoder_encode_frame_callback, &d->nalus));
	}
	ms_filter_unlock(f);
}


static void msimx6vpu_h264_enc_postprocess(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	
	rfc3984_destroy(d->packer);
	d->packer = NULL;
	d->shutdown = TRUE;
	VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(CLOSE_ENCODER, d, &encoder_close_callback, NULL));
}


static void msimx6vpu_h264_enc_uninit(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	ms_free(d);
	VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_UNINIT, NULL, NULL, NULL));
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
		ms_message("[msimx6vpu_h264_enc] reconfiguring encoder");
		ms_filter_lock(f);
		d->configure_done = FALSE;
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(CLOSE_ENCODER, d, &encoder_close_callback, NULL));
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_open_callback, NULL));
		ms_message("[msimx6vpu_h264_enc] encoder restart is done");
		ms_filter_unlock(f);
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

static int msimx6vpu_h264_enc_add_fmtp(MSFilter *f, void *arg){
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	const char *fmtp = (const char *)arg;
	char value[12];
	if (fmtp_get_value(fmtp, "packetization-mode", value, sizeof(value))) {
		d->mode = atoi(value);
		ms_message("[msimx6vpu_h264_enc] packetization-mode set to %i", d->mode);
	}
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
	{ MS_FILTER_ADD_FMTP,						msimx6vpu_h264_enc_add_fmtp					},
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
 * Init routine                                                               *
 *****************************************************************************/

extern "C" {
	MS2_PUBLIC void libmsimx6vpu_h264_init_enc(void) {
		ms_filter_register(&msimx6vpu_h264_enc_desc);
		ms_message("msimx6vpu-h264 encoder plugin registered.");
	}
}

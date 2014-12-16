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

#define MS_H264_CONF(required_bitrate, bitrate_limit, resolution, fps, cpu_count) \
	{ required_bitrate, bitrate_limit, { MS_VIDEO_SIZE_ ## resolution ## _W, MS_VIDEO_SIZE_ ## resolution ## _H }, fps, cpu_count, NULL }

static const MSVideoConfiguration h264_conf_list[] = {
	MS_H264_CONF( 384000,   600000, VGA,  25, 1),
	MS_H264_CONF( 384000,   512000, CIF,  15, 1),
	MS_H264_CONF( 256000,   384000, QVGA, 15, 1),
	MS_H264_CONF( 128000,   256000, QCIF, 15, 1),
	MS_H264_CONF(      0,   64000,  QCIF, 12, 1)
};

void encoder_open_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	ms_message("[msimx6vpu_h264_enc] open callback with result %i", result);
	
	if (result == -3) {
		if (!VpuWrapper::Instance()->IsVpuInitialized()) {
			VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_INIT, NULL, NULL, NULL));
		}
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_open_callback, NULL));
	} else  if (result == -1) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_open_callback, NULL));
	}
}

void encoder_restart_open_callback (void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	ms_message("[msimx6vpu_h264_enc] restart open callback with result %i", result);
	d->restart_started = FALSE;
	ms_filter_unlock(d->filter);
	
	if (result == -3) {
		if (!VpuWrapper::Instance()->IsVpuInitialized()) {
			VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_INIT, NULL, NULL, NULL));
		}
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_open_callback, NULL));
	} else  if (result == -1) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_open_callback, NULL));
	}
}

void encoder_init_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	
	ms_filter_lock(d->filter);
	d->init_command_queued = FALSE;
	if (result == 0) {
		d->configure_done = TRUE;
		ms_message("[msimx6vpu_h264_enc] encoder initialized");
	} else if (result == -4) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_open_callback, NULL));
	} else if (result != -3) {
		ms_error("[msimx6vpu_h264_enc] failed to initialize the encoder");
	}
	ms_filter_unlock(d->filter);
}

void encoder_encode_frame_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	uint32_t ts = 0;
	
	ms_filter_lock(d->filter);
	d->encode_frame_command_queued = FALSE;
	
	if (result < 0) {
		ms_filter_unlock(d->filter);
		return;
	}
	
	ts = d->filter->ticker->time * 90LL;
	if (!ms_queue_empty(d->nalus)) {
		rfc3984_pack(d->packer, d->nalus, d->filter->outputs[0], ts);
		if (d->packet_num == 0) {
			ms_message("[msimx6vpu_h264_enc] first frame encoded");
			ms_video_starter_first_frame(&d->starter, d->filter->ticker->time);
		}
		d->packet_num++;
	}
	ms_filter_unlock(d->filter);
}

void encoder_close_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	ms_message("[msimx6vpu_h264_enc] close callback");
	d->init_command_queued = FALSE;
}

void encoder_restart_close_callback (void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	ms_message("[msimx6vpu_h264_enc] restart close callback");
	d->init_command_queued = FALSE;
	ms_filter_lock(d->filter);
}

void encoder_uninit_callback(void *v, int result) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)v;
	ms_message("[msimx6vpu_h264_enc] uninit callback");
	ms_free(d);
	
	if (result == 0 && vpuWrapperInstance) {
		ms_message("[msimx6vpu_h264_enc] deleting vpu wrapper instance");
		delete vpuWrapperInstance;
	}
}

/******************************************************************************
 * Implementation of the decoder                                              *
 *****************************************************************************/

static void msimx6vpu_h264_enc_init(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)ms_new(MSIMX6VPUH264EncData, 1);
	
	d->vconf_list = &h264_conf_list[0];
	d->vconf = ms_video_find_best_configuration_for_bitrate(d->vconf_list, 384000, 1);
	
	if (!VpuWrapper::Instance()->IsVpuInitialized()) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_INIT, NULL, NULL, NULL));
	}
	
	d->handle = NULL;
	d->nalus = NULL;
	d->sps_mblkt = NULL;
	d->pps_mblkt = NULL;
	d->configure_done = FALSE;
	d->generate_keyframe = FALSE;
	d->packet_num = 0;
	d->packer = NULL;
	d->src_buffer_index = -1;
	d->src_width = 0;
	d->src_height = 0;
	d->encode_frame_command_queued = FALSE;
	d->init_command_queued = FALSE;
	d->shutdown = FALSE;
	d->restart_started = FALSE;
	d->mode = 0;
	d->latest_src_buffer = ENCODER_SRC_BUFFERS - 1;
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
	d->nalus = ms_queue_new(f, NULL, NULL, NULL);
}

static int msimx6vpu_h264_enc_fill_encoder_buffer(MSIMX6VPUH264EncData *d, MSPicture *pic) {
	int offset = 0;
	IMX6VPUFrameBuffer *framebuff;
	MSVideoSize roi = {0};
	uint8_t *dest_planes[3];
	int dest_strides[3];
	
	d->latest_src_buffer = (d->latest_src_buffer + 1) % ENCODER_SRC_BUFFERS;
	framebuff = d->fbpool[d->src_buffer_index + d->latest_src_buffer];
	offset = framebuff->desc.virt_uaddr - framebuff->desc.phy_addr;
	dest_planes[0] = (uint8_t *) framebuff->addrY + offset;
	dest_planes[1] = (uint8_t *) framebuff->addrCb + offset;
	dest_planes[2] = (uint8_t *) framebuff->addrCr + offset;
	
	dest_strides[0] = framebuff->strideY;
	dest_strides[1] = framebuff->strideC;
	dest_strides[2] = framebuff->strideC;
	
	roi.width = pic->w;
	roi.height = pic->h;
	
	ms_yuv_buf_copy(pic->planes, pic->strides, dest_planes, dest_strides, roi);
	return 0;
}

static void msimx6vpu_h264_enc_process(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	MSPicture pic;
	mblk_t *im;
	
	ms_filter_lock(f);
	if (!d->handle) {
		ms_error("[msimx6vpu_h264_enc] encoder not openned yet");
		ms_queue_flush(f->inputs[0]);
		ms_filter_unlock(f);
		return;
	}
	
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
			
			if (d->configure_done && !d->init_command_queued && !d->restart_started && !d->encode_frame_command_queued) {
				msimx6vpu_h264_enc_fill_encoder_buffer(d, &pic);
			}
		}
		freemsg(im);
	}
	
	if (d->configure_done && !d->encode_frame_command_queued) {
		d->encode_frame_command_queued = TRUE;
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(ENCODE_FRAME, d, &encoder_encode_frame_callback, d->nalus));
	}
	ms_filter_unlock(f);
}


static void msimx6vpu_h264_enc_postprocess(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	
	rfc3984_destroy(d->packer);
	d->packer = NULL;
	d->shutdown = TRUE;
	ms_queue_destroy(d->nalus);
	
	if (d->handle) {
		ms_message("[msimx6vpu_h264_enc] shutting down encoder");
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(CLOSE_ENCODER, d, &encoder_close_callback, NULL));
	}
}


static void msimx6vpu_h264_enc_uninit(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData *)f->data;
	VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_UNINIT, d, &encoder_uninit_callback, NULL));
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
		d->restart_started = TRUE;
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(CLOSE_ENCODER, d, &encoder_restart_close_callback, NULL));
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_ENCODER, d, &encoder_restart_open_callback, NULL));
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
		MSVideoConfiguration best_vconf = ms_video_find_best_configuration_for_bitrate(d->vconf_list, br, 1);
		msimx6vpu_h264_enc_set_configuration(f, &best_vconf);
	}
	return 0;
}

static int msimx6vpu_h264_enc_set_fps(MSFilter *f, void *arg){
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	float fps = *(float*)arg;
	d->vconf.fps = fps;
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
	best_vconf = ms_video_find_best_configuration_for_size(d->vconf_list, *vs, 1);
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
	{ MS_VIDEO_ENCODER_SET_CONFIGURATION,		msimx6vpu_h264_enc_set_configuration		},
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

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

static void update_sps(MSIMX6VPUH264DecData *d, mblk_t *sps){
	if (d->sps)
		freemsg(d->sps);
	d->sps=dupb(sps);
}

static void update_pps(MSIMX6VPUH264DecData *d, mblk_t *pps){
	if (d->pps)
		freemsg(d->pps);
	if (pps) d->pps=dupb(pps);
	else d->pps=NULL;
}

static bool_t check_sps_change(MSIMX6VPUH264DecData *d, mblk_t *sps){
	bool_t ret=FALSE;
	if (d->sps){
		ret=(msgdsize(sps)!=msgdsize(d->sps)) || (memcmp(d->sps->b_rptr,sps->b_rptr,msgdsize(sps))!=0);
		if (ret) {
			ms_message("[msimx6vpu_h264_dec] SPS changed ! %i,%i",(int)msgdsize(sps),(int)msgdsize(d->sps));
			update_sps(d,sps);
			update_pps(d,NULL);
		}
	} else {
		ms_message("[msimx6vpu_h264_dec] Receiving first SPS");
		update_sps(d,sps);
	}
	return ret;
}

static bool_t check_pps_change(MSIMX6VPUH264DecData *d, mblk_t *pps){
	bool_t ret=FALSE;
	if (d->pps){
		ret=(msgdsize(pps)!=msgdsize(d->pps)) || (memcmp(d->pps->b_rptr,pps->b_rptr,msgdsize(pps))!=0);
		if (ret) {
			ms_message("[msimx6vpu_h264_dec] PPS changed ! %i,%i",(int)msgdsize(pps),(int)msgdsize(d->pps));
			update_pps(d,pps);
		}
	}else {
		ms_message("[msimx6vpu_h264_dec] Receiving first PPS");
		update_pps(d,pps);
	}
	return ret;
}

static uint8_t* nalusToFrame(MSIMX6VPUH264DecData *d, MSQueue *naluq, bool_t *new_sps_pps, uint8_t *bitstream){
	mblk_t *im;
	uint8_t *dst,*src,*end;
	int nal_len;
	bool_t start_picture=TRUE;
	uint8_t nalu_type;
	*new_sps_pps=FALSE;
	dst = bitstream;
	end=bitstream+d->bitstream_size;
	while((im=ms_queue_get(naluq))!=NULL){
		src=im->b_rptr;
		nal_len=im->b_wptr-src;
		if (dst+nal_len+100>end){
			int pos=dst-bitstream;
			int new_size = d->bitstream_size+nal_len+100;
			ms_warning("[msimx6vpu_h264_dec] increasing bitstream size from %i to %i", d->bitstream_size, new_size);
			d->bitstream_size = new_size;
			bitstream = (uint8_t*) ms_realloc(bitstream, d->bitstream_size);
			dst=bitstream+pos;
			end=bitstream+d->bitstream_size;
		}
		if (src[0]==0 && src[1]==0 && src[2]==0 && src[3]==1){
			int size=im->b_wptr-src;
			/*workaround for stupid RTP H264 sender that includes nal markers */
			memcpy(dst,src,size);
			dst+=size;
		}else{
			nalu_type=(*src) & ((1<<5)-1);
			if (nalu_type==7)
				*new_sps_pps=check_sps_change(d,im) || *new_sps_pps;
			if (nalu_type==8)
				*new_sps_pps=check_pps_change(d,im) || *new_sps_pps;
			if (start_picture || nalu_type==7/*SPS*/ || nalu_type==8/*PPS*/ ){
				*dst++=0;
				start_picture=FALSE;
			}
		
			/*prepend nal marker*/
			*dst++=0;
			*dst++=0;
			*dst++=1;
			*dst++=*src++;
			while(src<(im->b_wptr-3)){
				if (src[0]==0 && src[1]==0 && src[2]<3){
					*dst++=0;
					*dst++=0;
					*dst++=3;
					src+=2;
				}
				*dst++=*src++;
			}
			*dst++=*src++;
			*dst++=*src++;
			*dst++=*src++;
		}
		freemsg(im);
	}
	return dst-bitstream;
}

void decoder_open_callback(void *v, int result) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)v;
	ms_message("[msimx6vpu_h264_dec] open callback with result %i", result);
	
	if (result == -3) {
		if (!VpuWrapper::Instance()->IsVpuInitialized()) {
			VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_INIT, NULL, NULL, NULL));
		}
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_DECODER, d, &decoder_open_callback, NULL));
	} else  if (result == -1) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_DECODER, d, &decoder_open_callback, NULL));
	}
}

void decoder_init_callback(void *v, int result) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)v;
	
	if (result == 0) {
		ms_filter_lock(d->filter);
		d->configure_done = TRUE;
		ms_filter_unlock(d->filter);
		ms_message("[msimx6vpu_h264_dec] decoder initialised");
	} else if (result == -4) {
		ms_warning("[msimx6vpu_h264_dec] decoder not openned yet, let's do it");
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_DECODER, d, &decoder_open_callback, NULL));
	} else if (result != -3) {
		ms_error("[msimx6vpu_h264_dec] failed to initialise the decoder: %i", result);
	}
}

void decoder_fill_buffer_callback(void *v, int result) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)v;
	if (result != 0) {
		ms_error("[msimx6vpu_h264_dec] failed to fill the vpu's bitstream buffer");
	}
}

void decoder_decode_frame_callback(void *v, int result) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)v;
	
	ms_filter_lock(d->filter);
	d->decode_frame_command_queued = FALSE;
	if (result == 0) {
		if (!d->first_image_decoded) {
			d->first_image_decoded = TRUE;
			ms_filter_notify_no_arg(d->filter, MS_VIDEO_DECODER_FIRST_IMAGE_DECODED);
			ms_message("[msimx6vpu_h264_dec] filter notified of first image decoded");
		}
		
		ms_queue_put(d->filter->outputs[0], dupmsg(d->yuv_msg));
	} else if (result == -3) {
		ms_filter_notify_no_arg(d->filter, MS_VIDEO_DECODER_DECODING_ERRORS);
	} else if (result == -1) {
		d->need_reinit = TRUE;
	}
	ms_filter_unlock(d->filter);
}

void decoder_close_callback(void *v, int result) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)v;
	ms_message("[msimx6vpu_h264_dec] close callback");
}

void decoder_uninit_callback(void *v, int result) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)v;
	ms_message("[msimx6vpu_h264_dec] uninit callback");
	
	if (d->sps) freemsg(d->sps);
	if (d->pps) freemsg(d->pps);
	if (d->yuv_msg) freemsg(d->yuv_msg);
	if (d->yuvBufAllocator) ms_yuv_buf_allocator_free(d->yuvBufAllocator);
	ms_free(d);
}

static void msimx6vpu_h264_dec_init(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)ms_new(MSIMX6VPUH264DecData, 1);

	d->configure_done = FALSE;
	d->sps = NULL;
	d->pps = NULL;
	d->outbuf.w = 0;
	d->outbuf.h = 0;
	d->yuv_msg = NULL;
	    
	d->bitstream_size = 0x40000;
	d->handle = NULL;
	d->first_image_decoded = FALSE;
	d->decode_frame_command_queued = FALSE;
	d->shutdown = FALSE;
	d->yuvBufAllocator = ms_yuv_buf_allocator_new();
	d->avpf_enabled = FALSE;
	d->need_reinit = FALSE;
	
	if (!VpuWrapper::Instance()->IsVpuInitialized()) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_INIT, NULL, NULL, NULL));
	}
	
	d->packet_num = 0;
	d->filter = f;
	f->data = d;
	rfc3984_init(&d->unpacker);
}

static void msimx6vpu_h264_dec_preprocess(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	
	if (!d->handle) {
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_DECODER, d, &decoder_open_callback, NULL));
	}
}

static void msimx6vpu_h264_dec_process(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	mblk_t *im;
	MSQueue nalus;
	
	ms_filter_lock(f);
	ms_queue_init(&nalus);
	while ((im = ms_queue_get(f->inputs[0])) != NULL) {
		/*push the sps/pps given in sprop-parameter-sets if any*/
		if ((d->packet_num == 0) && d->sps && d->pps) {
			mblk_set_timestamp_info(d->sps, mblk_get_timestamp_info(im));
			mblk_set_timestamp_info(d->pps, mblk_get_timestamp_info(im));
			rfc3984_unpack(&d->unpacker, d->sps, &nalus);
			rfc3984_unpack(&d->unpacker, d->pps, &nalus);
			d->sps = NULL;
			d->pps = NULL;
		}
		rfc3984_unpack(&d->unpacker, im, &nalus);
		if (!ms_queue_empty(&nalus)) {
			uint8_t* bitstream = (uint8_t*) ms_malloc0(d->bitstream_size);
			bool_t need_reinit = FALSE;
			int size = nalusToFrame(d, &nalus, &need_reinit, bitstream);
			
			if ((d->need_reinit || need_reinit) && d->configure_done) {
				ms_message("[msimx6vpu_h264_dec] need reinit");
				d->need_reinit = FALSE;
				VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(CLOSE_DECODER, d, &decoder_close_callback, NULL));
				VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(OPEN_DECODER, d, &decoder_open_callback, NULL));
				VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(FILL_DECODER_BUFFER, d, &decoder_fill_buffer_callback, bitstream, size));
				VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(INIT_DECODER, d, &decoder_init_callback, NULL));
			}
			
			if (d->handle != NULL) {
				if (d->configure_done) {
					VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(FILL_DECODER_BUFFER, d, &decoder_fill_buffer_callback, bitstream, size));
				} else {
					VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(FILL_DECODER_BUFFER, d, &decoder_fill_buffer_callback, bitstream, size));
					VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(INIT_DECODER, d, &decoder_init_callback, NULL));
				}
			}
		}
		d->packet_num++;
	}
	if (d->configure_done && !d->decode_frame_command_queued) {
		d->decode_frame_command_queued = TRUE;
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(DECODE_FRAME, d, &decoder_decode_frame_callback, NULL));
	}
	ms_filter_unlock(f);
}


static void msimx6vpu_h264_dec_postprocess(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	
	d->shutdown = TRUE;
	if (d->handle) {
		ms_message("[msimx6vpu_h264_dec] shutting down decoder");
		VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(CLOSE_DECODER, d, &decoder_close_callback, NULL));
	}
}

static void msimx6vpu_h264_dec_uninit(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)f->data;
	
	rfc3984_uninit(&d->unpacker);
	
	VpuWrapper::Instance()->VpuQueueCommand(new VpuCommand(VPU_UNINIT, d, &decoder_uninit_callback, NULL));
	VpuWrapper::CheckUnInitStatus();
}

/******************************************************************************
 * Methods to configure the decoder                                           *
 *****************************************************************************/

static int msimx6vpu_h264_reset_first_image(MSFilter *f, void *data) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)f->data;
	
	ms_filter_lock(f);
	d->first_image_decoded = FALSE;
	ms_message("[msimx6vpu_h264_dec] first image resetted");
	ms_filter_unlock(f);
	
	return 0;
}

static int msimx6vpu_h264_get_vsize(MSFilter *f, void *data) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)f->data;
	MSVideoSize *vsize = (MSVideoSize *)data;
	
	ms_filter_lock(f);
	if (d->first_image_decoded == TRUE) {
		vsize->width = d->picwidth;
		vsize->height = d->picheight;
	} else {
		vsize->width = MS_VIDEO_SIZE_UNKNOWN_W;
		vsize->height = MS_VIDEO_SIZE_UNKNOWN_H;
	}
	ms_filter_unlock(f);
	
	return 0;
}

static int msimx6vpu_h264_enable_avpf(MSFilter *f, void *data) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)f->data;
	ms_filter_lock(f);
	d->avpf_enabled = *(bool_t*)data;
	ms_filter_unlock(f);
	return 0;
}

static MSFilterMethod msimx6vpu_h264_dec_methods[] = {
	{	MS_VIDEO_DECODER_RESET_FIRST_IMAGE_NOTIFICATION,	msimx6vpu_h264_reset_first_image	},
	{	MS_FILTER_GET_VIDEO_SIZE,							msimx6vpu_h264_get_vsize			},
	{ 	MS_VIDEO_DECODER_ENABLE_AVPF,                    	msimx6vpu_h264_enable_avpf		    },
	{	0,													NULL								}
};

/******************************************************************************
 * Definition of the decoder                                                  *
 *****************************************************************************/

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
	.postprocess = msimx6vpu_h264_dec_postprocess,
	.uninit = msimx6vpu_h264_dec_uninit,
	.methods = msimx6vpu_h264_dec_methods,
	.flags = MS_FILTER_IS_PUMP
};

/******************************************************************************
 * Init routine                                                               *
 *****************************************************************************/

extern "C" {
	MS2_PUBLIC void libmsimx6vpu_h264_init_dec(void) {
		ms_filter_register(&msimx6vpu_h264_dec_desc);
		ms_message("msimx6vpu-h264 decoder plugin registered.");
	}
}
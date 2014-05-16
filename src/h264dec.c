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

struct imx6vpu_data {
	DecHandle handle;
	unsigned long virt_buf_addr;
	PhysicalAddress phy_buf_addr;
	PhysicalAddress phy_ps_buf;
	PhysicalAddress phy_slice_buf;
	int phy_slicebuf_size;
	int picwidth;
	int picheight;
	int lastPicWidth;
	int lastPicHeight;
	int stride;
	int regfbcount;
	int minfbcount;
	FrameBuffer *fb;
	struct frame_buf **pfbpool;
};

typedef struct _MSIMX6VPUH264DecData {
	struct imx6vpu_data vpu;
	Rfc3984Context unpacker;
	MSPicture outbuf;
	MSVideoSize vsize;
	uint64_t last_decoded_frame;
	uint64_t last_error_reported_time;
	mblk_t *yuv_msg;
	mblk_t *sps;
	mblk_t *pps;
	uint8_t *bitstream;
	int bitstream_size;
	unsigned int packet_num;
	bool_t first_image_decoded;
	bool_t configure_done;
} MSIMX6VPUH264DecData;

#define PS_SAVE_SIZE		0x080000
#define STREAM_BUF_SIZE		0x200000

/******************************************************************************
 * VPU low level access functions                                            *
 *****************************************************************************/

static int msimx6vpu_h264_vpu_dec_open(struct imx6vpu_data d) {
	RetCode ret;
	vpu_versioninfo version;
	DecHandle handle = {0};
	DecOpenParam oparam = {0};
	vpu_mem_desc mem_desc = {0};
	vpu_mem_desc ps_mem_desc = {0};
	
	ret = vpu_Init(NULL);
	if (ret) {
		ms_error("[msimx6vpu_h264_dec] vpu_Init error: %d", ret);
		return -1;
	}

	ret = vpu_GetVersionInfo(&version);
	if (ret) {
		ms_error("[msimx6vpu_h264_dec] vpu_GetVersionInfo error: %d", ret);
		vpu_UnInit();
		return -1;
	}

	ms_message("[msimx6vpu_h264_dec] VPU firmware version: %d.%d.%d_r%d", version.fw_major, version.fw_minor, version.fw_release, version.fw_code);
	ms_message("[msimx6vpu_h264_dec] VPU library version: %d.%d.%d", version.lib_major, version.lib_minor, version.lib_release);
	
	mem_desc.size = STREAM_BUF_SIZE;
	ret = IOGetPhyMem(&mem_desc);
	if (ret) {
		ms_error("[msimx6vpu_h264_dec] IOGetPhyMem error: %d", ret);
		goto err;
	}

	ret = IOGetVirtMem(&mem_desc);
	if (ret <= 0) {
		ms_error("[msimx6vpu_h264_dec] IOGetVirtMem error: %d", ret);
		return -1;
	}
	
	ps_mem_desc.size = PS_SAVE_SIZE;
	ret = IOGetPhyMem(&ps_mem_desc);
	if (ret) {
		ms_error("[msimx6vpu_h264_dec] IOGetPhyMem error: %d", ret);
		goto err;
	}
	
	oparam.bitstreamFormat = STD_AVC;
	oparam.bitstreamBuffer = mem_desc.phy_addr;
	oparam.bitstreamBufferSize = STREAM_BUF_SIZE;
	oparam.psSaveBuffer = ps_mem_desc.phy_addr;
	oparam.psSaveBufferSize = PS_SAVE_SIZE;
	
	ret = vpu_DecOpen(&handle, &oparam);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_dec] vpu_DecOpen error: %d", ret);
		goto err;
	}

	d.virt_buf_addr = mem_desc.virt_uaddr;
	d.phy_buf_addr = mem_desc.phy_addr;
	d.phy_ps_buf = ps_mem_desc.phy_addr;
	d.handle = handle;
	return 0;
	
err:
	IOFreePhyMem(&ps_mem_desc);
	IOFreeVirtMem(&mem_desc);
	IOFreePhyMem(&mem_desc);
	vpu_UnInit();
	return -1;
}

static int msimx6vpu_h264_read(void *src, void *dest, int n) {
	//TODO: Find a better way
	memcpy(dest, src, n);
	
	return n;
}

static int msimx6vpu_h264_vpu_fill_buffer(struct imx6vpu_data d, void *bitstream) {
	RetCode ret;
	unsigned long space, target_addr;
	PhysicalAddress pa_read_ptr, pa_write_ptr;
	int size, room, nread;
	bool_t eof = FALSE;
	
	ret = vpu_DecGetBitstreamBuffer(d.handle, &pa_read_ptr, &pa_write_ptr, &space);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_dec] vpu_DecGetBitstreamBuffer error: %d", ret);
		return -1;
	}

	if (space <= 0) {
		ms_warning("[msimx6vpu_h264_dec] space %lu <= 0", space);
		return 0;
	}
	size = space;
	
	target_addr = d.virt_buf_addr + (pa_write_ptr - d.phy_buf_addr);
	if (target_addr + size > d.virt_buf_addr + STREAM_BUF_SIZE) {
		room = (d.virt_buf_addr + STREAM_BUF_SIZE) - target_addr;
		nread = msimx6vpu_h264_read(bitstream, (void *)target_addr, room);
		if (nread <= 0) {
			ms_warning("[msimx6vpu_h264_dec] EOF or error");
			if (nread < 0) {
				if (nread == -EAGAIN) {
					return 0;
				}

				ms_error("[msimx6vpu_h264_dec] nread %d < 0", nread);
				return -1;
			}
			eof = TRUE;
		} else {
			if (nread != room) {
				goto update;
			}
			
			space = nread;
			nread = msimx6vpu_h264_read(bitstream, (void *)d.virt_buf_addr, (size - room));
			if (nread <= 0) {
				ms_warning("[msimx6vpu_h264_dec] EOF or error");
				if (nread < 0) {
					if (nread == -EAGAIN) {
						return 0;
					}

					ms_error("[msimx6vpu_h264_dec] nread %d < 0", nread);
					return -1;
				}
				eof = TRUE;
			}

			nread += space;
		}
	} else {
		nread = msimx6vpu_h264_read(bitstream, (void *)target_addr, size);
		if (nread <= 0) {
			ms_warning("[msimx6vpu_h264_dec] EOF or error");
			if (nread < 0) {
				if (nread == -EAGAIN) {
					return 0;
				}

				ms_error("[msimx6vpu_h264_dec] nread %d < 0", nread);
				return -1;
			}
			eof = TRUE;
		}
	}
	
update:	
	if (!eof) {
		ret = vpu_DecUpdateBitstreamBuffer(d.handle, nread);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_dec] vpu_DecUpdateBitstreamBuffer error: %d", ret);
			return -1;
		}
	} else {
		ret = vpu_DecUpdateBitstreamBuffer(d.handle, STREAM_BUF_SIZE);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_dec] vpu_DecUpdateBitstreamBuffer error: %d", ret);
			return -1;
		}
	}
	
	
	return 0;
}

static void msimx6vpu_h264_vpu_free_fb(struct imx6vpu_data d) {
	ms_free(d.fb);
}

static int msimx6vpu_h264_vpu_alloc_fb(struct imx6vpu_data d) {
	DecBufInfo bufinfo;
	RetCode ret;
	
	d.fb = ms_malloc0(d.regfbcount * sizeof(FrameBuffer));
	if (d.fb == NULL) {
		ms_error("[msimx6vpu_h264_dec] failed to allocate fb");
		return -1;
	}
	
	bufinfo.avcSliceBufInfo.bufferBase = d.phy_slice_buf;
	bufinfo.avcSliceBufInfo.bufferSize = d.phy_slicebuf_size;

	bufinfo.maxDecFrmInfo.maxMbX = d.stride / 16;
	bufinfo.maxDecFrmInfo.maxMbY = d.picheight / 16;
	bufinfo.maxDecFrmInfo.maxMbNum = d.stride * d.picheight / 256;
	
	ret = vpu_DecRegisterFrameBuffer(d.handle, d.fb, d.regfbcount, d.stride, &bufinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_dec] vpu_DecRegisterFrameBuffer error: %d", ret);
		return -1;
	}
	
	return 0;
}

static int msimx6vpu_h264_vpu_dec_init(struct imx6vpu_data d) {
	RetCode ret;
	DecInitialInfo initinfo = {0};
	vpu_mem_desc slice_mem_desc = {0};
	
	vpu_DecSetEscSeqInit(d.handle, 1);
	ret = vpu_DecGetInitialInfo(d.handle, &initinfo);
	vpu_DecSetEscSeqInit(d.handle, 0);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_dec] vpu_DecGetInitialInfo error: %d, errorcode: %ld", ret, initinfo.errorcode);
		return -1;
	}

	if (initinfo.streamInfoObtained) {
		ms_message("[msimx6vpu_h264_dec] H.264 Profile: %d Level: %d Interlace: %d", initinfo.profile, initinfo.level, initinfo.interlace);
		if (initinfo.aspectRateInfo) {
			int aspect_ratio_idc;
			int sar_width, sar_height;

			if ((initinfo.aspectRateInfo >> 16) == 0) {
				aspect_ratio_idc = (initinfo.aspectRateInfo & 0xFF);
				ms_message("[msimx6vpu_h264_dec] aspect_ratio_idc: %d", aspect_ratio_idc);
			} else {
				sar_width = (initinfo.aspectRateInfo >> 16) & 0xFFFF;
				sar_height = (initinfo.aspectRateInfo & 0xFFFF);
				ms_message("[msimx6vpu_h264_dec] sar_width: %d, sar_height: %d", sar_width, sar_height);
			}
		} else {
			ms_message("[msimx6vpu_h264_dec] aspect ratio is not present");
		}
	}
	
	d.lastPicWidth = initinfo.picWidth;
	d.lastPicHeight = initinfo.picHeight;
	ms_message("[msimx6vpu_h264_dec] decoder: width = %d, height = %d, frameRateRes = %lu, frameRateDiv = %lu, count = %u", initinfo.picWidth, initinfo.picHeight, initinfo.frameRateRes, initinfo.frameRateDiv, initinfo.minFrameBufferCount);
	
	// Let's use 2 more buffers than recommended
	if (initinfo.interlace) {
		d.regfbcount = d.minfbcount + 4;
	} else {
		d.regfbcount = d.minfbcount + 2;
	}
	d.picwidth = ((initinfo.picWidth + 15) & ~15);
	d.picheight = ((initinfo.picHeight + 31) & ~(31));
	
	if ((d.picwidth == 0) || (d.picheight == 0)) {
		ms_error("[msimx6vpu_h264_dec] width=%d, height=%d", d.picwidth, d.picheight);
		return -1;
	}
	
	/* worstSliceSize is in kilo-byte unit */
	d.phy_slicebuf_size = initinfo.worstSliceSize * 1024;
	d.stride = d.picwidth;
	
	
	slice_mem_desc.size = d.phy_slicebuf_size;
	ret = IOGetPhyMem(&slice_mem_desc);
	if (ret) {
		ms_error("[msimx6vpu_h264_dec] IOGetPhyMem error: %d", ret);
		return -1;
	}
	d.phy_slice_buf = slice_mem_desc.phy_addr;
	
	if (msimx6vpu_h264_vpu_alloc_fb(d) < 0) {
		ms_error("[msimx6vpu_h264_dec] Unable to allocate frame buffers");
		IOFreePhyMem(&slice_mem_desc);
		return -1;
	}

	return 0;
}


static void msimx6vpu_h264_vpu_dec_start(struct imx6vpu_data d) {
	ms_error("[msimx6vpu_h264_dec] no implemented yet");
}


static int msimx6vpu_h264_vpu_dec_close(struct imx6vpu_data d) {
	RetCode ret;

	ret = vpu_DecClose(d.handle);
	if (ret == RETCODE_FRAME_NOT_COMPLETE) {
		vpu_SWReset(d.handle, 0);
		ret = vpu_DecClose(d.handle);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_dec] vpu_DecClose error: %d", ret);
			return -1;
		}
	}
	
	return 0;
}

/******************************************************************************
 * Implementation of the decoder                                              *
 *****************************************************************************/

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
			ms_message("SPS changed ! %i,%i",(int)msgdsize(sps),(int)msgdsize(d->sps));
			update_sps(d,sps);
			update_pps(d,NULL);
		}
	} else {
		ms_message("Receiving first SPS");
		update_sps(d,sps);
	}
	return ret;
}

static bool_t check_pps_change(MSIMX6VPUH264DecData *d, mblk_t *pps){
	bool_t ret=FALSE;
	if (d->pps){
		ret=(msgdsize(pps)!=msgdsize(d->pps)) || (memcmp(d->pps->b_rptr,pps->b_rptr,msgdsize(pps))!=0);
		if (ret) {
			ms_message("PPS changed ! %i,%i",(int)msgdsize(pps),(int)msgdsize(d->pps));
			update_pps(d,pps);
		}
	}else {
		ms_message("Receiving first PPS");
		update_pps(d,pps);
	}
	return ret;
}


static void enlarge_bitstream(MSIMX6VPUH264DecData *d, int new_size){
	d->bitstream_size = new_size;
	d->bitstream = ms_realloc(d->bitstream, d->bitstream_size);
}

static int nalusToFrame(MSIMX6VPUH264DecData *d, MSQueue *naluq, bool_t *new_sps_pps){
	mblk_t *im;
	uint8_t *dst=d->bitstream,*src,*end;
	int nal_len;
	bool_t start_picture=TRUE;
	uint8_t nalu_type;
	*new_sps_pps=FALSE;
	end=d->bitstream+d->bitstream_size;
	while((im=ms_queue_get(naluq))!=NULL){
		src=im->b_rptr;
		nal_len=im->b_wptr-src;
		if (dst+nal_len+100>end){
			int pos=dst-d->bitstream;
			enlarge_bitstream(d, d->bitstream_size+nal_len+100);
			dst=d->bitstream+pos;
			end=d->bitstream+d->bitstream_size;
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
	return dst-d->bitstream;
}

static void msimx6vpu_h264_dec_init(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)ms_new(MSIMX6VPUH264DecData, 1);
	d->configure_done = FALSE;
	d->sps = NULL;
	d->pps = NULL;
	d->outbuf.w = 0;
	d->outbuf.h = 0;
	d->vsize.width = MS_VIDEO_SIZE_VGA_W;
	d->vsize.height = MS_VIDEO_SIZE_VGA_H;
	
	if (msimx6vpu_h264_vpu_dec_open(d->vpu) < 0) {
		ms_error("[msimx6vpu_h264_dec] failed to open the decoder");
	}
	
	d->bitstream_size = 65536;
	d->bitstream = ms_malloc0(d->bitstream_size);
	
	rfc3984_init(&d->unpacker);
	d->packet_num = 0;
	d->last_decoded_frame = 0;
	d->last_error_reported_time = 0;
	f->data = d;
}

static void msimx6vpu_h264_dec_preprocess(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	d->first_image_decoded = FALSE;
}

static void msimx6vpu_h264_dec_process(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	mblk_t *im;
	MSQueue nalus;
	
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
			int size;
			bool_t need_reinit=FALSE;
			
			size = nalusToFrame(d, &nalus, &need_reinit);
			ms_message("[msimx6vpu_h264_dec] size read: %i", size);
			if (!d->configure_done) {
				if (msimx6vpu_h264_vpu_fill_buffer(d->vpu, d->bitstream) < 0) {
					ms_error("[msimx6vpu_h264_dec] failed to fill the vpu's bitstream buffer");
				}
				if (msimx6vpu_h264_vpu_dec_init(d->vpu) < 0) {
					ms_error("[msimx6vpu_h264_dec] failed to initialise the decoder");
				}
				d->configure_done = TRUE;
			}
			
			msimx6vpu_h264_vpu_dec_start(d->vpu);
		}
	}
}

/*
static void msimx6vpu_h264_dec_postprocess(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	
}
*/

static void msimx6vpu_h264_dec_uninit(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)f->data;
	msimx6vpu_h264_vpu_dec_close(d->vpu);
	msimx6vpu_h264_vpu_free_fb(d->vpu);
	rfc3984_uninit(&d->unpacker);
	if (d->yuv_msg) freemsg(d->yuv_msg);
	if (d->sps) freemsg(d->sps);
	if (d->pps) freemsg(d->pps);
	ms_free(d->bitstream);
	ms_free(d);
}

/******************************************************************************
 * Methods to configure the decoder                                           *
 *****************************************************************************/

static int msimx6vpu_h264_reset_first_image(MSFilter *f, void *data) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)f->data;
	d->first_image_decoded = FALSE;
	return 0;
}

static MSFilterMethod msimx6vpu_h264_dec_methods[] = {
	{	MS_VIDEO_DECODER_RESET_FIRST_IMAGE_NOTIFICATION,	msimx6vpu_h264_reset_first_image	},
	{	0,							NULL					}
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
	.postprocess = NULL,//msimx6vpu_h264_dec_postprocess,
	.uninit = msimx6vpu_h264_dec_uninit,
	.methods = msimx6vpu_h264_dec_methods,
	.flags = 0
};

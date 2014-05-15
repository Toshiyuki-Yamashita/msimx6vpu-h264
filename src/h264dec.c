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

static int msimx6vpu_h264_vpu_alloc_fb(struct imx6vpu_data d) {
	DecBufInfo bufinfo;
	RetCode ret;
	
	d.fb = calloc(d.regfbcount, sizeof(FrameBuffer));
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

/*
static void msimx6vpu_h264_vpu_dec_start(struct imx6vpu_data d) {
	
}
*/

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

static void msimx6vpu_h264_dec_init(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData *)ms_new(MSIMX6VPUH264DecData, 1);
	d->sps = NULL;
	d->pps = NULL;
	d->outbuf.w = 0;
	d->outbuf.h = 0;
	d->vsize.width = MS_VIDEO_SIZE_VGA_W;
	d->vsize.height = MS_VIDEO_SIZE_VGA_H;
	
	msimx6vpu_h264_vpu_dec_open(d->vpu);
	
	rfc3984_init(&d->unpacker);
	d->packet_num = 0;
	d->last_decoded_frame = 0;
	d->last_error_reported_time = 0;
	d->configure_done = FALSE;
	f->data = d;
}

static void msimx6vpu_h264_dec_preprocess(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	d->first_image_decoded = FALSE;
}

static void msimx6vpu_h264_dec_process(MSFilter *f) {
	MSIMX6VPUH264DecData *d = (MSIMX6VPUH264DecData*)f->data;
	if (!d->configure_done) {
		//TODO: Get bitstream first and pass it to following function
		if (msimx6vpu_h264_vpu_fill_buffer(d->vpu, NULL) < 0) {
			
		}
		if (msimx6vpu_h264_vpu_dec_init(d->vpu) < 0) {
			
		}
		d->configure_done = TRUE;
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
	{	0,							NULL				}
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

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
#include "vpu.h"

#define MS_H264_CONF(required_bitrate, bitrate_limit, resolution, fps) \
	{ required_bitrate, bitrate_limit, { MS_VIDEO_SIZE_ ## resolution ## _W, MS_VIDEO_SIZE_ ## resolution ## _H }, fps, NULL }

static const MSVideoConfiguration h264_conf_list[] = {
#if defined(ANDROID) || (TARGET_OS_IPHONE == 1) || defined(__arm__)
	MS_H264_CONF( 170000,  512000, VGA,  20),
	MS_H264_CONF( 128000,  170000, QVGA, 15),
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

/******************************************************************************
 * Video starter                                                             *
 *****************************************************************************/

/* the goal of this small object is to tell when to send I frames at startup:
at 2 and 4 seconds*/
typedef struct VideoStarter {
	uint64_t next_time;
	int i_frame_count;
} VideoStarter;

static void video_starter_init(VideoStarter *vs) {
	vs->next_time = 0;
	vs->i_frame_count = 0;
}

static void video_starter_first_frame(VideoStarter *vs, uint64_t curtime) {
	vs->next_time = curtime + 2000;
}

static bool_t video_starter_need_i_frame(VideoStarter *vs, uint64_t curtime) {
	if (vs->next_time == 0) return FALSE;
	if (curtime >= vs->next_time) {
		vs->i_frame_count++;
		if (vs->i_frame_count == 1) {
			vs->next_time += 2000;
		} else {
			vs->next_time = 0;
		}
		return TRUE;
	}
	return FALSE;
}

/******************************************************************************
 * Encoder structures                                                        *
 *****************************************************************************/

typedef struct _MSIMX6VPUH264EncData {
	DecHandle handle;
	vpu_mem_desc bitstream_mem;
	int src_width;
	int src_height;
	int src_buffer_index;
	int regfbcount;
	int minfbcount;
	FrameBuffer *fbs;
	IMX6VPUFrameBuffer **fbpool;
	bool_t enc_frame_started;
	bool_t frame_ready_for_encoder;
	Rfc3984Context *packer;
	MSVideoConfiguration vconf;
	const MSVideoConfiguration *vconf_list;
	bool_t generate_keyframe;
	bool_t configure_done;
	unsigned int packet_num;
	VideoStarter starter;
	int mode;
} MSIMX6VPUH264EncData;

#define STREAM_BUF_SIZE		0x200000
#define STREAM_READ_SIZE	512 * 8
#define KBPS				1000

/******************************************************************************
 * VPU low level access functions                                            *
 *****************************************************************************/

static int msimx6vpu_h264_vpu_enc_open(MSIMX6VPUH264EncData *d) {
	RetCode ret;
	EncHandle handle = {0};
	EncOpenParam oparam = {0};
	EncSliceMode slicemode = {0};
	
	msimx6vpu_init("enc");
	
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
	
	slicemode.sliceMode = 1;
	slicemode.sliceSizeMode = 0;
	slicemode.sliceSize = ms_get_payload_max_size() - 1;
	
	oparam.bitstreamFormat = STD_AVC;
	oparam.bitstreamBuffer = d->bitstream_mem.phy_addr;
	oparam.bitstreamBufferSize = STREAM_BUF_SIZE;
	oparam.picWidth = d->vconf.vsize.width;
	oparam.picHeight = d->vconf.vsize.height;
	oparam.frameRateInfo = (int)d->vconf.fps;
	oparam.bitRate = d->vconf.required_bitrate / KBPS;
	oparam.ringBufferEnable = 1;
	oparam.chromaInterleave = 0;
	oparam.slicemode = slicemode;
	oparam.rcIntraQp = -1;
	oparam.userGamma = (0.75*32768);
	oparam.RcIntervalMode = 1;
	oparam.EncStdParam.avcParam.paraset_refresh_en = 1;
	
	ret = vpu_EncOpen(&handle, &oparam);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncOpen error: %d", ret);
		return -1;
	}

	d->handle = handle;
	ms_message("[msimx6vpu_h264_enc] vpu encoder opened");
	return 0;
	
err:
	IOFreeVirtMem(&d->bitstream_mem);
	IOFreePhyMem(&d->bitstream_mem);
	return -1;
}

static int msimx6vpu_h264_vpu_alloc_fb(MSIMX6VPUH264EncData *d) {
	RetCode ret;
	int i, err;
	int enc_fbwidth, enc_fbheight, enc_stride;
	int src_fbwidth, src_fbheight, src_stride;
	EncExtBufInfo extbufinfo = {0};
	PhysicalAddress subSampBaseA = 0;
	PhysicalAddress subSampBaseB = 0;
	
	enc_fbwidth = (d->vconf.vsize.width + 15) & ~15;
	enc_fbheight = (d->vconf.vsize.height + 15) & ~15;
	src_fbwidth = (d->src_width + 15) & ~15;
	src_fbheight = (d->src_height + 15) & ~15;
	enc_stride = enc_fbwidth;
	src_stride = src_fbwidth;
	
	d->fbpool = (IMX6VPUFrameBuffer**) ms_malloc0(d->regfbcount * sizeof(IMX6VPUFrameBuffer *));
	if (d->fbpool == NULL) {
		ms_error("[msimx6vpu_h264_enc] failed to allocate fbpool");
		return -1;
	}
	
	d->fbs = ms_malloc0(d->regfbcount * sizeof(FrameBuffer));
	if (d->fbs == NULL) {
		ms_error("[msimx6vpu_h264_enc] failed to allocate fb");
		return -1;
	}
	
	for (i = 0; i < d->regfbcount - 1; i++) { // Don't do it for src buffer
		d->fbpool[i] = (IMX6VPUFrameBuffer*) ms_malloc0(sizeof(IMX6VPUFrameBuffer));
		
		memset(&(d->fbpool[i]->desc), 0, sizeof(vpu_mem_desc));
		d->fbpool[i]->desc.size = 3 * enc_fbwidth * enc_fbheight / 2;
		err = IOGetPhyMem(&d->fbpool[i]->desc);
		if (err) {
			ms_error("[msimx6vpu_h264_enc] error getting phymem for buffer %i", i);
			return -1;
		}
		d->fbpool[i]->addrY = d->fbpool[i]->desc.phy_addr;
		d->fbpool[i]->addrCb = d->fbpool[i]->addrY + (enc_fbwidth * enc_fbheight);
		d->fbpool[i]->addrCr = d->fbpool[i]->addrCb + (enc_fbwidth * enc_fbheight / 4);
		d->fbpool[i]->strideY = enc_fbwidth;
		d->fbpool[i]->strideC = enc_fbwidth / 2;
		
		d->fbs[i].myIndex = i;
		d->fbs[i].bufY = d->fbpool[i]->addrY;
		d->fbs[i].bufCb = d->fbpool[i]->addrCb;
		d->fbs[i].bufCr = d->fbpool[i]->addrCr;
		d->fbs[i].strideY = d->fbpool[i]->strideY;
		d->fbs[i].strideC = d->fbpool[i]->strideC;
		d->fbpool[i]->fb = &(d->fbs[i]);
		
		d->fbpool[i]->desc.virt_uaddr = IOGetVirtMem(&(d->fbpool[i]->desc));
		if (d->fbpool[i]->desc.virt_uaddr <= 0) {
			ms_error("[msimx6vpu_h264_enc] error getting virt mem for buffer %i", i);
			return -1;
		}
	}

	subSampBaseA = d->fbs[d->minfbcount].bufY;
	subSampBaseB = d->fbs[d->minfbcount + 1].bufY;
	
	ret = vpu_EncRegisterFrameBuffer(d->handle, d->fbs, d->regfbcount, enc_stride, src_stride, subSampBaseA, subSampBaseB, &extbufinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncRegisterFrameBuffer error: %d", ret);
		return -1;
	}
	
	// Source buffer
	i = d->src_buffer_index;
	d->fbpool[i] = (IMX6VPUFrameBuffer*) ms_malloc0(sizeof(IMX6VPUFrameBuffer));
	memset(&(d->fbpool[i]->desc), 0, sizeof(vpu_mem_desc));
	d->fbpool[i]->desc.size = 3 * src_fbwidth * src_fbheight / 2;
	err = IOGetPhyMem(&d->fbpool[i]->desc);
	if (err) {
		ms_error("[msimx6vpu_h264_enc] error getting phymem for src buffer");
		return -1;
	}
	d->fbpool[i]->addrY = d->fbpool[i]->desc.phy_addr;
	d->fbpool[i]->addrCb = d->fbpool[i]->addrY + (src_fbwidth * src_fbheight);
	d->fbpool[i]->addrCr = d->fbpool[i]->addrCb + (src_fbwidth * src_fbheight / 4);
	d->fbpool[i]->strideY = src_fbwidth;
	d->fbpool[i]->strideC = src_fbwidth / 2;
	
	d->fbs[i].myIndex = i;
	d->fbs[i].bufY = d->fbpool[i]->addrY;
	d->fbs[i].bufCb = d->fbpool[i]->addrCb;
	d->fbs[i].bufCr = d->fbpool[i]->addrCr;
	d->fbs[i].strideY = d->fbpool[i]->strideY;
	d->fbs[i].strideC = d->fbpool[i]->strideC;
	d->fbpool[i]->fb = &(d->fbs[i]);
		
	d->fbpool[i]->desc.virt_uaddr = IOGetVirtMem(&(d->fbpool[i]->desc));
	if (d->fbpool[i]->desc.virt_uaddr <= 0) {
		ms_error("[msimx6vpu_h264_enc] error getting virt mem for src buffer");
		return -1;
	}
		
	return 0;
}

static int msimx6vpu_h264_vpu_enc_init(MSIMX6VPUH264EncData *d) {
	RetCode ret;
	int err;
	EncInitialInfo initinfo = {0};
	EncHeaderParam header = {0};
	
	if (msimx6vpu_isBusy()) {
		ms_warning("[msimx6vpu_h264_enc] vpu is busy, enc init delayed");
		return -1;
	}
	
	msimx6vpu_lockVPU();
	ret = vpu_EncGetInitialInfo(d->handle, &initinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncGetInitialInfo error: %d", ret);
		msimx6vpu_unlockVPU();
		return -2;
	}

	d->minfbcount = initinfo.minFrameBufferCount;
	d->regfbcount = d->minfbcount + 2 + 1; // 1 is for the src buffer
	d->src_buffer_index = d->regfbcount - 1;
	
	ms_message("[msimx6vpu_h264_enc] allocating frame buffers");
	if (msimx6vpu_h264_vpu_alloc_fb(d) < 0) {
		ms_message("[msimx6vpu_h264_enc] error allocating frame buffers");
		return -1;
	}
	ms_message("[msimx6vpu_h264_enc] frame buffer allocated");

	header.headerType = SPS_RBSP;
	ret = vpu_EncGiveCommand(d->handle, ENC_PUT_AVC_HEADER, &header);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncGiveCommand ENC_PUT_AVC_HEADER SPS_RBSP error: %i", ret);
		return -1;
	}
	
	memset(&header, 0, sizeof(EncHeaderParam));
	header.headerType = PPS_RBSP;
	ret = vpu_EncGiveCommand(d->handle, ENC_PUT_AVC_HEADER, &header);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncGiveCommand ENC_PUT_AVC_HEADER PPS_RBSP error: %i", ret);
		return -1;
	}
	
	msimx6vpu_unlockVPU();
	
	return 0;
}

static int frame_to_nalus(MSQueue *nalus, void *bitstream, int size, void *bitstream2, int size2) {
	uint8_t *ptr = bitstream, *bs_end = bitstream + size;
	int nal_size;
	bool_t loop_needed, has_looped;
	mblk_t *temp, *m;
	int tmp_buffer_size = ms_get_payload_max_size() - 1;
	
	loop_needed = bitstream2 != NULL && size2 > 0;
	has_looped = !loop_needed;
	
	temp = allocb(tmp_buffer_size, 0);
	nal_size = 0;
	
	if (ptr + 3 < bs_end && ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1) {
		ptr += 4; // Skip NAL marker 0001
	}
	
	do {
		if (ptr >= bs_end) {
			break;
		}
		
		if (ptr + 2 < bs_end && ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 1) {
			if (nalus) {
				m = allocb(nal_size, 0);
				memcpy(m->b_wptr, temp->b_rptr, nal_size);
				m->b_wptr += nal_size;
				ms_queue_put(nalus, dupmsg(m));
				freemsg(m);
			}
			freemsg(temp);
			temp = allocb(tmp_buffer_size, 0);
			nal_size = 0;
			
			ptr += 3; // Skip NAL marker 001
			continue;
		}
		
		if (ptr + 2 < bs_end && ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 3) {
			memcpy(temp->b_wptr, ptr, 2);
			temp->b_wptr += 2;
			nal_size += 2;
			
			ptr += 3;
			continue;
		}
		
		memcpy(temp->b_wptr, ptr, 1);
		temp->b_wptr += 1;
		nal_size += 1;
		ptr += 1;
	} while (ptr < bs_end);
	
	if (!has_looped) {
		bs_end = bitstream2 + size2;
		ptr = bitstream2;
		
		do {
			if (ptr >= bs_end) {
				break;
			}
			
			if (ptr + 2 < bs_end && ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 1) {
				if (nalus) {
					m = allocb(nal_size, 0);
					memcpy(m->b_wptr, temp->b_rptr, nal_size);
					m->b_wptr += nal_size;
					ms_queue_put(nalus, dupmsg(m));
					freemsg(m);
				}
				freemsg(temp);
				temp = allocb(tmp_buffer_size, 0);
				nal_size = 0;
				
				ptr += 3; // Skip NAL marker 001
				continue;
			}
			
			if (ptr + 2 < bs_end && ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 3) {
				memcpy(temp->b_wptr, ptr, 2);
				temp->b_wptr += 2;
				nal_size += 2;
				
				ptr += 3;
				continue;
			}
			
			memcpy(temp->b_wptr, ptr, 1);
			temp->b_wptr += 1;
			nal_size += 1;
			ptr += 1;
		} while (ptr < bs_end);
		
		if (nalus) {
			m = allocb(nal_size, 0);
			memcpy(m->b_wptr, temp->b_rptr, nal_size);
			m->b_wptr += nal_size;
			ms_queue_put(nalus, dupmsg(m));
			freemsg(m);
		}
		freemsg(temp);
	} else {
		if (nalus) {
			m = allocb(nal_size, 0);
			memcpy(m->b_wptr, temp->b_rptr, nal_size);
			m->b_wptr += nal_size;
			ms_queue_put(nalus, dupmsg(m));
			freemsg(m);
		}
		freemsg(temp);
	}
	
	return 0;
}

static int msimx6vpu_h264_vpu_read_ring_buffer(MSIMX6VPUH264EncData *d, int default_size, MSQueue *nalus) {
	RetCode ret;
	PhysicalAddress pa_read_ptr, pa_write_ptr;
	uint32_t size, target_addr;
	int space, room;
	
	ret = vpu_EncGetBitstreamBuffer(d->handle, &pa_read_ptr, &pa_write_ptr, (uint32_t *)&size);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncGetBitstreamBuffer error: %d", ret);
		return -1;
	}
	
	if (size < 0) {
		ms_warning("[msimx6vpu_h264_enc] size %i < 0 !", size);
		return 0;
	}
	
	if (default_size > 0) {
		if (size < default_size) {
			return 0;
		}
		space = default_size;
	} else {
		space = size;
	}
	
	if (space > 0) {
		target_addr = d->bitstream_mem.virt_uaddr + (pa_read_ptr - d->bitstream_mem.phy_addr);
		if (target_addr + space > d->bitstream_mem.virt_uaddr + STREAM_BUF_SIZE) {
			room = d->bitstream_mem.virt_uaddr + STREAM_BUF_SIZE - target_addr;
			frame_to_nalus(nalus, (void *)target_addr, room, (void *)d->bitstream_mem.virt_uaddr, space - room);
		} else {
			frame_to_nalus(nalus, (void *)target_addr, space, NULL, 0);
		}
		
		ret = vpu_EncUpdateBitstreamBuffer(d->handle, space);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_enc] vpu_EncUpdateBitstreamBuffer error: %d", ret);
			return -1;
		}
	}
	
	return space;
}

static int msimx6vpu_h264_vpu_enc_start(MSFilter *f, MSQueue *nalus) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	RetCode ret;
	EncParam params = {0};
	EncOutputInfo outinfo = {0};
	mblk_t *m;
	
	if (!d->enc_frame_started && !msimx6vpu_isBusy()) {
		msimx6vpu_lockVPU();
		params.sourceFrame = &d->fbs[d->src_buffer_index];
		if (d->generate_keyframe) {
			params.forceIPicture = 1;
			d->generate_keyframe = FALSE;
		}
	
		ret = vpu_EncStartOneFrame(d->handle, &params);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_enc] vpu_EncStartOneFrame failed with error: %d", ret);
			msimx6vpu_unlockVPU();
			return -1;
		}
		d->enc_frame_started = TRUE;
	}
	
	if (!d->enc_frame_started) {
		return -1;
	}
	
	if (vpu_IsBusy()) {
		return -1;
	}
	
	ret = vpu_EncGetOutputInfo(d->handle, &outinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_enc] vpu_EncGetOutputInfo error: %d", ret);
		msimx6vpu_unlockVPU();
		return -1;
	}
	
	ms_message("[msimx6vpu_h264_enc] frame encoding complete, %i slices, type of frame %i", outinfo.numOfSlices, outinfo.picType);
	if (outinfo.bitstreamWrapAround) {
		ms_warning("[msimx6vpu_h264_enc] encoder reports a buffer wrap around!");
	}
	
	d->enc_frame_started = FALSE;
	msimx6vpu_unlockVPU();
	
	if (outinfo.skipEncoded) {
		ms_warning("[msimx6vpu_h264_enc] skip encoding one frame");
		return -1;
	}
	
	return msimx6vpu_h264_vpu_read_ring_buffer(d, 0, nalus);
}

static void msimx6vpu_h264_vpu_enc_close(MSIMX6VPUH264EncData *d) {
	RetCode ret;
	EncOutputInfo outinfo = {0};
	int i;
	
	if (d->enc_frame_started) {
		ms_warning("[msimx6vpu_h264_enc] encoder running, let's finish the operation first");
		d->enc_frame_started = FALSE;
		vpu_SWReset(d->handle, 0);
		msimx6vpu_unlockVPU();
	}
	
	ret = vpu_EncClose(d->handle);
	if (ret == RETCODE_FRAME_NOT_COMPLETE) {
		vpu_SWReset(d->handle, 0);
		ret = vpu_EncClose(d->handle);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_enc] vpu_EncClose error: %d", ret);
		}
	}
	ms_message("[msimx6vpu_h264_enc] encoder closed");
	
	if (d->handle != NULL) {
		IOFreeVirtMem(&d->bitstream_mem);
		IOFreePhyMem(&d->bitstream_mem);
		if (d->configure_done) {
			for (i = 0; i < d->regfbcount; i++) {
				IOFreeVirtMem(&d->fbpool[i]->desc);
				IOFreePhyMem(&d->fbpool[i]->desc);
			}
			ms_free(d->fbpool);
			ms_free(d->fbs);
		}
		ms_message("[msimx6vpu_h264_enc] memory freed");
	}
	
	msimx6vpu_close("enc");
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
	d->packet_num = 0;
	d->packer = NULL;
	d->src_buffer_index = -1;
	d->enc_frame_started = FALSE;
	d->frame_ready_for_encoder = FALSE;
	d->src_width = 0;
	d->src_height = 0;
	d->mode = 0;
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
	rfc3984_set_mode(d->packer, d->mode);
	rfc3984_enable_stap_a(d->packer, FALSE);
	video_starter_init(&d->starter);
}

static void msimx6vpu_h264_vpu_fill_buffer(MSIMX6VPUH264EncData *d, MSPicture *pic) {
	int offset;
	IMX6VPUFrameBuffer *framebuff;
	MSVideoSize roi = {0};
	uint8_t *dest_planes[3];
	int dest_strides[3];
	
	framebuff = d->fbpool[d->src_buffer_index];
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
}

static void msimx6vpu_h264_enc_process(MSFilter *f) {
	MSIMX6VPUH264EncData *d = (MSIMX6VPUH264EncData*)f->data;
	uint32_t ts = f->ticker->time * 90LL;
	MSQueue nalus;
	mblk_t *im;
	MSPicture pic;
	
	if (!d->handle) {
		ms_error("[msimx6vpu_h264_enc] encoder not opened");
		ms_queue_flush(f->inputs[0]);
		return;
	}
	
	ms_filter_lock(f);
	ms_queue_init(&nalus);
	while ((im = ms_queue_get(f->inputs[0])) != NULL) {
		if (ms_yuv_buf_init_from_mblk(&pic, im) == 0) {
			/* send I frame 2 seconds and 4 seconds after the beginning */
			if (video_starter_need_i_frame(&d->starter, f->ticker->time)) {
				ms_message("[msimx6vpu_h264_enc] asking encoder for keyframe");
				d->generate_keyframe = TRUE;
			}
			
			d->src_width = pic.w;
			d->src_height = pic.h;
			
			if (!d->configure_done) {
				if (msimx6vpu_h264_vpu_enc_init(d) < 0) {
					ms_error("[msimx6vpu_h264_enc] failed to initialise the encoder");
				} else {
					ms_message("[msimx6vpu_h264_enc] encoder initialised");
					d->configure_done = TRUE;
				}
			}
			
			if (d->configure_done && !d->enc_frame_started && !d->frame_ready_for_encoder) {
				d->frame_ready_for_encoder = TRUE;
				msimx6vpu_h264_vpu_fill_buffer(d, &pic);
			}
		}
		freemsg(im);
	}
	
	if (d->configure_done && d->frame_ready_for_encoder) {
		if (msimx6vpu_h264_vpu_enc_start(f, &nalus) >= 0) {
			d->frame_ready_for_encoder = FALSE;
			if (!ms_queue_empty(&nalus)) {
				rfc3984_pack(d->packer, &nalus, f->outputs[0], ts);
				if (d->packet_num == 0) {
					ms_message("[msimx6vpu_h264_enc] first frame encoded");
					video_starter_first_frame(&d->starter, f->ticker->time);
				}
				d->packet_num++;
			}
		}
	}
	ms_filter_unlock(f);
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
		ms_message("[msimx6vpu_h264_enc] reconfiguring encoder");
		ms_filter_lock(f);
		d->configure_done = FALSE;
		
		if (d->enc_frame_started) {
			d->enc_frame_started = FALSE;
			d->frame_ready_for_encoder = FALSE;
		}
		msimx6vpu_h264_vpu_enc_close(d);
		msimx6vpu_h264_vpu_enc_open(d);
		
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
	.flags = MS_FILTER_IS_PUMP
};

/******************************************************************************
 * Init routine								      *
 *****************************************************************************/

MS2_PUBLIC void libmsimx6vpu_h264_init_enc(void) {
        ms_filter_register(&msimx6vpu_h264_enc_desc);
        ms_message("msimx6vpu-h264 encoder plugin registered.");
}

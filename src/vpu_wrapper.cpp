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

#define DEC_PS_SAVE_SIZE	0x080000
#define STREAM_BUF_SIZE		0x200000
#define KBPS				1000

VpuCommand::VpuCommand(VpuCommandEnum cmd, void *d, VpuCommandCallback cb, void *param)
 : command(cmd), data(d), callback(cb), extraParam(param)
{

}

const char* VpuCommand::ToString()
{
	switch (command) {
		case VPU_INIT:
			return "INIT";
		case VPU_UNINIT:
			return "VPU_UNINIT";
		case OPEN_DECODER:
			return "OPEN_DECODER";
		case OPEN_ENCODER:
			return "OPEN_ENCODER";
		case INIT_DECODER:
			return "INIT_DECODER";
		case INIT_ENCODER:
			return "INIT_ENCODER";
		case FILL_ENCODER_BUFFER:
			return "FILL_ENCODER_BUFFER";
		case FILL_DECODER_BUFFER:
			return "FILL_DECODER_BUFFER";
		case CLOSE_DECODER:
			return "CLOSE_DECODER";
		case CLOSE_ENCODER:
			return "CLOSE_ENCODER";
		case DECODE_FRAME:
			return "DECODE_FRAME";
		case ENCODE_FRAME:
			return "ENCODE_FRAME";
		default:
			return "Unknown command";
	}
}

void* VpuCommand::Run(VpuWrapper *wrapper)
{
	int result = -10;
	if (wrapper->debugModeEnabled) ms_message("[vpu_wrapper] running command %s", ToString());
	switch (command) {
		case VPU_INIT:
			result = wrapper->VpuInit();
			break;
		case VPU_UNINIT:
			wrapper->VpuUnInit();
			break;
		case OPEN_DECODER:
			result = wrapper->VpuOpenDecoder((MSIMX6VPUH264DecData *) data);
			break;
		case OPEN_ENCODER:
			result = wrapper->VpuOpenEncoder((MSIMX6VPUH264EncData *) data);
			break;
		case INIT_DECODER:
			result = wrapper->VpuInitDecoder((MSIMX6VPUH264DecData *) data);
			break;
		case INIT_ENCODER:
			result = wrapper->VpuInitEncoder((MSIMX6VPUH264EncData *) data);
			break;
		case FILL_ENCODER_BUFFER:
			result = wrapper->VpuFillEncoderBuffer((MSIMX6VPUH264EncData *) data, (MSPicture *)extraParam);
			break;
		case FILL_DECODER_BUFFER:
			result = wrapper->VpuFillDecoderBuffer((MSIMX6VPUH264DecData *) data, (int&)extraParam);
			break;
		case CLOSE_DECODER:
			wrapper->VpuCloseDecoder((MSIMX6VPUH264DecData *) data);
			break;
		case CLOSE_ENCODER:
			wrapper->VpuCloseEncoder((MSIMX6VPUH264EncData *) data);
			break;
		case DECODE_FRAME:
			wrapper->decodeFrameCommandCount += 1;
			result = wrapper->VpuDecodeFrame((MSIMX6VPUH264DecData *) data);
			break;
		case ENCODE_FRAME:
			wrapper->encodeFrameCommandCount += 1;
			result = wrapper->VpuEncodeFrame((MSIMX6VPUH264EncData *) data, (MSQueue *)extraParam);
			break;
		default:
			return NULL;
	}
	if (callback != NULL) {
		callback(data, result);
	}
	
	return NULL;
}

VpuCommand::~VpuCommand()
{

}

VpuWrapper* VpuWrapper::Instance()
{
	if (vpuWrapperInstance == NULL) {
		vpuWrapperInstance = new VpuWrapper();
	}
	return vpuWrapperInstance;
}

void* run(VpuWrapper *wrapper)
{
	if (wrapper->debugModeEnabled) ms_message("[vpu_wrapper] thread is looping");
	while (wrapper->threadRunning) {
		VpuCommand *command = wrapper->VpuDequeueCommand();
		if (command != NULL) {
			command->Run(wrapper);
		} else {
			ms_sleep(0.02);
		}
	}
	if (wrapper->debugModeEnabled) ms_message("[vpu_wrapper] thread loop has stopped");
	ms_thread_exit(NULL);
}

VpuWrapper::VpuWrapper() : isVpuInitialized(FALSE), debugModeEnabled(TRUE), threadRunning(FALSE), encodeFrameCommandCount(0), decodeFrameCommandCount(0)
{
	if (debugModeEnabled) ms_message("[vpu_wrapper] vpu wrapper created");
}

void VpuWrapper::VpuQueueCommand(VpuCommand *command)
{
	ms_mutex_lock(&mutex);
	commandQueue.push(command);
	if (debugModeEnabled) ms_message("[vpu_wrapper] command queued %s", command->ToString());
	
	if (!threadRunning) {
		threadRunning = TRUE;
		ms_thread_create(&thread, NULL, run, this);
		if (debugModeEnabled) ms_message("[vpu_wrapper] thread wasn't running so we started it");
	}
	ms_mutex_unlock(&mutex);
}

VpuCommand* VpuWrapper::VpuDequeueCommand()
{
	VpuCommand *command = NULL;
	ms_mutex_lock(&mutex);
	if (commandQueue.empty()) {
		ms_mutex_unlock(&mutex);
		return NULL;
	}
	command = commandQueue.front();
	commandQueue.pop();
	ms_mutex_unlock(&mutex);
	if (debugModeEnabled) ms_message("[vpu_wrapper] command dequeued %s", command->ToString());
	return command;
}

bool_t VpuWrapper::IsVpuInitialized()
{
	return isVpuInitialized;
}

int VpuWrapper::VpuInit()
{
	RetCode ret;
	vpu_versioninfo version;
	
	if (isVpuInitialized) {
		ms_warning("[vpuwrapper] VPU already initialized");
		return -1;
	}
	
	ret = vpu_Init(NULL);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpuwrapper] vpu_Init error: %d", ret);
		return -1;
	}
	isVpuInitialized = TRUE;

	ret = vpu_GetVersionInfo(&version);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpuwrapper] vpu_GetVersionInfo error: %d", ret);
		vpu_UnInit();
		return -1;
	}

	ms_message("[vpuwrapper] VPU firmware version: %d.%d.%d_r%d", version.fw_major, version.fw_minor, version.fw_release, version.fw_code);
	ms_message("[vpuwrapper] VPU library version: %d.%d.%d", version.lib_major, version.lib_minor, version.lib_release);
	
	return 0;
}

int VpuWrapper::VpuOpenDecoder(MSIMX6VPUH264DecData *d)
{
	RetCode ret;
	DecHandle handle = {0};
	DecOpenParam oparam = {0};
	
	if (d->handle != NULL) {
		ms_warning("[vpu_wrapper] decoder already openned");
		return;
	}
	
	d->bitstream_mem.size = STREAM_BUF_SIZE;
	ret = IOGetPhyMem(&d->bitstream_mem);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] IOGetPhyMem error: %d", ret);
		goto err;
	}

	ret = IOGetVirtMem(&d->bitstream_mem);
	if (ret <= 0) {
		ms_error("[vpu_wrapper] IOGetVirtMem error: %d", ret);
		goto err2;
	}
	
	d->ps_mem.size = DEC_PS_SAVE_SIZE;
	ret = IOGetPhyMem(&d->ps_mem);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] IOGetPhyMem error: %d", ret);
		goto err1;
	}
	
	oparam.bitstreamFormat = STD_AVC;
	oparam.bitstreamBuffer = d->bitstream_mem.phy_addr;
	oparam.bitstreamBufferSize = STREAM_BUF_SIZE;
	oparam.psSaveBuffer = d->ps_mem.phy_addr;
	oparam.psSaveBufferSize = DEC_PS_SAVE_SIZE;
	oparam.jpgLineBufferMode = 1;
	oparam.chromaInterleave = 0;
	oparam.bitstreamMode = 1;
	
	ret = vpu_DecOpen(&handle, &oparam);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_DecOpen error: %d", ret);
		goto err0;
	}

	d->handle = handle;
	d->virt_buf_addr = d->bitstream_mem.virt_uaddr;
	d->phy_buf_addr = d->bitstream_mem.phy_addr;
	d->phy_ps_buf = d->ps_mem.phy_addr;
	if (debugModeEnabled) ms_message("[vpu_wrapper] vpu decoder openned");
	return 0;
	
err0:
	IOFreePhyMem(&d->ps_mem);
err1:
	IOFreeVirtMem(&d->bitstream_mem);
err2:
	IOFreePhyMem(&d->bitstream_mem);
err:
	return -1;
}

int VpuWrapper::VpuOpenEncoder(MSIMX6VPUH264EncData* d)
{
	RetCode ret;
	EncHandle handle = {0};
	EncOpenParam oparam = {0};
	EncSliceMode slicemode = {0};
	
	if (d->handle != NULL) {
		ms_warning("[vpu_wrapper] encoder already openned");
		return;
	}
	
	d->bitstream_mem.size = STREAM_BUF_SIZE;
	ret = IOGetPhyMem(&d->bitstream_mem);
	if (ret) {
		ms_error("[vpu_wrapper] IOGetPhyMem error: %d", ret);
		goto err;
	}

	ret = IOGetVirtMem(&d->bitstream_mem);
	if (ret <= 0) {
		ms_error("[vpu_wrapper] IOGetVirtMem error: %d", ret);
		goto err1;
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
	oparam.gopSize = 1; // TODO
	oparam.userQpMin = -1;
	oparam.userQpMax = -1;
	
	ret = vpu_EncOpen(&handle, &oparam);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncOpen error: %d", ret);
		goto err0;
	}

	d->handle = handle;
	if (debugModeEnabled) ms_message("[vpu_wrapper] vpu encoder openned");
	return 0;
	
err0:
	IOFreeVirtMem(&d->bitstream_mem);
err1:
	IOFreePhyMem(&d->bitstream_mem);
err:
	return -1;
}

int VpuWrapper::VpuInitDecoder(MSIMX6VPUH264DecData* d)
{
	RetCode ret;
	DecInitialInfo initinfo = {0};
	
	if (d->configure_done) {
		ms_warning("[vpu_wrapper] decoder already initialized, skip");
		return -3;
	}
	
	vpu_DecSetEscSeqInit(d->handle, 1);
	ret = vpu_DecGetInitialInfo(d->handle, &initinfo);
	vpu_DecSetEscSeqInit(d->handle, 0);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_DecGetInitialInfo error: %d, errorcode: %ld", ret, initinfo.errorcode);
		return -2;
	}

	if (initinfo.streamInfoObtained) {
		if (debugModeEnabled) ms_message("[vpu_wrapper] H.264 Profile: %d Level: %d Interlace: %d", initinfo.profile, initinfo.level, initinfo.interlace);
		if (initinfo.aspectRateInfo) {
			int aspect_ratio_idc;
			int sar_width, sar_height;

			if ((initinfo.aspectRateInfo >> 16) == 0) {
				aspect_ratio_idc = (initinfo.aspectRateInfo & 0xFF);
				if (debugModeEnabled) ms_message("[vpu_wrapper] aspect_ratio_idc: %d", aspect_ratio_idc);
			} else {
				sar_width = (initinfo.aspectRateInfo >> 16) & 0xFFFF;
				sar_height = (initinfo.aspectRateInfo & 0xFFFF);
				if (debugModeEnabled) ms_message("[vpu_wrapper] sar_width: %d, sar_height: %d", sar_width, sar_height);
			}
		} else {
			ms_message("[vpu_wrapper] aspect ratio is not present");
		}
	}
	
	d->lastPicWidth = initinfo.picWidth;
	d->lastPicHeight = initinfo.picHeight;
	if (debugModeEnabled) ms_message("[vpu_wrapper] decoder: width = %d, height = %d, frameRateRes = %lu, frameRateDiv = %lu, count = %u", initinfo.picWidth, initinfo.picHeight, initinfo.frameRateRes, initinfo.frameRateDiv, initinfo.minFrameBufferCount);
	d->minfbcount = initinfo.minFrameBufferCount;
	
	// Let's use 2 more buffers than recommended
	if (initinfo.interlace) {
		d->regfbcount = d->minfbcount + 4;
	} else {
		d->regfbcount = d->minfbcount + 2;
	}
	if (debugModeEnabled) ms_warning("[vpu_wrapper] using %i fb buffers for decoder", d->regfbcount);
	d->picwidth = ((initinfo.picWidth + 15) & ~15);
	d->picheight = ((initinfo.picHeight + 31) & ~(31));
	
	if ((d->picwidth == 0) || (d->picheight == 0)) {
		if (debugModeEnabled) ms_error("[vpu_wrapper] width=%d, height=%d", d->picwidth, d->picheight);
		return -1;
	}
	
	/* worstSliceSize is in kilo-byte unit */
	d->phy_slicebuf_size = initinfo.worstSliceSize * 1024;
	d->stride = d->picwidth;
	
	
	d->slice_mem.size = d->phy_slicebuf_size;
	ret = IOGetPhyMem(&d->slice_mem);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] IOGetPhyMem error: %d", ret);
		return -1;
	}
	d->phy_slice_buf = d->slice_mem.phy_addr;
	
	if (VpuAllocDecoderBuffer(d) < 0) {
		ms_error("[vpu_wrapper] unable to allocate frame buffers");
		IOFreePhyMem(&d->slice_mem);
		return -1;
	}
	
	return 0;
}

int VpuWrapper::VpuInitEncoder(MSIMX6VPUH264EncData* d)
{
	RetCode ret;
	int err;
	EncInitialInfo initinfo = {0};
	EncHeaderParam header = {0};
	
	if (d->configure_done) {
		ms_warning("[vpu_wrapper] encoder already initialized, skip");
		return -3;
	}

	ret = vpu_EncGetInitialInfo(d->handle, &initinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncGetInitialInfo error: %d", ret);
		return -2;
	}

	d->minfbcount = initinfo.minFrameBufferCount;
	d->regfbcount = d->minfbcount + 2 + 1; // 1 is for the src buffer
	if (debugModeEnabled) ms_warning("[vpu_wrapper] using %i fb buffers for encoder", d->regfbcount);
	d->src_buffer_index = d->regfbcount - 1;
	
	if (VpuAllocEncoderBuffer(d) < 0) {
		ms_message("[vpu_wrapper] unable to allocate frame buffers");
		return -1;
	}

	header.headerType = SPS_RBSP;
	ret = vpu_EncGiveCommand(d->handle, ENC_PUT_AVC_HEADER, &header);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncGiveCommand ENC_PUT_AVC_HEADER SPS_RBSP error: %i", ret);
		return -1;
	}
	
	memset(&header, 0, sizeof(EncHeaderParam));
	header.headerType = PPS_RBSP;
	ret = vpu_EncGiveCommand(d->handle, ENC_PUT_AVC_HEADER, &header);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncGiveCommand ENC_PUT_AVC_HEADER PPS_RBSP error: %i", ret);
		return -1;
	}
	
	return 0;
}

int VpuWrapper::VpuAllocDecoderBuffer(MSIMX6VPUH264DecData* d)
{
	DecBufInfo bufinfo = {0};
	RetCode ret;
	int i, err;
	
	d->fbpool = (IMX6VPUFrameBuffer**) ms_malloc0(d->regfbcount * sizeof(IMX6VPUFrameBuffer *));
	if (d->fbpool == NULL) {
		ms_error("[vpu_wrapper] failed to allocate fbpool");
		return -1;
	}
	
	d->fbs = ms_malloc0(d->regfbcount * sizeof(FrameBuffer));
	if (d->fbs == NULL) {
		ms_error("[vpu_wrapper] failed to allocate fb");
		return -1;
	}
		
	for (i = 0; i < d->regfbcount; i++) {
		d->fbpool[i] = (IMX6VPUFrameBuffer*) ms_malloc0(sizeof(IMX6VPUFrameBuffer));
		
		memset(&(d->fbpool[i]->desc), 0, sizeof(vpu_mem_desc));
		d->fbpool[i]->desc.size = 3 * d->stride * d->picheight / 2;
		d->fbpool[i]->desc.size += (d->stride * d->picheight) / 4;
		err = IOGetPhyMem(&d->fbpool[i]->desc);
		if (err) {
			ms_error("[vpu_wrapper] error getting phymem for buffer %i", i);
			return -1;
		}
		
		d->fbpool[i]->addrY = d->fbpool[i]->desc.phy_addr;
		d->fbpool[i]->addrCb = d->fbpool[i]->addrY + (d->stride * d->picheight);
		d->fbpool[i]->addrCr = d->fbpool[i]->addrCb + (d->stride * d->picheight / 4);
		d->fbpool[i]->mvColBuf = d->fbpool[i]->addrCr + (d->stride * d->picheight / 4);
		d->fbpool[i]->strideY = d->stride;
		d->fbpool[i]->strideC = d->stride / 2;
		
		d->fbs[i].myIndex = i;
		d->fbs[i].bufY = d->fbpool[i]->addrY;
		d->fbs[i].bufCb = d->fbpool[i]->addrCb;
		d->fbs[i].bufCr = d->fbpool[i]->addrCr;
		d->fbs[i].bufMvCol = d->fbpool[i]->mvColBuf;
		d->fbs[i].strideY = d->fbpool[i]->strideY;
		d->fbs[i].strideC = d->fbpool[i]->strideC;
		
		d->fbpool[i]->desc.virt_uaddr = IOGetVirtMem(&(d->fbpool[i]->desc));
		if (d->fbpool[i]->desc.virt_uaddr <= 0) {
			ms_error("[vpu_wrapper] error getting virt mem for buffer %i", i);
			return -1;
		}
	}
	
	bufinfo.avcSliceBufInfo.bufferBase = d->phy_slice_buf;
	bufinfo.avcSliceBufInfo.bufferSize = d->phy_slicebuf_size;

	bufinfo.maxDecFrmInfo.maxMbX = d->stride / 16;
	bufinfo.maxDecFrmInfo.maxMbY = d->picheight / 16;
	bufinfo.maxDecFrmInfo.maxMbNum = d->stride * d->picheight / 256;
	
	ret = vpu_DecRegisterFrameBuffer(d->handle, d->fbs, d->regfbcount, d->stride, &bufinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_DecRegisterFrameBuffer error: %d", ret);
		return -1;
	}
	
	return 0;
}

int VpuWrapper::VpuAllocEncoderBuffer(MSIMX6VPUH264EncData* d)
{
	RetCode ret;
	int i, err;
	int enc_fbwidth, enc_fbheight, enc_stride;
	int src_fbwidth, src_fbheight, src_stride;
	int width, height, stride;
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
		ms_error("[vpu_wrapper] failed to allocate fbpool");
		return -1;
	}
	
	d->fbs = ms_malloc0(d->regfbcount * sizeof(FrameBuffer));
	if (d->fbs == NULL) {
		ms_error("[vpu_wrapper] failed to allocate fb");
		return -1;
	}
	
	
	for (i = 0; i < d->regfbcount; i++) {
		if (i == d->regfbcount - 1) { // Src buffer
			width = src_fbwidth;
			height = src_fbheight;
			stride = src_stride;
		} else {
			width = enc_fbwidth;
			height = enc_fbheight;
			stride = enc_stride;
		}
		d->fbpool[i] = (IMX6VPUFrameBuffer*) ms_malloc0(sizeof(IMX6VPUFrameBuffer));
		
		memset(&(d->fbpool[i]->desc), 0, sizeof(vpu_mem_desc));
		d->fbpool[i]->desc.size = 3 * width * height / 2;
		err = IOGetPhyMem(&d->fbpool[i]->desc);
		if (err) {
			ms_error("[vpu_wrapper] error getting phymem for buffer %i", i);
			return -1;
		}
		d->fbpool[i]->addrY = d->fbpool[i]->desc.phy_addr;
		d->fbpool[i]->addrCb = d->fbpool[i]->addrY + (width * height);
		d->fbpool[i]->addrCr = d->fbpool[i]->addrCb + (width * height / 4);
		d->fbpool[i]->strideY = width;
		d->fbpool[i]->strideC = width / 2;
		
		d->fbs[i].myIndex = i;
		d->fbs[i].bufY = d->fbpool[i]->addrY;
		d->fbs[i].bufCb = d->fbpool[i]->addrCb;
		d->fbs[i].bufCr = d->fbpool[i]->addrCr;
		d->fbs[i].strideY = d->fbpool[i]->strideY;
		d->fbs[i].strideC = d->fbpool[i]->strideC;
		
		d->fbpool[i]->desc.virt_uaddr = IOGetVirtMem(&(d->fbpool[i]->desc));
		if (d->fbpool[i]->desc.virt_uaddr <= 0) {
			ms_error("[vpu_wrapper] error getting virt mem for buffer %i", i);
			return -1;
		}
	}

	subSampBaseA = d->fbs[d->minfbcount].bufY;
	subSampBaseB = d->fbs[d->minfbcount + 1].bufY;
	
	ret = vpu_EncRegisterFrameBuffer(d->handle, d->fbs, d->regfbcount, enc_stride, src_stride, subSampBaseA, subSampBaseB, &extbufinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncRegisterFrameBuffer error: %d", ret);
		return -1;
	}
		
	return 0;
}

static int msimx6vpu_h264_read(void *src, void *dest, size_t n) {
	memcpy(dest, src, n);
	dest += n;
	return n;
}

int VpuWrapper::VpuFillDecoderBuffer(MSIMX6VPUH264DecData* d, int available)
{
	RetCode ret;
	unsigned long space, target_addr;
	PhysicalAddress pa_read_ptr, pa_write_ptr;
	int size, room, nread;
	bool_t eof = FALSE;
	int remaining = 0;
	uint8_t *bitstream = d->bitstream;
	
	ret = vpu_DecGetBitstreamBuffer(d->handle, &pa_read_ptr, &pa_write_ptr, &space);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[msimx6vpu_h264_dec] vpu_DecGetBitstreamBuffer error: %d", ret);
		return -1;
	}

	if (space <= 0) {
		ms_warning("[msimx6vpu_h264_dec] space %lu <= 0", space);
		return 0;
	} else {
		size = ((space >> 9) << 9);
	}
	
	target_addr = d->virt_buf_addr + (pa_write_ptr - d->phy_buf_addr);
	if (target_addr + size > d->virt_buf_addr + STREAM_BUF_SIZE) {
		room = (d->virt_buf_addr + STREAM_BUF_SIZE) - target_addr;
		if (available > room) {
			remaining = available - room;
			available = room;
		}
		nread = msimx6vpu_h264_read(bitstream, (void *)target_addr, available);
		if (nread <= 0) {
			ms_warning("[msimx6vpu_h264_dec] EOF or error (1)");
			if (nread < 0) {
				if (nread == -EAGAIN) {
					return 0;
				}

				ms_error("[msimx6vpu_h264_dec] nread %d < 0 (1)", nread);
				return -1;
			}
			eof = TRUE;
		} else {
			if (nread != available || remaining == 0) {
				if (nread != available) {
					ms_warning("[msimx6vpu_h264_dec] vpu_fill_buffer: unable to fill the requested size");
				}
				goto update;
			}
			
			space = nread;
			nread = msimx6vpu_h264_read(bitstream, (void *)d->virt_buf_addr, remaining);
			if (nread <= 0) {
				ms_warning("[msimx6vpu_h264_dec] EOF or error (2)");
				if (nread < 0) {
					if (nread == -EAGAIN) {
						return 0;
					}

					ms_error("[msimx6vpu_h264_dec] nread %d < 0 (2)", nread);
					return -1;
				}
				eof = TRUE;
			}

			nread += space;
		}
	} else {
		nread = msimx6vpu_h264_read(bitstream, (void *)target_addr, available);
		if (nread <= 0) {
			ms_warning("[msimx6vpu_h264_dec] EOF or error (3)");
			if (nread < 0) {
				if (nread == -EAGAIN) {
					return 0;
				}

				ms_error("[msimx6vpu_h264_dec] nread %d < 0 (3)", nread);
				return -1;
			}
			eof = TRUE;
		}
	}
	
update:	
	if (!eof) {
		ret = vpu_DecUpdateBitstreamBuffer(d->handle, nread);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_dec] vpu_DecUpdateBitstreamBuffer (1) error: %d", ret);
			return -1;
		}
	} else {
		ret = vpu_DecUpdateBitstreamBuffer(d->handle, STREAM_BUF_SIZE);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[msimx6vpu_h264_dec] vpu_DecUpdateBitstreamBuffer (2) error: %d", ret);
			return -1;
		}
	}
	
	return 0;
}



int VpuWrapper::VpuFillEncoderBuffer(MSIMX6VPUH264EncData *d, MSPicture *pic) {
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

int VpuWrapper::VpuReadEncoderBuffer(MSIMX6VPUH264EncData* d, MSQueue *nalus)
{
	RetCode ret;
	PhysicalAddress pa_read_ptr, pa_write_ptr;
	uint32_t size, target_addr;
	int space, room;
	
	ret = vpu_EncGetBitstreamBuffer(d->handle, &pa_read_ptr, &pa_write_ptr, (uint32_t *)&size);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncGetBitstreamBuffer error: %d", ret);
		return -1;
	}
	
	if (size < 0) {
		ms_warning("[vpu_wrapper] size %i < 0 !", size);
		return -1;
	}
	space = size;
	
	if (space > 0) {
		target_addr = d->bitstream_mem.virt_uaddr + (pa_read_ptr - d->bitstream_mem.phy_addr);
		if (target_addr + space > d->bitstream_mem.virt_uaddr + STREAM_BUF_SIZE) {
			room = d->bitstream_mem.virt_uaddr + STREAM_BUF_SIZE - target_addr;
			if (debugModeEnabled) ms_warning("[vpu_wrapper] frame (%i) is around the buffer of %i", space, space - room);
			frame_to_nalus(nalus, (void *)target_addr, room, (void *)d->bitstream_mem.virt_uaddr, space - room);
		} else {
			frame_to_nalus(nalus, (void *)target_addr, space, NULL, 0);
		}
		
		ret = vpu_EncUpdateBitstreamBuffer(d->handle, space);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[vpu_wrapper] vpu_EncUpdateBitstreamBuffer error: %d", ret);
			return -1;
		}
	}
	
	return space;
}

static void frame_to_mblkt(MSIMX6VPUH264DecData *d, int index) {
	IMX6VPUFrameBuffer *framebuff;
	MSVideoSize roi = {0};
	uint8_t *src_planes[3];
	int src_strides[3];
	int offset;
	int i;
	
	framebuff = d->fbpool[index];
	
	if (d->yuv_msg) {
		freemsg(d->yuv_msg);
	}
	
	d->yuv_msg = ms_yuv_buf_alloc(&d->outbuf, d->picwidth, d->picheight);
	roi.width = d->outbuf.w;
	roi.height = d->outbuf.h;
	
	offset = framebuff->desc.virt_uaddr - framebuff->desc.phy_addr;
	src_planes[0] = framebuff->addrY + offset;
	src_planes[1] = framebuff->addrCb + offset;
	src_planes[2] = framebuff->addrCr + offset;
	
	src_strides[0] = framebuff->strideY;
	src_strides[1] = framebuff->strideC;
	src_strides[2] = framebuff->strideC;
	
	for (int i = 0; i < 3; i++) {
		uint8_t *dst = d->outbuf.planes[i];
		uint8_t *src = src_planes[i];
		int h = d->picheight >> (( i > 0) ? 1 : 0);

		for(int j = 0; j < h; j++) {
			memcpy(dst, src, d->outbuf.strides[i]);
			dst += d->outbuf.strides[i];
			src += src_strides[(i == 0) ? 0 : 1];
		}
	}
}

int VpuWrapper::VpuDecodeFrame(MSIMX6VPUH264DecData* d)
{
	RetCode ret;
	DecParam decparams = {0};
	DecOutputInfo outinfos = {0};
	int i, loop;
	
	ret = vpu_DecStartOneFrame(d->handle, &decparams);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_DecStartOneFrame error: %d", ret);
		return -1;
	}
	
	loop = 0;
	while (vpu_IsBusy()) {
		vpu_WaitForInt(20);
		loop++;
	}
	
	ret = vpu_DecGetOutputInfo(d->handle, &outinfos);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_DecGetOutputInfo error: %d", ret);
		return -2;
	}
	
	if (outinfos.decodingSuccess == 0) {
		ms_warning("[vpu_wrapper] incomplete finish of decoding");
		return -2;
	}
	if (outinfos.notSufficientPsBuffer) {
		ms_error("[vpu_wrapper] vpu_DecGetOutputInfo error: PS buffer overflow");
		return -2;
	}
	if (outinfos.notSufficientSliceBuffer) {
		ms_error("[vpu_wrapper] vpu_DecGetOutputInfo error: Slice buffer overflow");
		return -2;
	}
	
	if (outinfos.indexFrameDecoded >= 0) {
		if (outinfos.decPicWidth != d->picwidth || outinfos.decPicHeight != d->picheight) {
			if (debugModeEnabled) ms_warning("[vpu_wrapper] size of the image has changed from %ix%i to %ix%i", 
				   d->picwidth, d->picheight, outinfos.decPicWidth, outinfos.decPicHeight);
		}
		d->picwidth = outinfos.decPicWidth;
		d->picheight = outinfos.decPicHeight;
		
		if (!d->first_image_decoded) {
			d->first_image_decoded = TRUE;
		}
		
		frame_to_mblkt(d, outinfos.indexFrameDecoded);
	
		ret = vpu_DecClrDispFlag(d->handle, outinfos.indexFrameDecoded);
		if (ret != RETCODE_SUCCESS) {
			ms_warning("[vpu_wrapper] failed to clear frame buffer %d", outinfos.indexFrameDecoded);
		}
		return 0;
	} else {
		for (i = 0; i < d->regfbcount; i++) {
			ret = vpu_DecClrDispFlag(d->handle, i);
			if (ret != RETCODE_SUCCESS) {
				ms_warning("[vpu_wrapper] failed to clear frame buffer %d", i);
			}
		}
		return -1;
	}
}

int VpuWrapper::VpuEncodeFrame(MSIMX6VPUH264EncData* d, MSQueue *nalus)
{
	RetCode ret;
	EncParam params = {0};
	EncOutputInfo outinfo = {0};
	mblk_t *m;
	int loop;
	
	params.sourceFrame = &d->fbs[d->src_buffer_index];
	if (d->generate_keyframe) {
		params.forceIPicture = 1;
		d->generate_keyframe = FALSE;
	}
		
	ret = vpu_EncStartOneFrame(d->handle, &params);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncStartOneFrame failed with error: %d", ret);
		return -1;
	}
	
	loop = 0;
	while (vpu_IsBusy()) {
		vpu_WaitForInt(20);
		loop++;
	}
	
	ret = vpu_EncGetOutputInfo(d->handle, &outinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("[vpu_wrapper] vpu_EncGetOutputInfo error: %d", ret);
		return -1;
	}
	
	if (outinfo.picType == 0) {
		if (debugModeEnabled) ms_message("[vpu_wrapper] I frame sent with %i slices", outinfo.numOfSlices);
	}
	if (outinfo.bitstreamWrapAround) {
		if (debugModeEnabled) ms_warning("[vpu_wrapper] encoder reports a buffer wrap around!");
	}
	
	if (outinfo.skipEncoded) {
		ms_warning("[vpu_wrapper] skip encoding one frame");
		return -1;
	}
	
	return VpuReadEncoderBuffer(d, nalus);
}

void free_framebuffer(IMX6VPUFrameBuffer *fb) {
	if (fb->desc.virt_uaddr) {
		IOFreeVirtMem(&fb->desc);
	}
	if (fb->desc.phy_addr) {
		IOFreePhyMem(&fb->desc);
	}
	memset(&fb->desc, 0, sizeof(vpu_mem_desc));
}

void VpuWrapper::VpuCloseDecoder(MSIMX6VPUH264DecData* d)
{
	RetCode ret;
	int i;
	
	ret = vpu_DecClose(d->handle);
	if (ret == RETCODE_FRAME_NOT_COMPLETE) {
		vpu_SWReset(d->handle, 0);
		ret = vpu_DecClose(d->handle);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[vpu_wrapper] vpu_DecClose error: %d", ret);
		}
	}
	
	if (d->handle != NULL) {
		IOFreePhyMem(&d->ps_mem);
		IOFreeVirtMem(&d->bitstream_mem);
		IOFreePhyMem(&d->bitstream_mem);
		IOFreePhyMem(&d->slice_mem);
	}
	
	if (d->fbpool) {
		for (i = 0; i < d->regfbcount; i++) {
			if (debugModeEnabled) ms_warning("[vpu_wrapper] dec freed buffer %i", i);
			free_framebuffer(d->fbpool[i]);
		}
	}
	if (d->fbs) {
		ms_free(d->fbs);
		d->fbs = NULL;
	}
	if (d->fbpool) {
		ms_free(d->fbpool);
		d->fbpool = NULL;
	}
	
	d->handle = NULL;
	if (debugModeEnabled) ms_message("[vpu_wrapper] vpu decoder closed");
}

void VpuWrapper::VpuCloseEncoder(MSIMX6VPUH264EncData* d)
{
	RetCode ret;
	int i;
	
	ret = vpu_EncClose(d->handle);
	if (ret == RETCODE_FRAME_NOT_COMPLETE) {
		vpu_SWReset(d->handle, 0);
		ret = vpu_EncClose(d->handle);
		if (ret != RETCODE_SUCCESS) {
			ms_error("[vpu_wrapper] vpu_EncClose error: %d", ret);
		}
	}
	
	if (d->handle != NULL) {
		IOFreeVirtMem(&d->bitstream_mem);
		IOFreePhyMem(&d->bitstream_mem);
	}
	
	if (d->fbpool) {
		for (i = 0; i < d->regfbcount; i++) {
			if (debugModeEnabled) ms_warning("[vpu_wrapper] enc freed buffer %i", i);
			free_framebuffer(d->fbpool[i]);
		}
	}
	if (d->fbs) {
		ms_free(d->fbs);
		d->fbs = NULL;
	}
	if (d->fbpool) {
		ms_free(d->fbpool);
		d->fbpool = NULL;
	}
	
	d->handle = NULL;
	if (debugModeEnabled) ms_message("[vpu_wrapper] vpu encoder closed");
}

void VpuWrapper::VpuUnInit()
{
	if (!isVpuInitialized) {
		if (debugModeEnabled) ms_warning("[vpuwrapper] VPU already uninitialized");
		return;
	}
	
	vpu_UnInit();
	isVpuInitialized = FALSE;
	if (debugModeEnabled) ms_message("[vpu_wrapper] UnInit done");
	ms_message("[vpu_wrapper] %i decode frame commands, %i encode frame commands", decodeFrameCommandCount, encodeFrameCommandCount);
}

VpuWrapper::~VpuWrapper()
{
	if (thread) {
		ms_thread_join(thread, NULL);
		if (debugModeEnabled) ms_message("[vpu_wrapper] thread has joined.");
	}
	
	ms_mutex_destroy(&mutex);
	if (debugModeEnabled) ms_message("[vpu_wrapper] vpu wrapper destroyed");
}

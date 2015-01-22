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

#include <queue>

extern  "C"
{
	#include <vpu_lib.h>
	#include <vpu_io.h>
	
	#include "mediastreamer2/msfilter.h"
	#include "mediastreamer2/msinterfaces.h"
	#include "mediastreamer2/msticker.h"
	#include "mediastreamer2/msvideo.h"
	#include "mediastreamer2/rfc3984.h"
	#include "mediastreamer2/videostarter.h"
	
	#define ENCODER_SRC_BUFFERS 4

	typedef struct imx6vpu_framebuffer {
		vpu_mem_desc desc;
		int strideY;
		int strideC;
		int addrY;
		int addrCb;
		int addrCr;
		int mvColBuf;
	} IMX6VPUFrameBuffer;

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
		mblk_t *sps_mblkt;
		mblk_t *pps_mblkt;
		MSFilter *filter;
		bool_t encode_frame_command_queued;
		bool_t init_command_queued;
		Rfc3984Context *packer;
		MSVideoConfiguration vconf;
		const MSVideoConfiguration *vconf_list;
		bool_t generate_keyframe;
		bool_t configure_done;
		unsigned int packet_num;
		MSVideoStarter starter;
		int mode;
		MSQueue *nalus;
		bool_t shutdown;
		int last_filled_buffer;
		bool_t restart_started;
		int last_encoded_buffer;
	} MSIMX6VPUH264EncData;

	typedef struct _MSIMX6VPUH264DecData {
		DecHandle handle;
		unsigned long virt_buf_addr;
		PhysicalAddress phy_buf_addr;
		PhysicalAddress phy_ps_buf;
		PhysicalAddress phy_slice_buf;
		vpu_mem_desc bitstream_mem;
		vpu_mem_desc ps_mem;
		vpu_mem_desc slice_mem;
		int phy_slicebuf_size;
		int picwidth;
		int picheight;
		int lastPicWidth;
		int lastPicHeight;
		int stride;
		int regfbcount;
		int minfbcount;
		FrameBuffer *fbs;
		IMX6VPUFrameBuffer **fbpool;
		MSFilter *filter;
		bool_t decode_frame_command_queued;
		Rfc3984Context unpacker;
		mblk_t *sps;
		mblk_t *pps;
		int bitstream_size;
		unsigned int packet_num;
		bool_t first_image_decoded;
		bool_t configure_done;
		MSPicture outbuf;
		mblk_t *yuv_msg;
		bool_t shutdown;
	} MSIMX6VPUH264DecData;

enum VpuCommandEnum {
	VPU_INIT,
	VPU_UNINIT,
	OPEN_ENCODER,
	OPEN_DECODER,
	INIT_ENCODER,
	INIT_DECODER,
	FILL_DECODER_BUFFER,
	CLOSE_ENCODER,
	CLOSE_DECODER,
	ENCODE_FRAME,
	DECODE_FRAME
};

typedef void (*VpuCommandCallback) (void *, int);

class VpuWrapper;

class VpuCommand {
public:
	VpuCommand(VpuCommandEnum cmd, void *d, VpuCommandCallback cb, void *param, int param2);
	VpuCommand(VpuCommandEnum cmd, void *d, VpuCommandCallback cb, void *param);
	void* Run(VpuWrapper *wrapper);
	const char *ToString();
	~VpuCommand();
	
private:
	VpuCommandEnum command;
	void *data, *extraParam;
	int extraParam2;
	VpuCommandCallback callback;
};

class VpuWrapper {
	friend class VpuCommand;
	
public:
	static VpuWrapper* Instance();
	static void CheckUnInitStatus();
	bool_t IsVpuInitialized();
	void VpuQueueCommand(VpuCommand *command);
	VpuCommand* VpuDequeueCommand();
	void CheckIfWaitingThreadForUnInitStatus();
	~VpuWrapper();
	static void init();
	
	bool_t isVpuInitialized;
	bool_t debugModeEnabled;
	bool_t threadRunning;
private:
	VpuWrapper();
	int VpuInit();
	int VpuUnInit();
	int VpuOpenEncoder(MSIMX6VPUH264EncData *d);
	int VpuOpenDecoder(MSIMX6VPUH264DecData *d);
	int VpuInitEncoder(MSIMX6VPUH264EncData *d);
	int VpuInitDecoder(MSIMX6VPUH264DecData *d);
	int VpuAllocEncoderBuffer(MSIMX6VPUH264EncData *d);
	int VpuAllocDecoderBuffer(MSIMX6VPUH264DecData *d);
	int VpuReadEncoderBuffer(MSIMX6VPUH264EncData *d, MSQueue *nalus);
	int VpuFillDecoderBuffer(MSIMX6VPUH264DecData *d, uint8_t *bitstream, int size);
	int VpuFillEncoderBuffer(MSIMX6VPUH264EncData *d, MSPicture *pic);
	void VpuCloseEncoder(MSIMX6VPUH264EncData *d);
	void VpuCloseDecoder(MSIMX6VPUH264DecData *d);
	int VpuEncodeFrame(MSIMX6VPUH264EncData *d, MSQueue *nalus);
	int VpuDecodeFrame(MSIMX6VPUH264DecData *d);
	
	std::queue<VpuCommand*> commandQueue;
	ms_mutex_t mutex;
	ms_thread_t thread;
	int encodeFrameCommandCount;
	int decodeFrameCommandCount;
	int encoderCount, decoderCount;
	
	static ms_mutex_t uninit_mutex;
	static ms_cond_t uninit_cond;
	bool_t notify_me_on_thread_exit;
};

static VpuWrapper *vpuWrapperInstance;

}
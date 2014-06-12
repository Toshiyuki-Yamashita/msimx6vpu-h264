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

typedef struct VPUWrapper {
	bool_t is_initialised;
	int refcount;
	bool_t is_ready;
} VPUWrapper;


typedef struct imx6vpu_framebuffer {
	vpu_mem_desc desc;
	FrameBuffer *fb;
	int strideY;
	int strideC;
} IMX6VPUFrameBuffer;

static VPUWrapper *imx6vpu;

int msimx6vpu_init();
void msimx6vpu_close();
bool_t msimx6vpu_isBusy();
void msimx6vpu_lockVPU();
void msimx6vpu_unlockVPU();
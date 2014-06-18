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

#include <vpu_lib.h>
#include <vpu_io.h>
#include "vpu.h"

int msimx6vpu_init(const char *caller) {
	RetCode ret;
	vpu_versioninfo version;
	
	if (imx6vpu == NULL) {
		ms_message("[msimx6vpu_h264_%s] creating imxvpu struct since it doesn't exist yet and refcount it", caller);
		imx6vpu = ms_malloc0(sizeof(VPUWrapper));
		imx6vpu->is_initialised = FALSE;
		imx6vpu->refcount = 1;
		imx6vpu->is_ready = FALSE;
	}
	
	if (imx6vpu->is_initialised) {
		imx6vpu->refcount += 1;
		ms_error("[msimx6vpu_h264_%s] VPU is already initialised, incrementing refcount to %i", caller, imx6vpu->refcount);
		return -1;
	}
	
	ret = vpu_Init(NULL);
	if (ret) {
		ms_error("[msimx6vpu_h264_%s] vpu_Init error: %d", caller, ret);
		return -1;
	}
	
	imx6vpu->is_initialised = TRUE;
	ms_message("[msimx6vpu_h264_%s] Init done", caller);

	ret = vpu_GetVersionInfo(&version);
	if (ret) {
		ms_error("[msimx6vpu_h264_%s] vpu_GetVersionInfo error: %d", caller, ret);
		vpu_UnInit();
		return -1;
	}

	ms_message("[msimx6vpu_h264_%s] VPU firmware version: %d.%d.%d_r%d", caller, version.fw_major, version.fw_minor, version.fw_release, version.fw_code);
	ms_message("[msimx6vpu_h264_%s] VPU library version: %d.%d.%d", caller, version.lib_major, version.lib_minor, version.lib_release);
	
	imx6vpu->is_ready = TRUE;
	return 0;
}

bool_t msimx6vpu_isBusy() {
	return !imx6vpu->is_ready;
}

void msimx6vpu_lockVPU() {
	if (!msimx6vpu_isBusy()) {
		imx6vpu->is_ready = FALSE;
	} else {
		ms_error("[msimx6vpu_h264] Already busy, wait");
	}
}

void msimx6vpu_unlockVPU() {
	imx6vpu->is_ready = TRUE;
}

void msimx6vpu_close(const char *caller) {
	imx6vpu->refcount -= 1;
	ms_message("[msimx6vpu_h264_%s] Close ? Refcount is %i", caller, imx6vpu->refcount);
	if (imx6vpu->refcount == 0) {
		vpu_UnInit();
		ms_message("[msimx6vpu_h264_%s] UnInit done", caller);
		imx6vpu->is_initialised = FALSE;
	}
}
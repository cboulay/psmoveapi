/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * Copyright (c) 2012 Benjamin Venditti <benjamin.venditti@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

#ifndef CAMERA_CONTROL_CLEYE_CLIENT_WIN32_H
#define CAMERA_CONTROL_CLEYE_CLIENT_WIN32_H

//-- includes -----
#ifndef _WIN64
// non-64bit builds can use the 32-bit CLEye dll
#include "../external/CLEye/CLEyeMulticam.h"

// These do nothing in non-64 bit builds
#define CLEyeInitializeServer(x) true
#define CLEyeDestroyServer()

#else
// 64bit Windows builds have to use a proxy interface to CLEye
#include <stdbool.h>

//-- typedefs -----
// camera instance type
typedef long CLEyeCameraInstance;

//-- constants -----
// camera modes
typedef enum
{ 
	CLEYE_MONO_PROCESSED,
	CLEYE_COLOR_PROCESSED,
	CLEYE_MONO_RAW,
	CLEYE_COLOR_RAW,
	CLEYE_BAYER_RAW
} CLEyeCameraColorMode;

// camera resolution
typedef enum
{ 
	CLEYE_QVGA,
	CLEYE_VGA
} CLEyeCameraResolution;

// camera parameters
typedef enum
{
	// camera sensor parameters
	CLEYE_AUTO_GAIN,			// [false, true]
	CLEYE_GAIN,					// [0, 79]
	CLEYE_AUTO_EXPOSURE,		// [false, true]
	CLEYE_EXPOSURE,				// [0, 511]
	CLEYE_AUTO_WHITEBALANCE,	// [false, true]
	CLEYE_WHITEBALANCE_RED,		// [0, 255]
	CLEYE_WHITEBALANCE_GREEN,	// [0, 255]
	CLEYE_WHITEBALANCE_BLUE,	// [0, 255]
	// camera linear transform parameters (valid for CLEYE_MONO_PROCESSED, CLEYE_COLOR_PROCESSED modes)
	CLEYE_HFLIP,				// [false, true]
	CLEYE_VFLIP,				// [false, true]
	CLEYE_HKEYSTONE,			// [-500, 500]
	CLEYE_VKEYSTONE,			// [-500, 500]
	CLEYE_XOFFSET,				// [-500, 500]
	CLEYE_YOFFSET,				// [-500, 500]
	CLEYE_ROTATION,				// [-500, 500]
	CLEYE_ZOOM,					// [-500, 500]
	// camera non-linear transform parameters (valid for CLEYE_MONO_PROCESSED, CLEYE_COLOR_PROCESSED modes)
	CLEYE_LENSCORRECTION1,		// [-500, 500]
	CLEYE_LENSCORRECTION2,		// [-500, 500]
	CLEYE_LENSCORRECTION3,		// [-500, 500]
	CLEYE_LENSBRIGHTNESS		// [-500, 500]
} CLEyeCameraParameter;

//-- interface -----
#ifdef __cplusplus
extern "C" {
#endif

// API Setup/Teardown
bool
CLEyeInitializeServer(const char *path_to_cleye_server_exe);

void
CLEyeDestroyServer();

// Camera information
int 
CLEyeGetCameraCount();

GUID 
CLEyeGetCameraUUID(int camId);

// Library initialization
CLEyeCameraInstance 
CLEyeCreateCamera(GUID camUUID, CLEyeCameraColorMode mode, CLEyeCameraResolution res, float frameRate);

bool 
CLEyeDestroyCamera(CLEyeCameraInstance cam);

// Camera capture control
bool 
CLEyeCameraStart(CLEyeCameraInstance cam);

bool 
CLEyeCameraStop(CLEyeCameraInstance cam);

// Camera LED control
bool 
CLEyeCameraLED(CLEyeCameraInstance cam, bool on);

// Camera parameters control
bool 
CLEyeSetCameraParameter(CLEyeCameraInstance cam, CLEyeCameraParameter param, int value);

int 
CLEyeGetCameraParameter(CLEyeCameraInstance cam, CLEyeCameraParameter param);

// Camera video frame image data retrieval
bool 
CLEyeCameraGetFrameDimensions(CLEyeCameraInstance cam, int* width, int* height);

bool 
CLEyeCameraGetFrame(CLEyeCameraInstance cam, PBYTE pData, int waitTimeout);

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif // _WIN64

#endif // CAMERA_CONTROL_CLEYE_CLIENT_WIN32_H

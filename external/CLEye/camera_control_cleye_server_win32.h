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

#ifndef CAMERA_CONTROL_CLEYE_SERVER_WIN32_H
#define CAMERA_CONTROL_CLEYE_SERVER_WIN32_H

#ifndef WIN32
#error "Only include camera_control_cleye_server_win32.h in win32 builds"
#endif

//-- constants -----
#define CLEYE_SERVER_PATH "psmove_cleye_server.exe"
#define SERVER_STARTED_EVENT_NAME "psmove_cleye_server_started_event"
#define SERVER_START_WAIT_TIMEOUT 1000
#define SERVER_STOP_WAIT_TIMEOUT 1000
#define SERVER_REQUEST_WAIT_TIMEOUT 50

#define SHARED_MEMORY_NAME "psmove_IPC_mem"
#define REQUEST_POSTED_EVENT_NAME "psmove_IPC_request_event"
#define RESPONSE_POSTED_EVENT_NAME "psmove_IPC_response_event"
#define VIDEO_BUFFER_MAX_SIZE 640*480*4

enum eServerRequestType
{
    _serverRequestType_getCameraCount,    
    _serverRequestType_getCameraUUID,
    _serverRequestType_createCamera,
    _serverRequestType_destroyCamera,
    _serverRequestType_cameraStart,
    _serverRequestType_cameraStop,
    _serverRequestType_setCameraLed,
    _serverRequestType_setCameraParameter,
    _serverRequestType_getCameraParameter,
    _serverRequestType_cameraGetFrameDimensions,
    _serverRequestType_cameraGetFrame,
    _serverRequestType_stopServer
};

//-- definitions ------

// Shared memory buffer that contains everything required to transmit
// data between the client and server
struct SharedMemoryBuffer
{
    union 
    {
        struct _BoolResponse
        {
            bool result;
        } BoolResult;

        struct _IntResponse
        {
            int result;
        } IntResult;

        struct _GetCameraUUIDResponse
        {
            GUID cameraUUID;
        } GetCameraUUIDResult;

        struct _CreateCameraResponse
        {
            long cameraInstance;
        } CreateCameraResponse;

        struct _CameraGetFrameDimensionsResponse
        {
            int width;
            int height;
        } CameraGetFrameDimensionsResponse;

        struct _CameraGetFrameResponse
        {
            unsigned char buffer[VIDEO_BUFFER_MAX_SIZE];
            int byteCount;
        } CameraGetFrameResponse;
    } Response;

    union 
    {
        struct _GetCameraUUIDRequest
        {
            int camId;
        } GetCameraUUIDRequest;

        struct _CreateCameraRequest
        {
            GUID camUUID;
            CLEyeCameraColorMode mode;
            CLEyeCameraResolution res; 
            float frameRate;
        } CreateCameraRequest;

        struct _DestroyCameraRequest
        {
            CLEyeCameraInstance camera;
        } DestroyCameraRequest;

        struct _CameraStartRequest
        {
            CLEyeCameraInstance camera;
        } CameraStartRequest;

        struct _CameraStopRequest
        {
            CLEyeCameraInstance camera;
        } CameraStopRequest;

        struct _CameraLEDRequest
        {
            CLEyeCameraInstance camera;
            bool on;
        } CameraLEDRequest;

        struct _SetCameraParameterRequest
        {
            CLEyeCameraInstance camera;
            CLEyeCameraParameter param;
            int value;
        } SetCameraParameterRequest;

        struct _GetCameraParameterRequest
        {
            CLEyeCameraInstance camera;
            CLEyeCameraParameter param;
        } GetCameraParameterRequest;

        struct _CameraGetFrameDimensionsRequest
        {
            CLEyeCameraInstance camera;
        } CameraGetFrameDimensionsRequest;

        struct _CameraGetFrameRequest
        {
            CLEyeCameraInstance camera;
            int waitTimeout;
        } CameraGetFrameRequest;
    } Request;

    eServerRequestType RequestType;
};

#endif // CAMERA_CONTROL_CLEYE_SERVER_WIN32_H

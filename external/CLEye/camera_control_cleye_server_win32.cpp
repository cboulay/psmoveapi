/**
 * PS3EYEDriver C API Interface for use with PS Move API
 * Copyright (c) 2014 Thomas Perl <m@thp.io>
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

 //-- includes -----
#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <windows.h>
#include <memory.h>

#include "CLEyeMulticam.h"
#include "camera_control_cleye_server_win32.h"

//-- constants -----
#define SERVER_REQUEST_IDLE_TIMEOUT 500
#define SERVER_MAX_INACTIVE_COUNT 120 // 1 min @ 500ms per timeout
#define MAX_TRACKED_CAMERAS 8

//-- macros -----
#define log(section, msg, ...) \
    fprintf(stderr, "[" section "] " msg, ## __VA_ARGS__)

#define log_ERROR(msg, ...) \
    log("ERROR", msg, ## __VA_ARGS__)

/* Macro: Debugging output */
#ifdef _DEBUG
#    define log_DEBUG(msg, ...) \
            log("DEBUG", msg, ## __VA_ARGS__)
#else
#    define log_DEBUG(msg, ...)
#endif

//-- definitions -----
struct CLEyeCameraState
{
    CLEyeCameraInstance camera;
    bool started;
    bool ledOn;
};

struct CLEyeServerState
{
    // Server process state
    HANDLE hServerStartedEvent; // Event used to signal that the server has started
    CLEyeCameraState trackedCameras[MAX_TRACKED_CAMERAS];

    // Shared memory state
    HANDLE hRequestPostedEvent; // Event used to signal the server a request has been posted
    HANDLE hResponsePostedEvent;// Event used to signal the client that a response has been posted
    HANDLE hMapFile;            // Handle to the mapped memory file
    SharedMemoryBuffer *pMapBuffer;        // Buffer that points to the shared memory
};

//-- globals -----
CLEyeServerState g_server_state = {};

//-- prototypes -----
bool create_shared_memory_state(CLEyeServerState *server_state);
void destroy_shared_memory_state(CLEyeServerState *server_state);
CLEyeCameraState *allocate_tracked_camera_state(CLEyeServerState *server_state);
void free_tracked_camera_state(CLEyeServerState *server_state, CLEyeCameraInstance camera);
void free_all_tracked_camera_state(CLEyeServerState *server_state);
CLEyeCameraState *find_tracked_camera_state(CLEyeServerState *server_state, CLEyeCameraInstance camera);

//-- entry point -----
int _tmain(int argc, _TCHAR* argv[])
{
    bool success= true;

    ZeroMemory(&g_server_state, sizeof(CLEyeServerState));

    success= create_shared_memory_state(&g_server_state);

    if (success)
    {
        bool keepRunning= true;
        int inactiveCount = 0;

        // Signal the client that spawned us that we are ready to listen to events
        SetEvent(g_server_state.hServerStartedEvent);
        Sleep(1);

        while (keepRunning)
        {
            if (WaitForSingleObject(g_server_state.hRequestPostedEvent, SERVER_REQUEST_IDLE_TIMEOUT) == WAIT_OBJECT_0)
            {
                inactiveCount= 0;

                ResetEvent(g_server_state.hRequestPostedEvent);

                switch(g_server_state.pMapBuffer->RequestType)
                {
                case _serverRequestType_getCameraCount:
                    g_server_state.pMapBuffer->Response.IntResult.result= 
                        CLEyeGetCameraCount();
                    break;
                case _serverRequestType_getCameraUUID:
                    g_server_state.pMapBuffer->Response.GetCameraUUIDResult.cameraUUID=
                        CLEyeGetCameraUUID(g_server_state.pMapBuffer->Request.GetCameraUUIDRequest.camId);
                    break;
                case _serverRequestType_createCamera:
                    {
                        CLEyeCameraInstance camera=
                            CLEyeCreateCamera(
                                g_server_state.pMapBuffer->Request.CreateCameraRequest.camUUID, 
                                g_server_state.pMapBuffer->Request.CreateCameraRequest.mode, 
                                g_server_state.pMapBuffer->Request.CreateCameraRequest.res, 
                                g_server_state.pMapBuffer->Request.CreateCameraRequest.frameRate);

                        g_server_state.pMapBuffer->Response.CreateCameraResponse.cameraInstance= (long)camera;
                        if (camera != 0)
                        {
                            CLEyeCameraState *trackedState= allocate_tracked_camera_state(&g_server_state);

                            if (trackedState != NULL)
                            {
                                ZeroMemory(trackedState, sizeof(CLEyeCameraState));
                                trackedState->camera= camera;
                            }
                        }
                    } break;
                case _serverRequestType_destroyCamera:
                    {
                        CLEyeCameraInstance camera= 
                            (CLEyeCameraInstance)g_server_state.pMapBuffer->Request.DestroyCameraRequest.camera;

                        free_tracked_camera_state(&g_server_state, camera);
                        g_server_state.pMapBuffer->Response.BoolResult.result= CLEyeDestroyCamera(camera);
                    }
                    break;
                case _serverRequestType_cameraStart:
                    {
                        CLEyeCameraInstance camera= 
                            (CLEyeCameraInstance)g_server_state.pMapBuffer->Request.CameraStartRequest.camera;
                        CLEyeCameraState *trackedState= find_tracked_camera_state(&g_server_state, camera);

                        g_server_state.pMapBuffer->Response.BoolResult.result= CLEyeCameraStart(camera);

                        if (g_server_state.pMapBuffer->Response.BoolResult.result && trackedState != NULL)
                        {
                            trackedState->started= true;
                        }
                    }
                    break;
                case _serverRequestType_cameraStop:
                    {
                        CLEyeCameraInstance camera= 
                            (CLEyeCameraInstance)g_server_state.pMapBuffer->Request.CameraStopRequest.camera;
                        CLEyeCameraState *trackedState= find_tracked_camera_state(&g_server_state, camera);

                        g_server_state.pMapBuffer->Response.BoolResult.result=
                            CLEyeCameraStop(g_server_state.pMapBuffer->Request.CameraStopRequest.camera);

                        if (g_server_state.pMapBuffer->Response.BoolResult.result && trackedState != NULL)
                        {
                            trackedState->started= false;
                        }
                    }
                    break;
                case _serverRequestType_setCameraLed:
                    {
                        CLEyeCameraInstance camera= 
                            (CLEyeCameraInstance)g_server_state.pMapBuffer->Request.CameraLEDRequest.camera;
                        CLEyeCameraState *trackedState= find_tracked_camera_state(&g_server_state, camera);

                        g_server_state.pMapBuffer->Response.BoolResult.result=
                            CLEyeCameraLED(
                                camera, 
                                g_server_state.pMapBuffer->Request.CameraLEDRequest.on);

                        if (g_server_state.pMapBuffer->Response.BoolResult.result && trackedState != NULL)
                        {
                            trackedState->ledOn= g_server_state.pMapBuffer->Request.CameraLEDRequest.on;
                        }
                    }
                    break;
                case _serverRequestType_setCameraParameter:
                    g_server_state.pMapBuffer->Response.BoolResult.result=
                        CLEyeSetCameraParameter(
                            g_server_state.pMapBuffer->Request.SetCameraParameterRequest.camera, 
                            g_server_state.pMapBuffer->Request.SetCameraParameterRequest.param,
                            g_server_state.pMapBuffer->Request.SetCameraParameterRequest.value);
                    break;
                case _serverRequestType_getCameraParameter:
                    g_server_state.pMapBuffer->Response.IntResult.result=
                        CLEyeGetCameraParameter(
                            g_server_state.pMapBuffer->Request.GetCameraParameterRequest.camera, 
                            g_server_state.pMapBuffer->Request.GetCameraParameterRequest.param);
                    break;
                case _serverRequestType_cameraGetFrameDimensions:
                    g_server_state.pMapBuffer->Response.BoolResult.result=
                        CLEyeCameraGetFrameDimensions(
                            g_server_state.pMapBuffer->Request.CameraGetFrameDimensionsRequest.camera, 
                            g_server_state.pMapBuffer->Response.CameraGetFrameDimensionsResponse.width,
                            g_server_state.pMapBuffer->Response.CameraGetFrameDimensionsResponse.height);
                    break;
                case _serverRequestType_cameraGetFrame:                  
                    g_server_state.pMapBuffer->Response.CameraGetFrameResponse.byteCount= 0;

                    if (CLEyeCameraGetFrame(
                            g_server_state.pMapBuffer->Request.CameraGetFrameRequest.camera,
                            g_server_state.pMapBuffer->Response.CameraGetFrameResponse.buffer,
                            g_server_state.pMapBuffer->Request.CameraGetFrameRequest.waitTimeout))
                    {
                        int width, height;

                        if (CLEyeCameraGetFrameDimensions(
                                g_server_state.pMapBuffer->Request.CameraGetFrameDimensionsRequest.camera, 
                                width, height))
                        {
                            // Assume RGBA
                            g_server_state.pMapBuffer->Response.CameraGetFrameResponse.byteCount= width*height*4;
                        }
                    }
                    break;
                case _serverRequestType_stopServer:
                    keepRunning= false;
                    break;
                default:
                    break;
                }

                SetEvent(g_server_state.hResponsePostedEvent);
                Sleep(1);
            }
            else
            {
                inactiveCount++;
                if (inactiveCount >= SERVER_MAX_INACTIVE_COUNT)
                {
                    keepRunning= false;
                }

                Sleep(1);
            }
        }
    }

    free_all_tracked_camera_state(&g_server_state);
    destroy_shared_memory_state(&g_server_state);

    return success ? 0 : -1;
}

//-- private methods -----
bool create_shared_memory_state(CLEyeServerState *server_state)
{
    bool success = true;

    // Create the events
    server_state->hRequestPostedEvent = CreateEventA(NULL, TRUE, FALSE, REQUEST_POSTED_EVENT_NAME);
    if (server_state->hRequestPostedEvent == NULL || server_state->hRequestPostedEvent == INVALID_HANDLE_VALUE)
    {
        log_ERROR("Failed to create CL eye filled event due to error: %d\n", GetLastError());
        success= false;
    }

    if (success)
    {
        server_state->hResponsePostedEvent = CreateEventA(NULL, TRUE, FALSE, RESPONSE_POSTED_EVENT_NAME);
        if (server_state->hResponsePostedEvent == NULL || server_state->hResponsePostedEvent == INVALID_HANDLE_VALUE)
        {
            log_ERROR("Failed to create CL eye available event due to error: %d\n", GetLastError());
            success= false;
        }
    }

    if (success)
    {
        server_state->hServerStartedEvent = CreateEventA(NULL, TRUE, FALSE, SERVER_STARTED_EVENT_NAME);
        if (server_state->hServerStartedEvent == NULL || server_state->hServerStartedEvent == INVALID_HANDLE_VALUE)
        {
            log_ERROR("Failed to create CL eye server started event due to error: %d\n", GetLastError());
            success= false;
        }
    }

    // Open the shared file 
    if (success)
    {
        server_state->hMapFile = 
            CreateFileMappingA(
                INVALID_HANDLE_VALUE,
                NULL,
                PAGE_READWRITE,
                0,
                sizeof(SharedMemoryBuffer),
                SHARED_MEMORY_NAME);
        if (server_state->hMapFile == NULL || server_state->hMapFile == INVALID_HANDLE_VALUE)
        {
            log_ERROR("Failed to open CL eye memory file due to error: %d\n", GetLastError());
            success= false;
        }
    }

    // Memory map the file
    if (success)
    {
        server_state->pMapBuffer = 
            (SharedMemoryBuffer*)MapViewOfFile(
                server_state->hMapFile, // handle to map object
                FILE_MAP_ALL_ACCESS, // read/write permission
                0,
                0,
                sizeof(SharedMemoryBuffer));
        if (server_state->pMapBuffer == NULL)
        {
            log_ERROR("Failed to map CL eye memory file due to error: %d\n", GetLastError());
            success= false;
        }
    }

    if (success)
    {
	    ZeroMemory(server_state->pMapBuffer, sizeof(SharedMemoryBuffer));
    }

    return success;
}

void destroy_shared_memory_state(CLEyeServerState *server_state)
{
    if (server_state->pMapBuffer != NULL)
    {
        UnmapViewOfFile(server_state->pMapBuffer);
        server_state->pMapBuffer= NULL;
    }

    if (server_state->hMapFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(server_state->hMapFile);
        server_state->hMapFile= INVALID_HANDLE_VALUE;
    }

    if (server_state->hResponsePostedEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(server_state->hResponsePostedEvent);
        server_state->hResponsePostedEvent= INVALID_HANDLE_VALUE;
    }

    if (server_state->hRequestPostedEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(server_state->hRequestPostedEvent);
        server_state->hRequestPostedEvent= INVALID_HANDLE_VALUE;
    }

    if (server_state->hServerStartedEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(server_state->hServerStartedEvent);
        server_state->hServerStartedEvent= INVALID_HANDLE_VALUE;
    }
}

CLEyeCameraState *allocate_tracked_camera_state(CLEyeServerState *server_state)
{
    CLEyeCameraState *cameraState= NULL;

    for (int index=0; index < MAX_TRACKED_CAMERAS; ++index)
    {
        if (server_state->trackedCameras[index].camera == NULL)
        {
            cameraState= &server_state->trackedCameras[index];
            break;
        }
    }

    return cameraState;
}

void free_tracked_camera_state(CLEyeServerState *server_state, CLEyeCameraInstance camera)
{
    CLEyeCameraState *tracked_state= find_tracked_camera_state(server_state, camera);

    if (tracked_state != NULL)
    {
        if (tracked_state->ledOn)
        {
            CLEyeCameraLED(camera, false);
        }

        if (tracked_state->started)
        {
            CLEyeCameraStop(camera);
        }

        ZeroMemory(tracked_state, sizeof(CLEyeCameraState));
    }
}

void free_all_tracked_camera_state(CLEyeServerState *server_state)
{
    for (int index=0; index < MAX_TRACKED_CAMERAS; ++index)
    {
        CLEyeCameraState *tracked_state= &server_state->trackedCameras[index];

        if (tracked_state->camera != NULL)
        {
            if (tracked_state->ledOn)
            {
                CLEyeCameraLED(tracked_state->camera, false);
            }

            if (tracked_state->started)
            {
                CLEyeCameraStop(tracked_state->camera);
            }

            CLEyeDestroyCamera(tracked_state->camera);

            ZeroMemory(tracked_state, sizeof(CLEyeCameraState));
        }
    }
}

CLEyeCameraState *find_tracked_camera_state(CLEyeServerState *server_state, CLEyeCameraInstance camera)
{
    CLEyeCameraState *cameraState= NULL;

    if (camera != NULL)
    {
        for (int index=0; index < MAX_TRACKED_CAMERAS; ++index)
        {
            if (server_state->trackedCameras[index].camera == camera)
            {
                cameraState= &server_state->trackedCameras[index];
                break;
            }
        }
    }

    return cameraState;
}
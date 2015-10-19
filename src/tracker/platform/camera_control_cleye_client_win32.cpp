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

 // We only want the proxy interface for win 64-bit builds.
 // In win 32-bit builds we can load the 32-bit CL Eye dll directly.
#ifdef _WIN64

//-- includes -----
#include "psmove.h"  // includes psmove_file.h
#include "psmove_private.h"

#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#include <windows.h>
#include <tlhelp32.h>

#include "camera_control_cleye_client_win32.h"
#include "camera_control_cleye_server_win32.h"

//-- definitions
struct CLEyeClientState
{
    bool IsInitialized;

    // Server process state
    PROCESS_INFORMATION serverProcessInformation;
    HANDLE hServerStartedEvent; // Event used to signal that the server has started

    // Shared memory state
    HANDLE hRequestPostedEvent; // Event used to signal the server a request has been posted
    HANDLE hResponsePostedEvent;// Event used to signal the client that a response has been posted
    HANDLE hMapFile;            // Handle to the mapped memory file
    SharedMemoryBuffer *pMapBuffer;        // Buffer that points to the shared memory
};

//-- globals -----
CLEyeClientState g_client_state= {};

//-- prototypes -----
bool create_server_process(const char *path_to_eleye_server_exe, CLEyeClientState *client_state);
void destroy_server_process(CLEyeClientState *client_state);
void terminate_all_server_processes();
bool create_shared_memory_state(CLEyeClientState *client_state);
void destroy_shared_memory_state(CLEyeClientState *client_state);
bool post_and_wait_for_server_response(CLEyeClientState *client_state, int timeout=SERVER_REQUEST_WAIT_TIMEOUT);

//-- interface -----
bool
CLEyeInitializeServer(const char *path_to_cleye_server_exe)
{
    bool success= false;

    ZeroMemory(&g_client_state, sizeof(CLEyeClientState));

    // Kill an orphaned server process since we don't know what state it's in
    terminate_all_server_processes();

    // Create a brand new server process
    if (create_server_process(path_to_cleye_server_exe, &g_client_state))
    {
        // Create the shared memory and associated events
        if (create_shared_memory_state(&g_client_state))
        {
            g_client_state.IsInitialized= true;
            success= true;
        }
    }

    if (!success)
    {
        destroy_shared_memory_state(&g_client_state);
        destroy_server_process(&g_client_state);
    }

    return success;
}

void
CLEyeDestroyServer()
{
    if (g_client_state.IsInitialized)
    {
        destroy_server_process(&g_client_state);
        destroy_shared_memory_state(&g_client_state);
        g_client_state.IsInitialized= false;
    }
}

CLEyeCameraInstance 
CLEyeCreateCamera(
    GUID camUUID, 
    CLEyeCameraColorMode mode,
    CLEyeCameraResolution res, 
    float frameRate)
{
    CLEyeCameraInstance result= NULL;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_createCamera;
        g_client_state.pMapBuffer->Request.CreateCameraRequest= {
                camUUID,
                mode,
                res,
                frameRate
            };

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.CreateCameraResponse.cameraInstance;
        }
    }

    return result;
}

bool 
CLEyeDestroyCamera(CLEyeCameraInstance cam)
{
    bool result= false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_destroyCamera;
        g_client_state.pMapBuffer->Request.DestroyCameraRequest= {cam};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.BoolResult.result;
        }
    }

    return result;
}

int 
CLEyeGetCameraCount()
{
    int result = 0;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_getCameraCount;

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.IntResult.result;
        }
    }

    return result;
}

GUID 
CLEyeGetCameraUUID(int camId)
{
    GUID result = {};

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_getCameraUUID;
        g_client_state.pMapBuffer->Request.GetCameraUUIDRequest= {camId};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.GetCameraUUIDResult.cameraUUID;
        }
    }

    return result;
}

bool 
CLEyeCameraStart(CLEyeCameraInstance cam)
{
    bool result = false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_cameraStart;
        g_client_state.pMapBuffer->Request.CameraStartRequest= {cam};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.BoolResult.result;
        }
    }

    return result;
}

bool 
CLEyeCameraStop(CLEyeCameraInstance cam)
{
    bool result = false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_cameraStop;
        g_client_state.pMapBuffer->Request.CameraStopRequest= {cam};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.BoolResult.result;
        }
    }

    return result;
}

bool 
CLEyeCameraLED(CLEyeCameraInstance cam, bool on)
{
    bool result = false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_setCameraLed;
        g_client_state.pMapBuffer->Request.CameraLEDRequest= {cam, on};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.BoolResult.result;
        }
    }

    return result;
}

bool
CLEyeSetCameraParameter(CLEyeCameraInstance cam, CLEyeCameraParameter param, int value)
{
    bool result = false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_setCameraParameter;
        g_client_state.pMapBuffer->Request.SetCameraParameterRequest= {cam, param, value};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.BoolResult.result;
        }
    }

    return result;
}

int
CLEyeGetCameraParameter(CLEyeCameraInstance cam, CLEyeCameraParameter param)
{
    bool result = false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_getCameraParameter;
        g_client_state.pMapBuffer->Request.GetCameraParameterRequest= {cam, param};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            result= g_client_state.pMapBuffer->Response.IntResult.result;
        }
    }

    return result;
}

bool
CLEyeCameraGetFrameDimensions(CLEyeCameraInstance cam, int* width, int* height)
{
    bool result = false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_cameraGetFrameDimensions;
        g_client_state.pMapBuffer->Request.CameraGetFrameDimensionsRequest= {cam};

        if (post_and_wait_for_server_response(&g_client_state))
        {
            if (width)
            {
                *width= g_client_state.pMapBuffer->Response.CameraGetFrameDimensionsResponse.width;

                //HACK: For some reason the initial frame size comes back at 513
                // but when we actually poll the data it's 640
                if (*width > 320)
                {
                    *width= 640;
                }
            }

            if (height)
            {
                *height= g_client_state.pMapBuffer->Response.CameraGetFrameDimensionsResponse.height;
            }

            result= true;
        }
    }

    return result;
}

bool
CLEyeCameraGetFrame(CLEyeCameraInstance cam, PBYTE pData, int waitTimeout)
{
    bool result = false;

    if (g_client_state.IsInitialized)
    {
        g_client_state.pMapBuffer->RequestType = _serverRequestType_cameraGetFrame;
        g_client_state.pMapBuffer->Request.CameraGetFrameRequest= {cam, waitTimeout};

        if (post_and_wait_for_server_response(&g_client_state, waitTimeout+SERVER_REQUEST_WAIT_TIMEOUT))
        {
            if (g_client_state.pMapBuffer->Response.CameraGetFrameResponse.byteCount > 0)
            {
                memcpy(
                    pData, 
                    g_client_state.pMapBuffer->Response.CameraGetFrameResponse.buffer, 
                    g_client_state.pMapBuffer->Response.CameraGetFrameResponse.byteCount);
                result= true;
            }
        }
    }

    return result;
}

//-- private methods -----
bool create_server_process(const char *path_to_eleye_server_exe, CLEyeClientState *client_state)
{
    bool success= true;    

    memset(client_state, 0, sizeof(CLEyeClientState));
        
    client_state->hServerStartedEvent = CreateEventA(NULL, TRUE, FALSE, SERVER_STARTED_EVENT_NAME);
    if (client_state->hServerStartedEvent == INVALID_HANDLE_VALUE)
    {
        psmove_DEBUG("Failed to create CL eye server started event to error: %d\n", GetLastError());
        success= false;
    }

    if (success)
    {
        STARTUPINFO startupInfo = {};
        startupInfo.cb = sizeof startupInfo;

        char full_path[MAX_PATH];

        if (path_to_eleye_server_exe != NULL && path_to_eleye_server_exe[0] != '\0')
        {
            _snprintf(full_path, MAX_PATH, "%s/%s", path_to_eleye_server_exe, CLEYE_SERVER_PATH);
        }
        else
        {
            _snprintf(full_path, MAX_PATH, "%s", CLEYE_SERVER_PATH);
        }

        if (!CreateProcessA(
                full_path, // Application Name
                NULL, // Command Line
                NULL, // Process Security Attributes
                NULL, // Thread Security Attributes
                FALSE, // Don't inherit handles 
                NORMAL_PRIORITY_CLASS | CREATE_NO_WINDOW, // Creation flags
                NULL, // Use the environment of this process
                NULL, // Use the same current directory of this process 
                &startupInfo, 
                &client_state->serverProcessInformation))
        {
            psmove_DEBUG("Failed to create CL eye server process due to error: %d\n", GetLastError());
            success= false;
        }
    }

    if (success)
    {
        if (WaitForSingleObject(client_state->hServerStartedEvent, SERVER_START_WAIT_TIMEOUT) != WAIT_OBJECT_0)
        {
            psmove_DEBUG("Failed to get start event from CL eye server process due to error: %d\n", GetLastError());
            success= false;
        }
    }

    return success= true;
}

void destroy_server_process(CLEyeClientState *client_state)
{
    if (client_state->serverProcessInformation.hProcess != INVALID_HANDLE_VALUE && 
        client_state->serverProcessInformation.hProcess != NULL)
    {
        client_state->pMapBuffer->RequestType= _serverRequestType_stopServer;
        post_and_wait_for_server_response(client_state);
        
        if (WaitForSingleObject(client_state->serverProcessInformation.hProcess, SERVER_STOP_WAIT_TIMEOUT) != WAIT_OBJECT_0)
        {
            TerminateProcess(client_state->serverProcessInformation.hProcess, -1);
        }

        CloseHandle(client_state->serverProcessInformation.hProcess);
        CloseHandle(client_state->serverProcessInformation.hThread);

        client_state->serverProcessInformation.hProcess= INVALID_HANDLE_VALUE;
        client_state->serverProcessInformation.hThread= INVALID_HANDLE_VALUE;
    }

    if (client_state->hServerStartedEvent != INVALID_HANDLE_VALUE && 
        client_state->hServerStartedEvent != NULL)
    {
        CloseHandle(client_state->hServerStartedEvent);
        client_state->hServerStartedEvent = INVALID_HANDLE_VALUE;
    }
}

void terminate_all_server_processes()
{
    PROCESSENTRY32 entry;
    entry.dwSize = sizeof(PROCESSENTRY32);

    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);

    if (Process32First(snapshot, &entry) == TRUE)
    {
        while (Process32Next(snapshot, &entry) == TRUE)
        {
            if (stricmp(entry.szExeFile, CLEYE_SERVER_PATH) == 0)
            {  
                HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, entry.th32ProcessID);

                if (hProcess != INVALID_HANDLE_VALUE && hProcess != NULL)
                {
                    TerminateProcess(hProcess, -1);
                    
                    if (WaitForSingleObject(hProcess, 100) != WAIT_OBJECT_0)
                    {
                        psmove_DEBUG("Failed to terminate orphaned cl eye server process: %d\n", GetLastError());
                    }

                    CloseHandle(hProcess);
                }
                else
                {
                    psmove_DEBUG("Failed to open orphaned cl eye server process: %d\n", GetLastError());
                }
            }
        }
    }

    CloseHandle(snapshot);
}

bool create_shared_memory_state(CLEyeClientState *client_state)
{
    bool success = true;

    // Create the events
    client_state->hRequestPostedEvent = CreateEventA(NULL, TRUE, FALSE, REQUEST_POSTED_EVENT_NAME);
    if (client_state->hRequestPostedEvent == NULL || client_state->hRequestPostedEvent == INVALID_HANDLE_VALUE)
    {
        psmove_DEBUG("Failed to create CL eye request event due to error: %d\n", GetLastError());
        success= false;
    }

    if (success)
    {
        client_state->hResponsePostedEvent = CreateEventA(NULL, TRUE, FALSE, RESPONSE_POSTED_EVENT_NAME);
        if (client_state->hResponsePostedEvent == NULL || client_state->hResponsePostedEvent == INVALID_HANDLE_VALUE)
        {
            psmove_DEBUG("Failed to create CL eye response event due to error: %d\n", GetLastError());
            success= false;
        }
    }

    // Open the shared file 
    if (success)
    {
        client_state->hMapFile = 
            OpenFileMappingA(
                FILE_MAP_ALL_ACCESS, // read/write access
                FALSE, // do not inherit the name
                SHARED_MEMORY_NAME);	// name of mapping object
        if (client_state->hMapFile == NULL || client_state->hMapFile == INVALID_HANDLE_VALUE)
        {
            psmove_DEBUG("Failed to open CL eye memory file due to error: %d\n", GetLastError());
            success= false;
        }
    }

    // Memory map the file
    if (success)
    {
        client_state->pMapBuffer = 
            (SharedMemoryBuffer*)MapViewOfFile(
                client_state->hMapFile, // handle to map object
                FILE_MAP_ALL_ACCESS, // read/write permission
                0,
                0,
                sizeof(SharedMemoryBuffer));
        if (client_state->pMapBuffer == NULL)
        {
            psmove_DEBUG("Failed to map CL eye memory file due to error: %d\n", GetLastError());
            success= false;
        }
    }

    return success;
}

void destroy_shared_memory_state(CLEyeClientState *client_state)
{
    if (client_state->pMapBuffer != NULL)
    {
        UnmapViewOfFile(client_state->pMapBuffer);
        client_state->pMapBuffer= NULL;
    }

    if (client_state->hMapFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(client_state->hMapFile);
        client_state->hMapFile= INVALID_HANDLE_VALUE;
    }

    if (client_state->hResponsePostedEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(client_state->hResponsePostedEvent);
        client_state->hResponsePostedEvent= INVALID_HANDLE_VALUE;
    }

    if (client_state->hRequestPostedEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(client_state->hRequestPostedEvent);
        client_state->hRequestPostedEvent= INVALID_HANDLE_VALUE;
    }
}

bool post_and_wait_for_server_response(CLEyeClientState *client_state, int timeout)
{
    SetEvent(client_state->hRequestPostedEvent);

    bool success= WaitForSingleObject(client_state->hResponsePostedEvent, timeout) == WAIT_OBJECT_0;

    ResetEvent(client_state->hRequestPostedEvent);
    ResetEvent(client_state->hResponsePostedEvent);

    return success;
}

#endif //_WIN64
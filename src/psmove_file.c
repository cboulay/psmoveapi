
/**
* PS Move API - An interface for the PS Move Motion Controller
* Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
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

#include "psmove.h"  // includes psmove_file.h
#include "psmove_private.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
    #include <windows.h>
    #include <direct.h>
    #define ENV_USER_HOME "APPDATA"
    #define PATH_SEP "\\"
#else
    #define ENV_USER_HOME "HOME"
    #define PATH_SEP "/"
    #include <unistd.h>
#endif

/* System-wide data directory */
#define PSMOVE_SYSTEM_DATA_DIR "/etc/psmoveapi"

FILE* psmove_file_open(const char *filename, const char *mode)
{
#ifdef _WIN32
    FILE *file_pointer = NULL;
    errno_t error_code = fopen_s(&file_pointer, filename, mode);
    return (error_code == 0) ? file_pointer : NULL;
#else
    return fopen(filename, mode);
#endif // _WIN32
}

void psmove_file_close(FILE* file_pointer)
{
	fclose(file_pointer);
}

enum PSMove_Bool
psmove_util_get_env_string(
	const char *environment_variable_name,
	const size_t buffer_size,
	char *out_buffer)
{
	char *env = getenv(environment_variable_name);
    
    if (env) {
        out_buffer = strdup(env);
        return PSMove_True;
    } else {
        return PSMove_False;
    }
}

enum PSMove_Bool
psmove_util_set_env_string(
	const char *environment_variable_name, 
	const char *string_value)
{
    //int result_code = putenv(string_value);
    int result_code = setenv(environment_variable_name, string_value, 1);
	return (result_code == 0) ? PSMove_True : PSMove_False;
}

int
psmove_util_get_env_int(const char *name)
{
	char buffer[256];

	if (psmove_util_get_env_string(name, 256, buffer)) {
		char *end;
		long result = strtol(buffer, &end, 10);

		if (*end == '\0' && *buffer != '\0') {
			return result;
		}
	}

	return -1;
}

enum PSMove_Bool
psmove_util_set_env_int(
	const char *environment_variable_name,
	const int int_value)
{
	char string_value[64];
	sprintf(string_value, "%d", int_value);
	return psmove_util_set_env_string(environment_variable_name, string_value);
}

const char *
psmove_util_get_data_dir()
{
	static char dir[FILENAME_MAX];

	if (strlen(dir) == 0)
	{
		enum PSMove_Bool success = psmove_util_get_env_string(ENV_USER_HOME, strlen(dir)/sizeof(dir), dir);
		assert(success == PSMove_True);
        strncat(dir, PATH_SEP ".psmoveapi", sizeof(dir) - strlen(dir) - 1);
	}

	return dir;
}

char *
psmove_util_get_file_path(const char *filename)
{
    const char *parent = psmove_util_get_data_dir();
    char *result;
    struct stat st;

#ifndef _WIN32
	// if run as root, use system-wide data directory
	if (geteuid() == 0) {
		parent = PSMOVE_SYSTEM_DATA_DIR;
	}
#endif

    if (stat(filename, &st) == 0) {
		// File exists in the current working directory, prefer that
		// to the file in the default data / configuration directory
		return strdup(filename);
	}

	if (stat(parent, &st) != 0) {
#ifdef _WIN32
		psmove_return_val_if_fail(_mkdir(parent) == 0, NULL);
#else
		psmove_return_val_if_fail(mkdir(parent, 0777) == 0, NULL);
#endif
	}

	size_t result_length = strlen(parent) + 1 + strlen(filename) + 1;
	result = (char *)(malloc(result_length));
    strcpy(result, parent);
	strcat(result, PATH_SEP);
	strcat(result, filename);

	return result;
}

char *
psmove_util_get_system_file_path(const char *filename)
{
	char *result;
	size_t len = strlen(PSMOVE_SYSTEM_DATA_DIR) + 1 + strlen(filename) + 1;

	result= (char *)(malloc(len));
	if (result == NULL) {
		return NULL;
	}
    snprintf(result, len, "%s%s%s", PSMOVE_SYSTEM_DATA_DIR, PATH_SEP, filename);
	return result;
}

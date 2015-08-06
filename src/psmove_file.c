
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
    #ifndef PATH_MAX
        #define PATH_MAX MAX_PATH
    #endif

    #define ENV_USER_HOME "APPDATA"
    #define PATH_SEP "\\"
#else
    #define ENV_USER_HOME "HOME"
    #define PATH_SEP "/"
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
	size_t needed_buffer_size = 0;
	errno_t result_code = getenv_s(&needed_buffer_size, out_buffer, buffer_size, environment_variable_name);
	assert(needed_buffer_size <= buffer_size);

	return (result_code == 0 && needed_buffer_size > 0) ? PSMove_True : PSMove_False;
}

enum PSMove_Bool
psmove_util_set_env_string(
	const char *environment_variable_name, 
	const char *string_value)
{
	errno_t result_code = _putenv_s(environment_variable_name, string_value);

	return (result_code == 0) ? PSMove_True : PSMove_False;
}

int
psmove_util_get_env_int(const char *name)
{
	char buffer[256];

	if (psmove_util_get_env_string(name, _countof(buffer), buffer)) {
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
	sprintf_s(string_value, sizeof(string_value), "%d", int_value);

	return psmove_util_set_env_string(environment_variable_name, string_value);
}

const char *
psmove_util_get_data_dir()
{
	static char dir[PATH_MAX];

	if (strlen(dir) == 0)
	{
		enum PSMove_Bool success = psmove_util_get_env_string(ENV_USER_HOME, _countof(dir), dir);
		assert(success == PSMove_True);

		strncat_s(dir, _countof(dir), PATH_SEP ".psmoveapi", sizeof(dir));
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
		return _strdup(filename);
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
	strcpy_s(result, result_length, parent);
	strcat_s(result, result_length, PATH_SEP);
	strcat_s(result, result_length, filename);

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

	_snprintf_s(result, len, _TRUNCATE, "%s%s%s", PSMOVE_SYSTEM_DATA_DIR, PATH_SEP, filename);

	return result;
}

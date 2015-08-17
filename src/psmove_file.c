
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

char *
psmove_util_get_env_string(const char *name)
{
    char *env = getenv(name);

    if (env) {
        return strdup(env);  // Don't forget to free the result.
    }

    return NULL;
}

enum PSMove_Bool
psmove_util_set_env_string(
    const char *environment_variable_name, 
    const char *string_value)
{
    char *full_str;
    size_t full_len = strlen(environment_variable_name) + 1 + strlen(string_value) + 1;
    full_str = (char *)(malloc(full_len * sizeof(char)));  // Can be freed when name removed from environment.
    snprintf(full_str, full_len, "%s=%s", environment_variable_name, string_value);
    int result_code = putenv(full_str);
    return (result_code == 0) ? PSMove_True : PSMove_False;
}

int
psmove_util_get_env_int(const char *name)
{
    char *env = getenv(name);

    if (env) {
        char *end;
        long result = strtol(env, &end, 10);
        if (*end == '\0' && *env != '\0') {
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
    int size = snprintf(NULL, 0, "%d", int_value);
    char * string_value = malloc(size + 1);
    sprintf(string_value, "%d", 132);
    enum PSMove_Bool result = psmove_util_set_env_string(environment_variable_name, string_value);
    free(string_value);
    return result;
}

const char *
psmove_util_get_data_dir()
{
    char* parent = psmove_util_get_env_string(ENV_USER_HOME);
    char* child = ".psmoveapi";
    size_t path_length = strlen(parent) + strlen(PATH_SEP) + strlen(child) + 1;
    char* full_path = (char *)(malloc(path_length * sizeof(char)));
    snprintf(full_path, path_length, "%s%s%s", parent, PATH_SEP, child);
    free(parent);
    return full_path;
}

char *
psmove_util_get_file_path(const char *filename)
{
    const char *parent = psmove_util_get_data_dir();
    char *full_path;
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

    // Make the parent directory if it does not exist
	if (stat(parent, &st) != 0) {
#ifdef _WIN32
		psmove_return_val_if_fail(_mkdir(parent) == 0, NULL);
#else
		psmove_return_val_if_fail(mkdir(parent, 0777) == 0, NULL);
#endif
	}

    size_t path_length = strlen(parent) + strlen(PATH_SEP) + strlen(filename) +1;
    full_path = (char *)(malloc(path_length * sizeof(char)));
    snprintf(full_path, path_length, "%s%s%s", parent, PATH_SEP, filename);

    return full_path;
}

char *
psmove_util_get_system_file_path(const char *filename)
{
    char *full_path;
    size_t path_length = strlen(PSMOVE_SYSTEM_DATA_DIR) + strlen(PATH_SEP) + strlen(filename) + 1;

    full_path = (char *)(malloc(path_length * sizeof(char)));
    if (full_path == NULL) {
		return NULL;
	}
    snprintf(full_path, path_length, "%s%s%s", PSMOVE_SYSTEM_DATA_DIR, PATH_SEP, filename);
    return full_path;
}

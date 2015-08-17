
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

/* Performance measurement structures and functions */
#ifndef __PSMOVE_UNIT_TEST_H
#define __PSMOVE_UNIT_TEST_H

//-- macros ----
#define UNIT_TEST_SUITE_BEGIN() \
	bool success = true; \
	fprintf(stdout, "Running Unit Tests.\n"); \

#define UNIT_TEST_SUITE_CALL_CPP_MODULE(method) \
	bool method(); \
	success&= method(); \

#define UNIT_TEST_SUITE_DECLARE_C_MODULE(method) \
	extern "C" { bool method(); }; 

#define UNIT_TEST_SUITE_CALL_C_MODULE(method) \
	success&= method(); \

#define UNIT_TEST_SUITE_END() \
if (success) \
{ \
	fprintf(stdout, "All Unit Tests Passed.\n"); \
} \
else \
{ \
	fprintf(stdout, "Some Unit Tests Failed!.\n"); \
} \

#define UNIT_TEST_MODULE_BEGIN(name) \
	bool success = true; \
	const char *__module_name= name; \
	fprintf(stdout, "[%s]\n", __module_name); \

#define UNIT_TEST_MODULE_CALL_TEST(method) \
	bool method(); \
	success&= method(); \

#define UNIT_TEST_MODULE_END() \
 	fprintf(stdout, "  %s module - %s\n", __module_name, success ? "PASSED" : "FAILED"); \
	return success; \

#define UNIT_TEST_BEGIN(name) \
	const char *__test_name= name; \
	bool success= true; \

#define UNIT_TEST_COMPLETE() \
 	fprintf(stdout, "    %s - %s\n", __test_name, success ? "PASSED" : "FAILED"); \
	return success; \

#endif // __PSMOVE_UNIT_TEST_H
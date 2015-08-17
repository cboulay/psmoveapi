 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
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
#include <stdio.h>
#include <stdlib.h>
#include "psmove.h"
#include "unit_test.h"

//-- prototypes -----
UNIT_TEST_SUITE_DECLARE_C_MODULE(run_vector_unit_tests)

//-- entry point -----
int
main(int argc, char* argv[])
{
	if (!psmove_init(PSMOVE_CURRENT_VERSION)) 
	{
		fprintf(stderr, "PS Move API init failed (wrong version?)\n");
		exit(1);
	}

	UNIT_TEST_SUITE_BEGIN()
		UNIT_TEST_SUITE_CALL_C_MODULE(run_vector_unit_tests);
		UNIT_TEST_SUITE_CALL_CPP_MODULE(run_quaternion_unit_tests);
		UNIT_TEST_SUITE_CALL_CPP_MODULE(run_matrix_unit_tests);
		UNIT_TEST_SUITE_CALL_CPP_MODULE(run_alignment_unit_tests);
	UNIT_TEST_SUITE_END()

	psmove_shutdown();

	return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
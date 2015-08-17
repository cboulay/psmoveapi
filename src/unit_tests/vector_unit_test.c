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
#include <assert.h>
#include <stdbool.h>

#include "psmove.h"
#include "math/psmove_vector.h"
#include "unit_test.h"

//-- public interface -----
bool run_vector_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("vector_math")
		UNIT_TEST_MODULE_CALL_TEST(psmove_vector_test_min_component)
		UNIT_TEST_MODULE_CALL_TEST(psmove_vector_test_max_component)
		UNIT_TEST_MODULE_CALL_TEST(psmove_vector_test_min)
		UNIT_TEST_MODULE_CALL_TEST(psmove_vector_test_max)
		UNIT_TEST_MODULE_CALL_TEST(psmove_vector_test_normalize)
	UNIT_TEST_MODULE_END()
}

//-- private functions -----
bool
psmove_vector_test_min_component()
{
	UNIT_TEST_BEGIN("min component")
	PSMove_3AxisVector v = psmove_3axisvector_xyz(1.f, 2.f, 3.f);
	float min_component = psmove_3axisvector_min_component(&v);
	success &= is_nearly_equal(min_component, 1.f, k_real_epsilon);
	assert(success);

	UNIT_TEST_COMPLETE()
}

bool
psmove_vector_test_max_component()
{
	UNIT_TEST_BEGIN("max component")
	PSMove_3AxisVector v = psmove_3axisvector_xyz(1.f, 2.f, 3.f);
	float max_component = psmove_3axisvector_max_component(&v);
	success &= is_nearly_equal(max_component, 3.f, k_real_epsilon);
	assert(success);

	UNIT_TEST_COMPLETE()
}

bool
psmove_vector_test_min()
{
	UNIT_TEST_BEGIN("min vector")
		PSMove_3AxisVector v1 = psmove_3axisvector_xyz(1.f, 2.f, 2.f);
	PSMove_3AxisVector v2 = psmove_3axisvector_xyz(2.f, 1.f, 1.f);
	PSMove_3AxisVector min_v_expected = psmove_3axisvector_xyz(1.f, 1.f, 1.f);
	PSMove_3AxisVector min_v_acutal = psmove_3axisvector_min_vector(&v1, &v2);
	success &= psmove_3axisvector_is_nearly_equal(&min_v_expected, &min_v_acutal, k_real_epsilon);
	assert(success);

	UNIT_TEST_COMPLETE()
}

bool
psmove_vector_test_max()
{
	UNIT_TEST_BEGIN("max vector")
	PSMove_3AxisVector v1 = psmove_3axisvector_xyz(1.f, 2.f, 2.f);
	PSMove_3AxisVector v2 = psmove_3axisvector_xyz(2.f, 1.f, 1.f);
	PSMove_3AxisVector max_v_expected = psmove_3axisvector_xyz(2.f, 2.f, 2.f);
	PSMove_3AxisVector max_v_acutal = psmove_3axisvector_max_vector(&v1, &v2);
	success &= psmove_3axisvector_is_nearly_equal(&max_v_expected, &max_v_acutal, k_real_epsilon);
	assert(success);

	UNIT_TEST_COMPLETE()
}

bool
psmove_vector_test_normalize()
{
	UNIT_TEST_BEGIN("normalize vector")

	PSMove_3AxisVector n = psmove_3axisvector_xyz(1.f, 2.f, 3.f);
	psmove_3axisvector_normalize_with_default(&n, k_psmove_vector_zero);
	success &= is_nearly_equal(psmove_3axisvector_length(&n), 1.f, k_normal_epsilon);
	assert(success);

	PSMove_3AxisVector v = *k_psmove_vector_zero;
	float length = psmove_3axisvector_normalize_with_default(&v, k_psmove_vector_one);
	success &= is_nearly_equal(length, 0.f, k_normal_epsilon);
	assert(success);
	success &= psmove_3axisvector_is_nearly_equal(&v, k_psmove_vector_one, k_normal_epsilon);
	assert(success);

	UNIT_TEST_COMPLETE()
}
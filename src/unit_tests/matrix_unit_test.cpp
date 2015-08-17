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

#include "psmove.h"
#include "math/psmove_quaternion.hpp"
#include "unit_test.h"

//-- prototypes -----

//-- public interface -----
bool run_matrix_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("matrix_math")
		UNIT_TEST_MODULE_CALL_TEST(psmove_matrix_test_inverse_multiplication);
	UNIT_TEST_MODULE_END()
}

//-- private functions -----

bool
psmove_matrix_test_inverse_multiplication()
{
	UNIT_TEST_BEGIN("matrix inverse multiplication")

	Eigen::Quaternionf q = Eigen::Quaternionf(0.980671287f, 0.177366823f, 0.0705093816f, 0.0430502370f);
	assert_quaternion_is_normalized(q);
	Eigen::Quaternionf q_inv = q.conjugate();

	Eigen::Matrix3f m = psmove_quaternion_to_clockwise_matrix3f(q);
	Eigen::Matrix3f m_inv = psmove_quaternion_to_clockwise_matrix3f(q_inv);
	Eigen::Matrix3f m_actual = m * m_inv;
	Eigen::Matrix3f m_expected;

	m_expected.setIdentity();

	success &= m_actual.isApprox(m_expected, k_normal_epsilon);
	assert(success);

	UNIT_TEST_COMPLETE()
}
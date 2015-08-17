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
#include "math/psmove_alignment.hpp"
#include "unit_test.h"

//-- prototypes -----

//-- public interface -----
bool run_alignment_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("alignment_math")
		UNIT_TEST_MODULE_CALL_TEST(psmove_alignment_test_vector_alignment);
		UNIT_TEST_MODULE_CALL_TEST(psmove_alignment_test_vector_frame_alignment);
	UNIT_TEST_MODULE_END()
}

//-- private functions -----

bool
psmove_alignment_test_vector_alignment()
{
	UNIT_TEST_BEGIN("vector alignment")

	// Test some typical direction
	{
		Eigen::Vector3f from = Eigen::Vector3f(1.0f, 0.f, 0.0f);
		Eigen::Vector3f to = Eigen::Vector3f(0.0f, 0.f, 1.0f);
		Eigen::Quaternionf q_expected = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitY());
		Eigen::Quaternionf q_actual = psmove_alignment_quaternion_between_vectors(from, to);
		Eigen::Vector3f exptected_from_rotated = psmove_vector3f_clockwise_rotate(q_expected, from);
		Eigen::Vector3f actual_from_rotated = psmove_vector3f_clockwise_rotate(q_actual, from);
		success &= to.isApprox(actual_from_rotated, k_normal_epsilon);
		success &= to.isApprox(exptected_from_rotated, k_normal_epsilon);
		assert(success);
	}

	// Test some typical direction
	{
		Eigen::Vector3f from = Eigen::Vector3f(0.0f, 1.f, 0.0f);
		Eigen::Vector3f to = Eigen::Vector3f(0.288588405f, -0.909195602f, 0.300133437f);
		Eigen::Quaternionf q = psmove_alignment_quaternion_between_vectors(from, to);
		Eigen::Vector3f from_rotated = psmove_vector3f_clockwise_rotate(q, from);
		success &= to.isApprox(from_rotated, k_normal_epsilon);
		assert(success);
	}

	// Test direct opposite directions
	{
		Eigen::Vector3f from = Eigen::Vector3f(0.0f, 1.f, 0.0f);
		Eigen::Vector3f to = Eigen::Vector3f(0.0f, -1.f, 0.0f);
		Eigen::Quaternionf q = psmove_alignment_quaternion_between_vectors(from, to);
		Eigen::Vector3f from_rotated = psmove_vector3f_clockwise_rotate(q, from);
		success &= to.isApprox(from_rotated, k_normal_epsilon);
		assert(success);
	}

	UNIT_TEST_COMPLETE()
}

bool
psmove_alignment_test_vector_frame_alignment()
{
	UNIT_TEST_BEGIN("inverse multiplication")

	// Ideal case with no local minima
	{
		Eigen::Vector3f from0 = Eigen::Vector3f::UnitX();
		Eigen::Vector3f from1 = Eigen::Vector3f::UnitY();

		Eigen::Quaternionf q_expected = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitZ());
		Eigen::Vector3f to0 = psmove_vector3f_clockwise_rotate(q_expected, from0);
		Eigen::Vector3f to1 = psmove_vector3f_clockwise_rotate(q_expected, from1);

		const Eigen::Vector3f* from[2] = { &from0, &from1 };
		const Eigen::Vector3f* to[2] = { &to0, &to1 };

		Eigen::Quaternionf q_actual;
		success =
			psmove_alignment_quaternion_between_vector_frames(
				from, to, k_normal_epsilon, Eigen::Quaternionf::Identity(), q_actual);
		assert(success);

		success &= q_expected.isApprox(q_actual, k_normal_epsilon);
		assert(success);

		Eigen::Vector3f from0_rotated = psmove_vector3f_clockwise_rotate(q_actual, *from[0]);
		Eigen::Vector3f from1_rotated = psmove_vector3f_clockwise_rotate(q_actual, *from[1]);
		success &= to0.isApprox(from0_rotated, k_normal_epsilon);
		success &= to1.isApprox(from1_rotated, k_normal_epsilon);
		assert(success);
	}

	// Real world case
	{
		// Accelerometer and Magnetometer measurements in identity pose
		Eigen::Vector3f from0 = Eigen::Vector3f(-0.000000000f, -1.00000000f, 0.000000000f);
		Eigen::Vector3f from1 = Eigen::Vector3f(0.288588405f, -0.909195602f, 0.300133437f);

		// Accelerometer and Magnetometer measurements when in psmove cradle
		Eigen::Vector3f to0 = Eigen::Vector3f(0.0745674670f, -0.00552607095f, 0.997200668f);
		Eigen::Vector3f to1 = Eigen::Vector3f(0.165051967f, 0.264549226f, 0.950142860f);

		const Eigen::Vector3f* from[2] = { &from0, &from1 };
		const Eigen::Vector3f* to[2] = { &to0, &to1 };

		Eigen::Quaternionf q_actual;
		success =
			psmove_alignment_quaternion_between_vector_frames(
				from, to, 0.15f, Eigen::Quaternionf::Identity(), q_actual);
		assert(success);

		Eigen::Vector3f from0_rotated = psmove_vector3f_clockwise_rotate(q_actual, *from[0]);
		Eigen::Vector3f from1_rotated = psmove_vector3f_clockwise_rotate(q_actual, *from[1]);
		success &= to0.isApprox(from0_rotated, 0.15f);
		success &= to1.isApprox(from1_rotated, 0.15f);
		assert(success);
	}

	UNIT_TEST_COMPLETE()
}
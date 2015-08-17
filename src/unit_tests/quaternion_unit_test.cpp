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
static bool psmove_quaternion_test_rotation_method_consistency(const Eigen::Quaternionf &q, const Eigen::Vector3f &v);

//-- public interface -----
bool run_quaternion_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("quaternion_math")
		UNIT_TEST_MODULE_CALL_TEST(psmove_quaternion_test_inverse_multiplication);
		UNIT_TEST_MODULE_CALL_TEST(psmove_quaternion_test_euler_angles);
		UNIT_TEST_MODULE_CALL_TEST(psmove_quaternion_test_matrix_conversion);
		UNIT_TEST_MODULE_CALL_TEST(psmove_quaternion_test_rotate_with_angle_axis_quaternion)
		UNIT_TEST_MODULE_CALL_TEST(psmove_quaternion_test_rotate_with_arbitrary_quaternion)
	UNIT_TEST_MODULE_END()
}

//-- private functions -----
bool
psmove_quaternion_test_inverse_multiplication()
{
	UNIT_TEST_BEGIN("inverse multiplication")
	Eigen::Quaternionf q = Eigen::Quaternionf(0.980671287f, 0.177366823f, 0.0705093816f, 0.0430502370f);
	assert_quaternion_is_normalized(q);
	Eigen::Quaternionf q_inverse = q.conjugate();
	Eigen::Quaternionf q_identity = q * q_inverse;

	success &= q_identity.isApprox(Eigen::Quaternionf::Identity(), k_normal_epsilon);
	assert(success);
	UNIT_TEST_COMPLETE()
}

bool
psmove_quaternion_test_euler_angles()
{
	UNIT_TEST_BEGIN("euler angles")
		Eigen::Quaternionf q_pitch90 = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitX());
	Eigen::Quaternionf q_yaw90 = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitY());
	Eigen::Quaternionf q_roll90 = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitZ());

	Eigen::Quaternionf q_euler_yaw90 = psmove_quaterion_yaw_pitch_roll(k_real_half_pi, 0.f, 0.f);
	Eigen::Quaternionf q_euler_pitch90 = psmove_quaterion_yaw_pitch_roll(0.f, k_real_half_pi, 0.f);
	Eigen::Quaternionf q_euler_roll90 = psmove_quaterion_yaw_pitch_roll(0.f, 0.f, k_real_half_pi);

	success &= q_pitch90.isApprox(q_euler_pitch90, k_normal_epsilon);
	assert(success);
	success &= q_yaw90.isApprox(q_euler_yaw90, k_normal_epsilon);
	assert(success);
	success &= q_roll90.isApprox(q_euler_roll90, k_normal_epsilon);
	assert(success);

	float yaw_radians, pitch_radians, roll_radians;
	
	psmove_quaternion_get_yaw_pitch_roll(q_euler_pitch90, &yaw_radians, &pitch_radians, &roll_radians);
	success &= is_nearly_equal(yaw_radians, 0.f, k_normal_epsilon);
	success &= is_nearly_equal(pitch_radians, k_real_half_pi, k_normal_epsilon);
	success &= is_nearly_equal(roll_radians, 0.f, k_normal_epsilon);
	assert(success);

	psmove_quaternion_get_yaw_pitch_roll(q_euler_yaw90, &yaw_radians, &pitch_radians, &roll_radians);
	success &= is_nearly_equal(yaw_radians, k_real_half_pi, k_normal_epsilon);
	success &= is_nearly_equal(pitch_radians, 0.f, k_normal_epsilon);
	success &= is_nearly_equal(roll_radians, 0.f, k_normal_epsilon);
	assert(success);

	psmove_quaternion_get_yaw_pitch_roll(q_euler_roll90, &yaw_radians, &pitch_radians, &roll_radians);
	success &= is_nearly_equal(yaw_radians, 0.f, k_normal_epsilon);
	success &= is_nearly_equal(pitch_radians, 0.f, k_normal_epsilon);
	success &= is_nearly_equal(roll_radians, k_real_half_pi, k_normal_epsilon);
	assert(success);

	UNIT_TEST_COMPLETE()
}

bool
psmove_quaternion_test_matrix_conversion()
{
	UNIT_TEST_BEGIN("matrix conversion")
	Eigen::Quaternionf q = Eigen::Quaternionf(0.980671287f, 0.177366823f, 0.0705093816f, 0.0430502370f);
	assert_quaternion_is_normalized(q);

	Eigen::Matrix3f m = psmove_quaternion_to_clockwise_matrix3f(q);
	Eigen::Quaternionf q_copy = psmove_matrix3f_to_clockwise_quaternion(m);

	success &= q.isApprox(q_copy, k_normal_epsilon);
	assert(success);
	UNIT_TEST_COMPLETE()
}

bool
psmove_quaternion_test_rotate_with_angle_axis_quaternion()
{
	UNIT_TEST_BEGIN("rotate with angle axis quaternion")
	Eigen::Quaternionf q_pitch90 = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitX());
	Eigen::Quaternionf q_yaw90 = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitY());
	Eigen::Quaternionf q_roll90 = psmove_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitZ());
	Eigen::Quaternionf q_euler_pitch90 = psmove_quaterion_yaw_pitch_roll(0.f, k_real_half_pi, 0.f);
	Eigen::Quaternionf q_euler_yaw90 = psmove_quaterion_yaw_pitch_roll(k_real_half_pi, 0.f, 0.f);
	Eigen::Quaternionf q_euler_roll90 = psmove_quaterion_yaw_pitch_roll(0.f, 0.f, k_real_half_pi);
	Eigen::Vector3f v_x = Eigen::Vector3f::UnitX();
	Eigen::Vector3f v_y = Eigen::Vector3f::UnitY();
	Eigen::Vector3f v_z = Eigen::Vector3f::UnitZ();

	// Make sure we get the answers we expect with angle axis rotation
	Eigen::Vector3f v_x_rotated = psmove_vector3f_clockwise_rotate(q_yaw90, v_x);
	success &= v_x_rotated.isApprox(Eigen::Vector3f::UnitZ(), k_normal_epsilon);
	assert(success);
	Eigen::Vector3f v_y_rotated = psmove_vector3f_clockwise_rotate(q_roll90, v_y);
	success &= v_y_rotated.isApprox(Eigen::Vector3f::UnitX(), k_normal_epsilon);
	assert(success);
	Eigen::Vector3f v_z_rotated = psmove_vector3f_clockwise_rotate(q_pitch90, v_z);
	success &= v_z_rotated.isApprox(Eigen::Vector3f::UnitY(), k_normal_epsilon);
	assert(success);

	// Make sure we get the answers we expect with euler angle rotation
	v_x_rotated = psmove_vector3f_clockwise_rotate(q_euler_yaw90, v_x);
	success &= v_x_rotated.isApprox(Eigen::Vector3f::UnitZ(), k_normal_epsilon);
	assert(success);
	v_y_rotated = psmove_vector3f_clockwise_rotate(q_euler_roll90, v_y);
	success &= v_y_rotated.isApprox(Eigen::Vector3f::UnitX(), k_normal_epsilon);
	assert(success);
	v_z_rotated = psmove_vector3f_clockwise_rotate(q_euler_pitch90, v_z);
	success &= v_z_rotated.isApprox(Eigen::Vector3f::UnitY(), k_normal_epsilon);
	assert(success);

	// Make sure all rotation methods are consistent with each other
	success &= psmove_quaternion_test_rotation_method_consistency(q_roll90, v_x);
	success &= psmove_quaternion_test_rotation_method_consistency(q_pitch90, v_y);
	success &= psmove_quaternion_test_rotation_method_consistency(q_yaw90, v_z);

	UNIT_TEST_COMPLETE()
}

bool
psmove_quaternion_test_rotate_with_arbitrary_quaternion()
{
	UNIT_TEST_BEGIN("rotate with arbitrary quaternion")
	Eigen::Quaternionf q = Eigen::Quaternionf(0.980671287f, 0.177366823f, 0.0705093816f, 0.0430502370f);
	Eigen::Vector3f v = Eigen::Vector3f(0.288588405f, -0.909195602f, 0.300133437f);

	success &= psmove_quaternion_test_rotation_method_consistency(q, v);
	UNIT_TEST_COMPLETE()
}

static bool
psmove_quaternion_test_rotation_method_consistency(const Eigen::Quaternionf &q, const Eigen::Vector3f &v)
{
	bool success = true;

	assert_quaternion_is_normalized(q);

	// This is the same as computing the rotation by computing: q^-1*[0,v]*q, but cheaper
	Eigen::Vector3f v_rotated = psmove_vector3f_clockwise_rotate(q, v);

	// Make sure doing the matrix based rotation performs the same result
	{
		Eigen::Matrix3f m = psmove_quaternion_to_clockwise_matrix3f(q);
		Eigen::Vector3f v_test = m * v;

		success &= v_test.isApprox(v_rotated, k_normal_epsilon);
		assert(success);
	}

	// Make sure the Hamilton product style rotation matches
	{
		Eigen::Quaternionf v_as_quaternion = Eigen::Quaternionf(0.f, v.x(), v.y(), v.z());
		Eigen::Quaternionf q_inv = q.conjugate();
		Eigen::Quaternionf qinv_v = q_inv * v_as_quaternion;
		Eigen::Quaternionf qinv_v_q = qinv_v * q;

		success &=
			is_nearly_equal(qinv_v_q.w(), 0.f, k_normal_epsilon) &&
			is_nearly_equal(qinv_v_q.x(), v_rotated.x(), k_normal_epsilon) &&
			is_nearly_equal(qinv_v_q.y(), v_rotated.y(), k_normal_epsilon) &&
			is_nearly_equal(qinv_v_q.z(), v_rotated.z(), k_normal_epsilon);
		assert(success);
	}

	return success;
}
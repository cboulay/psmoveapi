
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

/* Math functions related to quaternions */
#ifndef __PSMOVE_QUATERNION_H
#define __PSMOVE_QUATERNION_H

//-- includes -----
#include "psmove_advanced_math.hpp"
#include "psmove_vector.h"

//-- pre-declarations ----

//-- structures -----

//-- constants -----
ADDAPI extern const Eigen::Quaternionf *k_psmove_quaternion_zero;

//-- interface -----

// Creates a quaternion that rotates about the x(pitch), y(yaw), and z(roll) axes clockwise for a positive angle
ADDAPI Eigen::Quaternionf
ADDCALL psmove_quaterion_yaw_pitch_roll(float yaw_radians, float pitch_radians, float roll_radians);

// Extract the pitch(x), yaw(y), and roll(z) angles from a quaternion
// Positive values can be interpreted clockwise rotations about their respective axis
ADDAPI void
ADDCALL psmove_quaternion_get_yaw_pitch_roll(
	const Eigen::Quaternionf &q, float *out_yaw_radians, float *out_pitch_radians, float *out_roll_radians);

// Creates a quaternion that rotates clockwise about the axis for a positive angle
// when appied with psmove_vector_clockwise_rotate()
ADDAPI Eigen::Quaternionf
ADDCALL psmove_quaternion_angle_axis(float radians, const Eigen::Vector3f &axis);

ADDAPI Eigen::Quaternionf
ADDCALL psmove_quaternion_normalized_lerp(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b, const float u);

ADDAPI Eigen::Quaternionf
ADDCALL psmove_quaternion_safe_divide_with_default(const Eigen::Quaternionf &q, const float divisor, const Eigen::Quaternionf &default_result);

ADDAPI float
ADDCALL psmove_quaternion_normalize_with_default(Eigen::Quaternionf &inout_v, const Eigen::Quaternionf &default_result);

ADDAPI bool
ADDCALL psmove_quaternion_is_valid(const Eigen::Quaternionf &q);

ADDAPI Eigen::Vector3f
ADDCALL psmove_vector3f_clockwise_rotate(const Eigen::Quaternionf &q, const Eigen::Vector3f &v);

// Applies quaternion as (q^-1*v*q)
ADDAPI PSMove_3AxisVector
ADDCALL psmove_vector_clockwise_rotate(const Eigen::Quaternionf &q, const PSMove_3AxisVector &v);

ADDAPI Eigen::Matrix3f
ADDCALL psmove_quaternion_to_clockwise_matrix3f(const Eigen::Quaternionf &q);

ADDAPI Eigen::Quaternionf
ADDCALL psmove_matrix3f_to_clockwise_quaternion(const Eigen::Matrix3f &m);

//-- macros -----
#define assert_quaternion_is_normalized(q) assert(is_nearly_equal(q.squaredNorm(), 1.f, k_normal_epsilon))

#endif // __PSMOVE_QUATERNION_H
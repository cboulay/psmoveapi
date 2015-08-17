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

/* General math utility methods for aligning frames of reference */
#ifndef __PSMOVE_ALIGNMENT_H
#define __PSMOVE_ALIGNMENT_H

//-- includes -----
#include "psmove_advanced_math.hpp"

//-- pre-declarations -----

//-- constants ----

//-- macros ----

//-- inline -----

//-- interface -----
ADDAPI Eigen::Quaternionf
ADDCALL psmove_alignment_quaternion_between_vectors(const Eigen::Vector3f &from, const Eigen::Vector3f &to);

ADDAPI void
ADDCALL psmove_alignment_compute_objective_vector(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, const Eigen::Vector3f &s,
	Eigen::Matrix<float,3,1> &out_f, float *out_squared_error);

ADDAPI void
ADDCALL psmove_alignment_compute_objective_jacobian(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, Eigen::Matrix<float, 4, 3> &J);

ADDAPI bool
ADDCALL psmove_alignment_quaternion_between_vector_frames(
	const Eigen::Vector3f* from[2], const Eigen::Vector3f* to[2], const float tolerance, const Eigen::Quaternionf &initial_q,
	Eigen::Quaternionf &out_q);

#endif // __PSMOVE_ALIGNMENT_H
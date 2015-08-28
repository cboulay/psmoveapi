
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
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "psmove.h"
#include "psmove_private.h"
#include "psmove_orientation.h"

#include "math/psmove_quaternion.hpp"
#include "math/psmove_alignment.hpp"
#include "math/psmove_vector.h"

#ifdef CMAKE_BUILD
#include <unistd.h>
#endif

//-- constants -----
#define SAMPLE_FREQUENCY 120.f

// Madgwick MARG Filter Constants
#define gyroMeasDrift 3.14159265358979f * (0.9f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrtf(3.0f / 4.0f) * gyroMeasError // compute beta
#define gyroMeasError 3.14159265358979f * (1.5f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define zeta sqrtf(3.0f / 4.0f) * gyroMeasDrift // compute zeta

// Complementary ARG Filter constants
#define k_base_earth_frame_align_weight 0.02f

// Calibration Pose transform
const PSMove_3AxisTransform g_psmove_identity_pose_upright = {1,0,0, 0,1,0, 0,0,1};
const PSMove_3AxisTransform *k_psmove_identity_pose_upright = &g_psmove_identity_pose_upright;

const PSMove_3AxisTransform g_psmove_identity_pose_laying_flat = {1,0,0, 0,0,-1, 0,1,0};
const PSMove_3AxisTransform *k_psmove_identity_pose_laying_flat = &g_psmove_identity_pose_laying_flat;

//Sensor Transforms
const PSMove_3AxisTransform g_psmove_sensor_transform_identity = {1,0,0, 0,1,0, 0,0,1};
const PSMove_3AxisTransform *k_psmove_sensor_transform_identity = &g_psmove_sensor_transform_identity;

const PSMove_3AxisTransform g_psmove_sensor_transform_opengl = {1,0,0, 0,0,1, 0,-1,0};
const PSMove_3AxisTransform *k_psmove_sensor_transform_opengl= &g_psmove_sensor_transform_opengl;

//-- structures ----
struct _PSMoveMadwickMARGState
{
	// estimate gyroscope biases error
	Eigen::Quaternionf omega_bias; 
};
typedef struct _PSMoveMadwickMARGState PSMoveMadgwickMARGState;

struct _PSMovComplementaryMARGState
{
	float mg_weight;
};
typedef struct _PSMovComplementaryMARGState PSMoveComplementaryMARGState;

struct _PSMoveOrientation {
    PSMove *move;

    /* Current sampling frequency */
    float sample_freq;

    /* Sample frequency measurements */
    long sample_freq_measure_start;
    long sample_freq_measure_count;

    /* Output value as quaternion */
	Eigen::Quaternionf quaternion;

    /* Quaternion measured when controller points towards camera */
	Eigen::Quaternionf reset_quaternion;

	/* Transforms the gravity and magnetometer calibration vectors recorded when the 
	   controller was help upright during calibration. This is needed if you want the "identity pose" 
	   to be something besides the one used during calibration */
	PSMove_3AxisTransform calibration_transform;

	/* Transforms the sensor data from PSMove Space to some user defined coordinate space */
	PSMove_3AxisTransform sensor_transform;

	/* Per filter type data */
	enum PSMoveOrientation_Fusion_Type fusion_type;
	struct
	{
		PSMoveMadgwickMARGState madgwick_marg_state;
		PSMoveComplementaryMARGState complementary_marg_state;
	} fusion_state;
};

//-- prototypes -----
static void _psmove_orientation_fusion_imu_update(
	PSMoveOrientation *orientation_state,
	float deltat,
	const Eigen::Vector3f &sensor_gyroscope,
	const Eigen::Vector3f &sensor_accelerometer);
static void _psmove_orientation_fusion_madgwick_marg_update(
	PSMoveOrientation *orientation_state,
	float deltat,
	const Eigen::Vector3f &sensor_gyroscope,
	const Eigen::Vector3f &sensor_accelerometer,
	const Eigen::Vector3f &sensor_magnetometer);
static void _psmove_orientation_fusion_complementary_marg_update(
	PSMoveOrientation *orientation_state,
	float delta_t,
	const Eigen::Vector3f &sensor_gyroscope,
	const Eigen::Vector3f &sensor_acceleration,
	const Eigen::Vector3f &sensor_magnetometer);

//-- public methods -----
PSMoveOrientation *
psmove_orientation_new(PSMove *move)
{
    psmove_return_val_if_fail(move != NULL, NULL);

    if (!psmove_has_calibration(move)) {
        psmove_DEBUG("Can't create orientation - no calibration!\n");
        return NULL;
    }

	PSMoveOrientation *orientation_state = (PSMoveOrientation *)calloc(1, sizeof(PSMoveOrientation));

    orientation_state->move = move;

    /* Initial sampling frequency */
	orientation_state->sample_freq = SAMPLE_FREQUENCY;

    /* Measurement */
    orientation_state->sample_freq_measure_start = psmove_util_get_ticks();
    orientation_state->sample_freq_measure_count = 0;

    /* Initial quaternion */
	orientation_state->quaternion.setIdentity();
	orientation_state->reset_quaternion.setIdentity();

	/* Initialize data specific to the selected filter */
	psmove_orientation_set_fusion_type(orientation_state, OrientationFusion_ComplementaryMARG);

	/* Set the transform used re-orient the calibration data used by the orientation fusion algorithm */
	psmove_orientation_set_calibration_transform(orientation_state, k_psmove_identity_pose_laying_flat);

	/* Set the transform used re-orient the sensor data used by the orientation fusion algorithm */
	psmove_orientation_set_sensor_data_transform(orientation_state, k_psmove_sensor_transform_opengl);

    return orientation_state;
}

void
psmove_orientation_set_fusion_type(PSMoveOrientation *orientation_state, enum PSMoveOrientation_Fusion_Type fusion_type)
{
	orientation_state->fusion_type = fusion_type;

	switch (fusion_type)
	{
	case OrientationFusion_None:
	case OrientationFusion_MadgwickIMU:
		// No initialization
		break;
	case OrientationFusion_MadgwickMARG:
		{
			PSMoveMadgwickMARGState *marg_state = &orientation_state->fusion_state.madgwick_marg_state;

			marg_state->omega_bias = Eigen::Quaternionf(0.f, 0.f, 0.f, 0.f);
		}
		break;
	case OrientationFusion_ComplementaryMARG:
		{
			PSMoveComplementaryMARGState *marg_state = &orientation_state->fusion_state.complementary_marg_state;

			// Start off fully using the rotation from earth-frame.
			// Then drop down 
			marg_state->mg_weight = 1.f;
		}
		break;
	default:
		break;
	}
}

void
psmove_orientation_set_calibration_transform(PSMoveOrientation *orientation_state, const PSMove_3AxisTransform *transform)
{
	psmove_return_if_fail(orientation_state != NULL);
	psmove_return_if_fail(transform != NULL);

	orientation_state->calibration_transform= *transform;
}

void
psmove_orientation_set_sensor_data_transform(PSMoveOrientation *orientation_state, const PSMove_3AxisTransform *transform)
{
	psmove_return_if_fail(orientation_state != NULL);
	psmove_return_if_fail(transform != NULL);

	orientation_state->sensor_transform= *transform;
}

PSMove_3AxisVector
psmove_orientation_get_gravity_calibration_direction(PSMoveOrientation *orientation_state)
{
	psmove_return_val_if_fail(orientation_state != NULL, *k_psmove_vector_zero);

	PSMove_3AxisVector identity_g;
	psmove_get_identity_gravity_calibration_direction(orientation_state->move, &identity_g);

	// First apply the calibration data transform.
	// This allows us to pretend the "identity pose" was some other orientation the vertical during calibration
	identity_g= psmove_3axisvector_apply_transform(&identity_g, &orientation_state->calibration_transform);

	// Next apply the sensor data transform.
	// This allows us to pretend the sensors are in some other coordinate system (like OpenGL where +Y is up)
	identity_g= psmove_3axisvector_apply_transform(&identity_g, &orientation_state->sensor_transform);

	return identity_g;
}

PSMove_3AxisVector
psmove_orientation_get_magnetometer_calibration_direction(PSMoveOrientation *orientation_state)
{
	psmove_return_val_if_fail(orientation_state != NULL, *k_psmove_vector_zero);

	PSMove_3AxisVector identity_m;
	psmove_get_identity_magnetometer_calibration_direction(orientation_state->move, &identity_m);

	// First apply the calibration data transform.
	// This allows us to pretend the "identity pose" was some other orientation the vertical during calibration
	identity_m= psmove_3axisvector_apply_transform(&identity_m, &orientation_state->calibration_transform);

	// Next apply the sensor data transform.
	// This allows us to pretend the sensors are in some other coordinate system (like OpenGL where +Y is up)
	identity_m= psmove_3axisvector_apply_transform(&identity_m, &orientation_state->sensor_transform);

	return identity_m;
}

PSMove_3AxisVector
psmove_orientation_get_accelerometer_vector(PSMoveOrientation *orientation_state, enum PSMove_Frame frame)
{
	psmove_return_val_if_fail(orientation_state != NULL, *k_psmove_vector_zero);

	float ax, ay, az;
	psmove_get_accelerometer_frame(orientation_state->move, frame, &ax, &ay, &az);

	// Apply the "identity pose" transform
	PSMove_3AxisVector a = psmove_3axisvector_xyz(ax, ay, az);
	a= psmove_3axisvector_apply_transform(&a, &orientation_state->sensor_transform);

	return a;
}

PSMove_3AxisVector
psmove_orientation_get_accelerometer_normalized_vector(PSMoveOrientation *orientation_state, enum PSMove_Frame frame)
{
	psmove_return_val_if_fail(orientation_state != NULL, *k_psmove_vector_zero);

	PSMove_3AxisVector a= psmove_orientation_get_accelerometer_vector(orientation_state, frame);

	// Normalize the accelerometer vector
	psmove_3axisvector_normalize_with_default(&a, k_psmove_vector_zero);

	return a;
}

PSMove_3AxisVector
psmove_orientation_get_gyroscope_vector(PSMoveOrientation *orientation_state, enum PSMove_Frame frame)
{
	psmove_return_val_if_fail(orientation_state != NULL, *k_psmove_vector_zero);

	float omega_x, omega_y, omega_z;
	psmove_get_gyroscope_frame(orientation_state->move, frame, &omega_x, &omega_y, &omega_z);

	// Apply the "identity pose" transform
	PSMove_3AxisVector omega = psmove_3axisvector_xyz(omega_x, omega_y, omega_z);
	omega= psmove_3axisvector_apply_transform(&omega, &orientation_state->sensor_transform);

	return omega;
}

PSMove_3AxisVector
psmove_orientation_get_magnetometer_normalized_vector(PSMoveOrientation *orientation_state)
{
    psmove_return_val_if_fail(orientation_state != NULL, *k_psmove_vector_zero);

	PSMove_3AxisVector m;
	psmove_get_magnetometer_3axisvector(orientation_state->move, &m);
	m= psmove_3axisvector_apply_transform(&m, &orientation_state->sensor_transform);

	// Make sure we always return a normalized direction
	psmove_3axisvector_normalize_with_default(&m, k_psmove_vector_zero);

	return m;
}

void
psmove_orientation_update(PSMoveOrientation *orientation_state)
{
    psmove_return_if_fail(orientation_state != NULL);

    int frame_half;
    long now = psmove_util_get_ticks();

    if (now - orientation_state->sample_freq_measure_start >= 1000) 
	{
        float measured = ((float)orientation_state->sample_freq_measure_count) /
            ((float)(now-orientation_state->sample_freq_measure_start))*1000.f;
        psmove_DEBUG("Measured sample_freq: %f\n", measured);

        orientation_state->sample_freq = measured;
        orientation_state->sample_freq_measure_start = now;
        orientation_state->sample_freq_measure_count = 0;
    }

    /* We get 2 measurements per call to psmove_poll() */
    orientation_state->sample_freq_measure_count += 2;

	Eigen::Quaternionf quaternion_backup = orientation_state->quaternion;
	float deltaT = 1.f / fmax(orientation_state->sample_freq, SAMPLE_FREQUENCY); // time delta = 1/frequency

    for (frame_half=0; frame_half<2; frame_half++) 
	{
		switch (orientation_state->fusion_type)
		{
		case OrientationFusion_None:
			break;
		case OrientationFusion_MadgwickIMU:
			{
				// Get the sensor data transformed by the sensor_transform
                //PSMove_3AxisVector m=
                //	psmove_orientation_get_magnetometer_normalized_vector(orientation_state);
				PSMove_3AxisVector a= 
					psmove_orientation_get_accelerometer_normalized_vector(orientation_state, (enum PSMove_Frame)(frame_half));
				PSMove_3AxisVector omega= 
					psmove_orientation_get_gyroscope_vector(orientation_state, (enum PSMove_Frame)(frame_half));

				// Apply the filter
				_psmove_orientation_fusion_imu_update(
					orientation_state,
					deltaT,
					/* Gyroscope */
					Eigen::Vector3f(omega.x, omega.y, omega.z),
					/* Accelerometer */
					Eigen::Vector3f(a.x, a.y, a.z));
			}
			break;
		case OrientationFusion_MadgwickMARG:
            {
                PSMove_3AxisVector m = psmove_orientation_get_magnetometer_normalized_vector(orientation_state);
                PSMove_3AxisVector a = psmove_orientation_get_accelerometer_normalized_vector(orientation_state, (enum PSMove_Frame)(frame_half));
                PSMove_3AxisVector omega = psmove_orientation_get_gyroscope_vector(orientation_state, (enum PSMove_Frame)(frame_half));
                _psmove_orientation_fusion_madgwick_marg_update(orientation_state, deltaT,
                                                                /* Gyroscope */
                                                                Eigen::Vector3f(omega.x, omega.y, omega.z),
                                                                /* Accelerometer */
                                                                Eigen::Vector3f(a.x, a.y, a.z),
                                                                /* Magnetometer */
                                                                Eigen::Vector3f(m.x, m.y, m.z));
            }
			break;
		case OrientationFusion_ComplementaryMARG:
			{
				PSMove_3AxisVector m= 
					psmove_orientation_get_magnetometer_normalized_vector(orientation_state);
				PSMove_3AxisVector a= 
					psmove_orientation_get_accelerometer_normalized_vector(orientation_state, (enum PSMove_Frame)(frame_half));
				PSMove_3AxisVector omega= 
					psmove_orientation_get_gyroscope_vector(orientation_state, (enum PSMove_Frame)(frame_half));

				// Apply the filter
				_psmove_orientation_fusion_complementary_marg_update(
					orientation_state,
					deltaT,
					/* Gyroscope */
					Eigen::Vector3f(omega.x, omega.y, omega.z),
					/* Accelerometer */
					Eigen::Vector3f(a.x, a.y, a.z),
					/* Magnetometer */
					Eigen::Vector3f(m.x, m.y, m.z));
			}
			break;
		}

		if (!psmove_quaternion_is_valid(orientation_state->quaternion)) 
		{
            psmove_DEBUG("Orientation is NaN!");
			orientation_state->quaternion = quaternion_backup;
        }
    }
}

void
psmove_orientation_get_quaternion(PSMoveOrientation *orientation_state,
        float *q0, float *q1, float *q2, float *q3)
{
    psmove_return_if_fail(orientation_state != NULL);

	Eigen::Quaternionf result= orientation_state->reset_quaternion * orientation_state->quaternion;

    if (q0) {
		*q0 = result.w();
    }

    if (q1) {
		*q1 = result.x();
    }

    if (q2) {
		*q2 = result.y();
    }

    if (q3) {
		*q3 = result.z();
    }
}

void
psmove_orientation_reset_quaternion(PSMoveOrientation *orientation_state)
{
    psmove_return_if_fail(orientation_state != NULL);

	Eigen::Quaternionf q_inverse = orientation_state->quaternion.conjugate();

	psmove_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
	orientation_state->reset_quaternion = q_inverse;
}

void
psmove_orientation_free(PSMoveOrientation *orientation_state)
{
    psmove_return_if_fail(orientation_state != NULL);

    free(orientation_state);
}

// -- Orientation Filters ----

// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
static void _psmove_orientation_fusion_imu_update(
	PSMoveOrientation *orientation_state,
	float delta_t,
	const Eigen::Vector3f &current_omega,
	const Eigen::Vector3f &current_g)
{
	// Current orientation from earth frame to sensor frame
	Eigen::Quaternionf SEq = orientation_state->quaternion;
	Eigen::Quaternionf SEq_new = SEq;

	// Compute the quaternion derivative measured by gyroscopes
	// Eqn 12) q_dot = 0.5*q*omega
	Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
	Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) *omega;

	if (current_g.isApprox(Eigen::Vector3f::Zero(), k_normal_epsilon))
	{
		// Get the direction of the gravitational fields in the identity pose		
		PSMove_3AxisVector identity_g= psmove_orientation_get_gravity_calibration_direction(orientation_state);
		Eigen::Vector3f k_identity_g_direction = Eigen::Vector3f(identity_g.x, identity_g.y, identity_g.z);

		// Eqn 15) Applied to the gravity vector
		// Fill in the 3x1 objective function matrix f(SEq, Sa) =|f_g|
		Eigen::Matrix<float, 3, 1> f_g;
		psmove_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

		// Eqn 21) Applied to the gravity vector
		// Fill in the 4x3 objective function Jacobian matrix: J_gb(SEq)= [J_g]
		Eigen::Matrix<float, 4, 3> J_g;
		psmove_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

		// Eqn 34) gradient_F= J_g(SEq)*f(SEq, Sa)
		// Compute the gradient of the objective function
		Eigen::Matrix<float, 4, 1> gradient_f = J_g * f_g;
		Eigen::Quaternionf SEqHatDot =
			Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

		// normalize the gradient
		psmove_quaternion_normalize_with_default(SEqHatDot, *k_psmove_quaternion_zero);

		// Compute the estimated quaternion rate of change
		// Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
		Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*beta);

		// Compute then integrate the estimated quaternion rate
		// Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
		SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*delta_t);
	}
	else
	{
		SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_omega.coeffs()*delta_t);
	}

	// Make sure the net quaternion is a pure rotation quaternion
	SEq_new.normalize();

	// Save the new quaternion back into the orientation state
	orientation_state->quaternion = SEq_new;
}

// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
static void 
_psmove_orientation_fusion_madgwick_marg_update(
	PSMoveOrientation *orientation_state,
	float delta_t,
	const Eigen::Vector3f &current_omega,
	const Eigen::Vector3f &current_g,
	const Eigen::Vector3f &current_m)
{
	// If there isn't a valid magnetometer or accelerometer vector, fall back to the IMU style update
	if (current_g.isZero(k_normal_epsilon) || current_m.isZero(k_normal_epsilon))
	{
		_psmove_orientation_fusion_imu_update(
			orientation_state,
			delta_t,
			current_omega,
			current_g);
		return;
	}

	PSMoveMadgwickMARGState *marg_state = &orientation_state->fusion_state.madgwick_marg_state;

	// Current orientation from earth frame to sensor frame
	Eigen::Quaternionf SEq = orientation_state->quaternion;

	// Get the direction of the magnetic fields in the identity pose.	
	// NOTE: In the original paper we converge on this vector over time automatically (See Eqn 45 & 46)
	// but since we've already done the work in calibration to get this vector, let's just use it.
	// This also removes the last assumption in this function about what 
	// the orientation of the identity-pose is (handled by the sensor transform).
	PSMove_3AxisVector identity_m= psmove_orientation_get_magnetometer_calibration_direction(orientation_state);
	Eigen::Vector3f k_identity_m_direction = Eigen::Vector3f(identity_m.x, identity_m.y, identity_m.z);

	// Get the direction of the gravitational fields in the identity pose
	PSMove_3AxisVector identity_g= psmove_orientation_get_gravity_calibration_direction(orientation_state);
	Eigen::Vector3f k_identity_g_direction = Eigen::Vector3f(identity_g.x, identity_g.y, identity_g.z);

	// Eqn 15) Applied to the gravity and magnetometer vectors
	// Fill in the 6x1 objective function matrix f(SEq, Sa, Eb, Sm) =|f_g|
	//                                                               |f_b|
	Eigen::Matrix<float, 3, 1> f_g;
	psmove_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

	Eigen::Matrix<float, 3, 1> f_m;
	psmove_alignment_compute_objective_vector(SEq, k_identity_m_direction, current_m, f_m, NULL);

	Eigen::Matrix<float, 6, 1> f_gb;
	f_gb.block<3, 1>(0, 0) = f_g;
	f_gb.block<3, 1>(3, 0) = f_m;

	// Eqn 21) Applied to the gravity and magnetometer vectors
	// Fill in the 4x6 objective function Jacobian matrix: J_gb(SEq, Eb)= [J_g|J_b]
	Eigen::Matrix<float, 4, 3> J_g;
	psmove_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

	Eigen::Matrix<float, 4, 3> J_m;
	psmove_alignment_compute_objective_jacobian(SEq, k_identity_m_direction, J_m);

	Eigen::Matrix<float, 4, 6> J_gb;
	J_gb.block<4, 3>(0, 0) = J_g; J_gb.block<4, 3>(0, 3) = J_m;

	// Eqn 34) gradient_F= J_gb(SEq, Eb)*f(SEq, Sa, Eb, Sm)
	// Compute the gradient of the objective function
	Eigen::Matrix<float, 4, 1> gradient_f = J_gb*f_gb;
	Eigen::Quaternionf SEqHatDot =
		Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

	// normalize the gradient to estimate direction of the gyroscope error
	psmove_quaternion_normalize_with_default(SEqHatDot, *k_psmove_quaternion_zero);

	// Eqn 47) omega_err= 2*SEq*SEqHatDot
	// compute angular estimated direction of the gyroscope error
	Eigen::Quaternionf omega_err = Eigen::Quaternionf(SEq.coeffs()*2.f) * SEqHatDot;

	// Eqn 48) net_omega_bias+= zeta*omega_err
	// Compute the net accumulated gyroscope bias
	marg_state->omega_bias = Eigen::Quaternionf(marg_state->omega_bias.coeffs() + omega_err.coeffs()*zeta*delta_t);
	marg_state->omega_bias.w() = 0.f; // no bias should accumulate on the w-component

	// Eqn 49) omega_corrected = omega - net_omega_bias
	Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
	Eigen::Quaternionf corrected_omega = Eigen::Quaternionf(omega.coeffs() - marg_state->omega_bias.coeffs());

	// Compute the rate of change of the orientation purely from the gyroscope
	// Eqn 12) q_dot = 0.5*q*omega
	Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) * corrected_omega;

	// Compute the estimated quaternion rate of change
	// Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
	Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*beta);

	// Compute then integrate the estimated quaternion rate
	// Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
	Eigen::Quaternionf SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*delta_t);

	// Make sure the net quaternion is a pure rotation quaternion
	SEq_new.normalize();

	// Save the new quaternion back into the orientation state
	orientation_state->quaternion = SEq_new;
}

static void
_psmove_orientation_fusion_complementary_marg_update(
	PSMoveOrientation *orientation_state,
	float delta_t,
	const Eigen::Vector3f &current_omega,
	const Eigen::Vector3f &current_g,
	const Eigen::Vector3f &current_m)
{
    // TODO: Following variable is unused
	PSMoveMadgwickMARGState *marg_state = &orientation_state->fusion_state.madgwick_marg_state;

	// Get the direction of the magnetic fields in the identity pose	
	PSMove_3AxisVector identity_m= psmove_orientation_get_magnetometer_calibration_direction(orientation_state);
	Eigen::Vector3f k_identity_m_direction = Eigen::Vector3f(identity_m.x, identity_m.y, identity_m.z);

	// Get the direction of the gravitational field in the identity pose
	PSMove_3AxisVector identity_g= psmove_orientation_get_gravity_calibration_direction(orientation_state);
	Eigen::Vector3f k_identity_g_direction = Eigen::Vector3f(identity_g.x, identity_g.y, identity_g.z);

	// Angular Rotation (AR) Update
	//-----------------------------
	// Compute the rate of change of the orientation purely from the gyroscope
	// q_dot = 0.5*q*omega
	Eigen::Quaternionf q_omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
	Eigen::Quaternionf q_derivative = Eigen::Quaternionf(orientation_state->quaternion.coeffs()*0.5f) * q_omega;

	// Integrate the rate of change to get a new orientation
	// q_new= q + q_dot*dT
	Eigen::Quaternionf q_step = Eigen::Quaternionf(q_derivative.coeffs() * delta_t);
	Eigen::Quaternionf ar_orientation = Eigen::Quaternionf(orientation_state->quaternion.coeffs() + q_step.coeffs());

	// Make sure the resulting quaternion is normalized
	ar_orientation.normalize();

	// Magnetic/Gravity (MG) Update
	//-----------------------------
	const Eigen::Vector3f* mg_from[2] = { &k_identity_g_direction, &k_identity_m_direction };
	const Eigen::Vector3f* mg_to[2] = { &current_g, &current_m };
	Eigen::Quaternionf mg_orientation;
	bool mg_align_success =
		psmove_alignment_quaternion_between_vector_frames(
			mg_from, mg_to, 0.1f, orientation_state->quaternion, mg_orientation);

	// Blending Update
	//----------------
	if (mg_align_success)
	{
		// The final rotation is a blend between the integrated orientation and absolute rotation from the earth-frame
		float mg_wight = orientation_state->fusion_state.complementary_marg_state.mg_weight;
		orientation_state->quaternion =
			psmove_quaternion_normalized_lerp(ar_orientation, mg_orientation, mg_wight);

		// Update the blend weight
		orientation_state->fusion_state.complementary_marg_state.mg_weight =
			lerp_clampf(mg_wight, k_base_earth_frame_align_weight, 0.9f);
	}
	else
	{
		orientation_state->quaternion = ar_orientation;
	}
}
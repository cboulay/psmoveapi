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

//-- includes ----
#include <assert.h>

#include "psmove_kalman_filter.h"
#include "psmove_tracker.h"
#include "../psmove_private.h"
#include "psmove_advanced_math.hpp"

//-- structures ----
struct _PSMovePositionKalmanFilter {
	// 'x_(k-1|k-1)' - State vector for the previous frame
	// In this case a 6x1 vector containing position and velocity
	// x/y/z coordinates of sphere in cm from camera focal point
	// vx/vy/vz coordinates of sphere in cm/s
	Eigen::Matrix<float, 6, 1> state_vector;

	// 'P_(k-1|k-1)' - The covariance matrix of the state 'x_(k-1)'
	// Captures the uncertainty of the state vector for the state vector
	// In this case a 6x6 matrix
	Eigen::Matrix<float, 6, 6> state_covariance_matrix;

	// The variance in the acceleration noise
	// Used to compute the Process Noise Matrix 'Q'
	float acceleration_variance;

	// 'R' - The covariance matrix of the measurement vector 'z'
	// The measurement noise 'R', which is the inaccuracy introduced by the sensors 
	// It models the difference between sensor's measurement value and real position..
	// In this case 'R' is a 3x3 matrix computed at calibration time
	Eigen::Matrix<float, 3, 3> measurement_noise_matrix;

    // The last time we ran the filter update
    PSMove_timestamp last_filter_update;

	bool is_initialized;
};

//-- prototypes -----
static void MakeStateTransitionMatrix(const float dT, Eigen::Matrix<float, 6, 6> &m);
static void MakeControlModelMatrix(const float dT, Eigen::Matrix<float, 6, 3> &m);
static void MakeProcessNoiseMatrix(const float varA, const float dT, Eigen::Matrix<float, 6, 6> &m);
static void MakeControlInputVector(const PSMove_3AxisVector *v, Eigen::Matrix<float, 3, 1> &m);
static void ComputeErrorVector(const Eigen::Matrix<float, 6, 1> &predict_state_vector, const PSMove_3AxisVector *measured_position, Eigen::Matrix<float, 3, 1> &error_vector);
static void ComputHPHtMatrix(const Eigen::Matrix<float, 6, 6> &P, Eigen::Matrix<float, 3, 3> &HPH_t);
static void ComputPHtMatrix(const Eigen::Matrix<float, 6, 6> &P, Eigen::Matrix<float, 6, 3> &PH_t);
static void ComputIMinusKHMatrix(const Eigen::Matrix<float, 6, 3> &K, Eigen::Matrix<float, 6, 6> &IMinusKH);

//-- public methods -----
PSMovePositionKalmanFilter *
psmove_position_kalman_filter_new()
{
	PSMovePositionKalmanFilter *filter_state = (PSMovePositionKalmanFilter *)calloc(1, sizeof(PSMovePositionKalmanFilter));

	filter_state->is_initialized = false;
	filter_state->acceleration_variance = 0.f;
	filter_state->measurement_noise_matrix.setZero();
	filter_state->state_covariance_matrix.setZero();
	filter_state->state_vector.setZero();

	return filter_state;
}

void
psmove_position_kalman_filter_free(PSMovePositionKalmanFilter *filter_state)
{
    psmove_return_if_fail(filter_state != NULL);
	free(filter_state);
}

void
psmove_position_kalman_filter_init(
	const PSMoveTrackerSmoothingSettings *tracker_settings, const PSMove_3AxisVector *position,
	PSMovePositionKalmanFilter *filter_state)
{
	const float k_large_variance = 10.f;

	// Set the calibrated noise parameters
	filter_state->acceleration_variance = tracker_settings->acceleration_variance;
	// Copy the raw C-style matrix into the 3x3 Eigen matrix
	filter_state->measurement_noise_matrix = 
		Eigen::Map< Eigen::Matrix<float, 3, 3> >((float *)&tracker_settings->measurement_covariance.m);

	// Initialize the state covariance matrix to have large initial uncertainty about position/velocity
	filter_state->state_covariance_matrix.setZero();
	filter_state->state_covariance_matrix(0,0) = k_large_variance;
	filter_state->state_covariance_matrix(1,1) = k_large_variance;
	filter_state->state_covariance_matrix(2,2) = k_large_variance;
	filter_state->state_covariance_matrix(3,3) = k_large_variance;
	filter_state->state_covariance_matrix(4,4) = k_large_variance;
	filter_state->state_covariance_matrix(5,5) = k_large_variance;

	// Initialize the state vector with the given starting location and assume zero velocity
	filter_state->state_vector(0,0) = position->x;
	filter_state->state_vector(1,0) = position->y;
	filter_state->state_vector(2,0) = position->z;
	filter_state->state_vector(3,0) = 0.f;
	filter_state->state_vector(4,0) = 0.f;
	filter_state->state_vector(5,0) = 0.f;

    // Record now as the last time we updated the filter
    filter_state->last_filter_update= psmove_timestamp();

	filter_state->is_initialized = true;
}

PSMove_3AxisVector
psmove_position_kalman_filter_get_position(PSMovePositionKalmanFilter *filter_state)
{
	return 
		psmove_3axisvector_xyz(
			filter_state->state_vector(0, 0), 
			filter_state->state_vector(1, 0), 
			filter_state->state_vector(2, 0));
}

PSMove_3AxisVector
psmove_position_kalman_filter_get_velocity(PSMovePositionKalmanFilter *filter_state)
{
	return 
		psmove_3axisvector_xyz(
			filter_state->state_vector(3, 0), 
			filter_state->state_vector(4, 0), 
			filter_state->state_vector(5, 0));
}

PSMove_3AxisVector
psmove_position_kalman_filter_get_position_variance(PSMovePositionKalmanFilter *filter_state)
{
	return 
		psmove_3axisvector_xyz(
			filter_state->state_covariance_matrix(0, 0), 
			filter_state->state_covariance_matrix(1, 1), 
			filter_state->state_covariance_matrix(2, 2));
}

PSMove_3AxisVector
psmove_position_kalman_filter_get_velocity_variance(PSMovePositionKalmanFilter *filter_state)
{
	return 
		psmove_3axisvector_xyz(
			filter_state->state_covariance_matrix(3, 3), 
			filter_state->state_covariance_matrix(4, 4), 
			filter_state->state_covariance_matrix(5, 5));
}

void 
psmove_position_kalman_filter_update(
	const PSMoveTrackerSmoothingSettings *tracker_settings,
	const PSMove_3AxisVector *measured_position,	// The position measured by the sensors.
	const PSMove_3AxisVector *acceleration_control,	// The world space acceleration measured on the controller.
	PSMovePositionKalmanFilter *filter_state)
{
	// Initialize the filter if it hasn't been initialized already
	if (!filter_state->is_initialized)
	{
		psmove_position_kalman_filter_init(tracker_settings, measured_position, filter_state);
		return;
	}

    // Record now as the last time we updated the filter
    PSMove_timestamp now = psmove_timestamp();

    // Compute how long it has been since our last successful position update
    double time_delta = psmove_timestamp_value(psmove_timestamp_diff(now, filter_state->last_filter_update));

    // Remember now as the last time we updated
    filter_state->last_filter_update= now;

	// 'x_(k|k-1)' - Predicted state given the previous state
	// In this case a 6x1 vector containing position and velocity
	Eigen::Matrix<float, 6, 1> predict_state_vector;

	// 'P_(k|k-1)' - Predicted covariance matrix of the state 'x_(k|k-1)'
	// Captures the uncertainty of the state vector for the predicted state vector
	// In this case a 6x6 matrix
	Eigen::Matrix<float, 6, 6> predict_state_covariance_matrix;

	// -- Prediction Phase --
	{
		// 'F' - State transition matrix. Model of how the physics evolves for the current state.
		// In this case a 6x6 matrix 
		Eigen::Matrix<float, 6, 6> state_transition_matrix;

		// 'B' - Control model matrix. Model of how control inputs maps the input vector to the state.
		// In this case a 6x3 matrix 
		Eigen::Matrix<float, 6, 3> control_model_matrix;

		// 'u' - Control input vector. The acceleration control applied to the system.
		// In this case a 3x1 matrix
		Eigen::Matrix<float, 3, 1> control_input_vector;

		// 'Q' - Process noise covariance matrix.
		// The process noise sigma_p, which estimates the error in our system model. 
		// It is dependent on the model, i.e. how exactly it estimates future 
		// values from the current state of the Kalman filter.
		// In this case a 6x6 matrix: Q = B*B_transpose*acceleration_variance
		Eigen::Matrix<float, 6, 6> process_noise_matrix;

		// Build the matrices needed for prediction
		MakeStateTransitionMatrix(time_delta, state_transition_matrix); // 6x6 Matrix F
		MakeControlModelMatrix(time_delta, control_model_matrix); // 6x3 Matrix B
		MakeControlInputVector(acceleration_control, control_input_vector); // 3x1 Vector u
		MakeProcessNoiseMatrix(filter_state->acceleration_variance, time_delta, process_noise_matrix); // 6x6 Matrix Q

		// x_(k|k-1) = F*x_(k-1|k-1) + B*u
		predict_state_vector= 
			state_transition_matrix*filter_state->state_vector 
			+ control_model_matrix*control_input_vector;

		// P_(k|k-1) = F*P_(k-1|k-1)*transpose(F) + Q
		predict_state_covariance_matrix = 
			state_transition_matrix*filter_state->state_covariance_matrix*state_transition_matrix.transpose()
			+ process_noise_matrix;
	}

	// -- Correction Phase--
	{
		// 'x_(k|k)' - Corrected state given the predicted state
		// In this case a 6x1 vector containing position and velocity
		Eigen::Matrix<float, 6, 1> corrected_state_vector;

		// 'P_(k|k)' - The covariance matrix of the state 'x_(k)'
		// Captures the uncertainty of the corrected state vector
		Eigen::Matrix<float, 6, 6> corrected_state_covariance_matrix;

		// 'y' - The error between the measurement and what we expected
		Eigen::Matrix<float, 3, 1> error_vector;

		// 'S^-1' - The inverse of covariance of the move
		Eigen::Matrix<float, 3, 3> move_covariance_inverse_matrix;

		// 'K' - The Kalman gain, which is used to compute the corrected 'x_(k)' and 'P_(k)'
		Eigen::Matrix<float, 6, 3> kalman_gain;

		// y = z - H*x_(k|k-1)
		// In this case 'y' is a 3x1 vector where:
		//  z = the sensor measurement vector of the system (in this case the measure position)
		//  H = the observation matrix that maps the the state vector into sensor vector space
		ComputeErrorVector(predict_state_vector, measured_position, error_vector);

		// S^-1 = (H * P_(k|k-1) * transpose(H) + R)^-1
		// In this case S is a 3x3 matrix where:
		//  H = the observation matrix that maps the the state vector into sensor vector space
		//  P_(k|k-1) = The predicted state covariance matrix computed in the update phase
		//  R = The covariance matrix of the measurement vector 'z'
		{
			Eigen::Matrix<float, 3, 3> HPH_t;

			ComputHPHtMatrix(predict_state_covariance_matrix, HPH_t);
			move_covariance_inverse_matrix= (HPH_t + filter_state->measurement_noise_matrix).inverse();
		}

		// K = P_(k|k-1)*transpose(H)*S^-1
		{
			Eigen::Matrix<float, 6, 3> PH_t;

			ComputPHtMatrix(predict_state_covariance_matrix, PH_t);
			kalman_gain= PH_t * move_covariance_inverse_matrix;
		}

		// x_(k|k) = x_(k|k-1) + K*y
		corrected_state_vector= predict_state_vector + kalman_gain*error_vector;

		// P_(k|k) = (I - K*H)*P_(k|k-1)
		{
			Eigen::Matrix<float, 6, 6> IMinusKH;

			ComputIMinusKHMatrix(kalman_gain, IMinusKH);
			corrected_state_covariance_matrix = IMinusKH*predict_state_covariance_matrix;
		}

		// x_(k-1|k-1)= x_(k|k)
		filter_state->state_vector= corrected_state_vector;

		// P_(k-1|k-1) = P_(k|k)
		filter_state->state_covariance_matrix= corrected_state_covariance_matrix;
	}
}

//-- private methods ----
static void MakeStateTransitionMatrix(
	const float dT,
	Eigen::Matrix<float, 6, 6> &m)
{
	m.setIdentity();
	m(0,3) = dT;
	m(1,4) = dT;
	m(2,5) = dT;
}

static void MakeControlModelMatrix(
	const float dT,
	Eigen::Matrix<float, 6, 3> &m)
{
	const float one_half_delta_t_sq = 0.5f*dT*dT;

	// Control Model B used to apply acceleration input onto state
	m(0,0) = one_half_delta_t_sq; m(0,1) = 0.f;                 m(0,2) = 0.f;
	m(1,0) = 0.f;                 m(1,1) = one_half_delta_t_sq; m(1,2) = 0.f;
	m(2,0) = 0.f;                 m(2,1) = 0.f;                 m(2,2) = one_half_delta_t_sq;
	m(3,0) = dT;                  m(3,1) = 0.f;                 m(3,2) = 0.f;
	m(4,0) = 0.f;                 m(4,1) = dT;                  m(4,2) = 0.f;
	m(5,0) = 0.f;                 m(5,1) = 0.f;                 m(5,2) = dT;
}

static void MakeProcessNoiseMatrix(
	const float varA, // variance of acceleration due to noise
	const float dT,
	Eigen::Matrix<float, 6, 6> &m)
{
	const float dT_squared = dT*dT;
	const float q2 = varA * dT_squared;
	const float q3 = varA * 0.5f*dT_squared*dT;
	const float q4 = varA * 0.25f*dT_squared*dT_squared;

	// Q = B * Transpose(B) * var(acceleration)
	m(0,0) =  q4; m(0,1) = 0.f; m(0,2) = 0.f;   m(0,3) =  q3; m(0,4) = 0.f; m(0,5) = 0.f;
	m(1,0) = 0.f; m(1,1) =  q4; m(1,2) = 0.f;   m(1,3) = 0.f; m(1,4) =  q3; m(1,5) = 0.f;
	m(2,0) = 0.f; m(2,1) = 0.f; m(2,2) =  q4;   m(2,3) = 0.f; m(2,4) = 0.f; m(2,5) =  q3;

	m(3,0) =  q3; m(3,1) = 0.f; m(3,2) = 0.f;   m(3,3) =  q2; m(3,4) = 0.f; m(3,5) = 0.f;
	m(4,0) = 0.f; m(4,1) =  q3; m(4,2) = 0.f;   m(4,3) = 0.f; m(4,4) =  q2; m(4,5) = 0.f;
	m(5,0) = 0.f; m(5,1) = 0.f; m(5,2) =  q3;   m(5,3) = 0.f; m(5,4) = 0.f; m(5,5) =  q2;
}

static void MakeControlInputVector(
	const PSMove_3AxisVector *v,
	Eigen::Matrix<float, 3, 1> &m)
{
	m(0,0) = v->x;
	m(1,0) = v->y;
	m(2,0) = v->z;
}

static void ComputeErrorVector(
	const Eigen::Matrix<float, 6, 1> &PredictStateVector,
	const PSMove_3AxisVector *MeasuredPosition,
	Eigen::Matrix<float, 3, 1> &ErrorVector)
{
	ErrorVector(0,0) = MeasuredPosition->x - PredictStateVector(0,0);
	ErrorVector(1,0) = MeasuredPosition->y - PredictStateVector(1,0);
	ErrorVector(2,0) = MeasuredPosition->z - PredictStateVector(2,0);
}

static void ComputHPHtMatrix(
	const Eigen::Matrix<float, 6, 6> &P,
	Eigen::Matrix<float, 3, 3> &HPH_t)
{
	// Compute H * P_(k|k-1) * transpose(H)
	// Where H (a.k.a the Extraction matrix) equals:
	//
	// |1 0 0 0 0 0|
	// |0 1 0 0 0 0|
	// |0 0 1 0 0 0|
	//
	// This has the effect of extracting the upper-left 3x3 portion of P
	HPH_t(0,0) = P(0,0); HPH_t(0,1) = P(0,1); HPH_t(0,2) = P(0,2);
	HPH_t(1,0) = P(1,0); HPH_t(1,1) = P(1,1); HPH_t(1,2) = P(1,2);
	HPH_t(2,0) = P(2,0); HPH_t(2,1) = P(2,1); HPH_t(2,2) = P(2,2);
}

static void ComputPHtMatrix(
	const Eigen::Matrix<float, 6, 6> &P,
	 Eigen::Matrix<float, 6, 3> &PH_t)
{
	// Compute P_(k|k-1) * transpose(H)
	// Where transpose(H) (a.k.a the transpose of the extraction matrix) equals:
	//
	// |1 0 0|
	// |0 1 0|
	// |0 0 1|
	// |0 0 0|
	// |0 0 0|
	// |0 0 0|
	//
	// This has the effect of extracting the left 6x3 portion of P
	PH_t(0,0) = P(0,0); PH_t(0,1) = P(0,1); PH_t(0,2) = P(0,2);
	PH_t(1,0) = P(1,0); PH_t(1,1) = P(1,1); PH_t(1,2) = P(1,2);
	PH_t(2,0) = P(2,0); PH_t(2,1) = P(2,1); PH_t(2,2) = P(2,2);
	PH_t(3,0) = P(3,0); PH_t(3,1) = P(3,1); PH_t(3,2) = P(3,2);
	PH_t(4,0) = P(4,0); PH_t(4,1) = P(4,1); PH_t(4,2) = P(4,2);
	PH_t(5,0) = P(5,0); PH_t(5,1) = P(5,1); PH_t(5,2) = P(5,2);
}

static void ComputIMinusKHMatrix(
	const Eigen::Matrix<float, 6, 3> &K,
	Eigen::Matrix<float, 6, 6> &IMinusKH)
{
	// Compute I - K*H:
	// Where I is a 6x6 identity matrix
	// K (the the 6x3 Kalman gain) equals:
	// |k00 k01 k02|
	// |k10 k11 k12|
	// |k20 k21 k22|
	// |k30 k31 k32|
	// |k40 k41 k42|
	// |k50 k51 k52|
	// And H (a.k.a the Extraction matrix) equals:
	// |1 0 0 0 0 0|
	// |0 1 0 0 0 0|
	// |0 0 1 0 0 0|

	// This has the effect of extracting the left 6x3 portion of P
	// from which the identity matrix subtracts the upper-left 3x3 portion.
	IMinusKH(0,0) = 1.f-K(0,0); IMinusKH(0,1) =    -K(0,1); IMinusKH(0,2) =    -K(0,2); IMinusKH(0,3) = 0.f; IMinusKH(0,4) = 0.f; IMinusKH(0,5) = 0.f;
	IMinusKH(1,0) =    -K(1,0); IMinusKH(1,1) = 1.f-K(1,1); IMinusKH(1,2) =    -K(1,2); IMinusKH(1,3) = 0.f; IMinusKH(1,4) = 0.f; IMinusKH(1,5) = 0.f;
	IMinusKH(2,0) =    -K(2,0); IMinusKH(2,1) =    -K(2,1); IMinusKH(2,2) = 1.f-K(2,2); IMinusKH(2,3) = 0.f; IMinusKH(2,4) = 0.f; IMinusKH(2,5) = 0.f;
	IMinusKH(3,0) =    -K(3,0); IMinusKH(3,1) =    -K(3,1); IMinusKH(3,2) =    -K(3,2); IMinusKH(3,3) = 1.f; IMinusKH(3,4) = 0.f; IMinusKH(3,5) = 0.f;
	IMinusKH(4,0) =    -K(4,0); IMinusKH(4,1) =    -K(4,1); IMinusKH(4,2) =    -K(4,2); IMinusKH(4,3) = 0.f; IMinusKH(4,4) = 1.f; IMinusKH(4,5) = 0.f;
	IMinusKH(5,0) =    -K(5,0); IMinusKH(5,1) =    -K(5,1); IMinusKH(5,2) =    -K(5,2); IMinusKH(5,3) = 0.f; IMinusKH(5,4) = 0.f; IMinusKH(5,5) = 1.f;
}

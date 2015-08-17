
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


#ifndef PSMOVE_ORIENTATION_H
#define PSMOVE_ORIENTATION_H

#ifdef __cplusplus
extern "C" {
#endif

//-- includes -----
#include "psmove.h"
#include "math/psmove_vector.h"

//-- pre-declarations -----
struct _PSMoveOrientation;
typedef struct _PSMoveOrientation PSMoveOrientation;

//-- constants ----
enum PSMoveOrientation_Fusion_Type {
	OrientationFusion_None,
	OrientationFusion_MadgwickIMU,
	OrientationFusion_MadgwickMARG,
	OrientationFusion_ComplementaryMARG,
};

ADDAPI extern const PSMove_3AxisTransform *k_psmove_identity_pose_upright;
ADDAPI extern const PSMove_3AxisTransform *k_psmove_identity_pose_laying_flat;

ADDAPI extern const PSMove_3AxisTransform *k_psmove_sensor_transform_identity;
ADDAPI extern const PSMove_3AxisTransform *k_psmove_sensor_transform_opengl;

//-- interface -----
ADDAPI PSMoveOrientation *
ADDCALL psmove_orientation_new(PSMove *move);

ADDAPI void
ADDCALL psmove_orientation_free(PSMoveOrientation *orientation_state);

ADDAPI void
ADDCALL psmove_orientation_set_fusion_type(PSMoveOrientation *orientation_state, enum PSMoveOrientation_Fusion_Type fusion_type);

/**
 * \brief Set the transform used on the calibration data in the psmove_get_transform_<sensor>_... methods
 *
 * This method sets the transform used to modify the calibration vectors returned by:
 * - psmove_orientation_get_magnetometer_calibration_direction()
 * - psmove_orientation_get_gravity_calibration_direction()
 *
 * The transformed calibration data is used by the orientation filter to compute 
 * a quaternion (see \ref psmove_orientation_get_quaternion) that represents 
 * the controllers current rotation from the "identity pose".
 * 
 * Historically, the "identity pose" bas been with the controller laying flat
 * with the controller pointing at the screen. However, now that we have a
 * calibration step that record the magnetic field direction relative to 
 * gravity it makes more sense to make the identity pose with the controller 
 * sitting vertically since it's more stable to record that way. 
 *  
 * In order to maintain reverse compatibility, this transform defaults to rotating
 * the vertically recorded calibration vectors 90 degrees about X axis as if the 
 * controller was laying flat during calibration.
 *
 * Therefore, if you want a different "identity pose" then the default,
 * use this method to set a custom transform.
 *
 * There are the following transforms available:
 * - k_psmove_identity_pose_upright - "identity pose" is the controller standing upright
 * - k_psmove_identity_pose_laying_flat - "identity pose" is the controller laying down pointed at the screen
 *
 * \param orientation_state A valid \ref PSMoveOrientation handle
 * \param transform A \ref PSMove_3AxisTransform transform to apply to the calibration data
 **/
ADDAPI void
ADDCALL psmove_orientation_set_calibration_transform(PSMoveOrientation *orientation_state, const PSMove_3AxisTransform *transform);

ADDAPI PSMove_3AxisVector
ADDCALL psmove_orientation_get_gravity_calibration_direction(PSMoveOrientation *orientation_state);

ADDAPI PSMove_3AxisVector
ADDCALL psmove_orientation_get_magnetometer_calibration_direction(PSMoveOrientation *orientation_state);

/**
 * \brief Set the transform used on the sensor data in the psmove_get_transform_<sensor>_... methods
 *
 * This method sets the transform used to modify the sensor vectors returned by:
 * - psmove_orientation_get_accelerometer_vector()
 * - psmove_orientation_get_accelerometer_normalized_vector()
 * - psmove_orientation_get_gyroscope_vector()
 * - psmove_orientation_get_magnetometer_normalized_vector()
 *
 * The transformed sensor data is used by the orientation filter to compute 
 * a quaternion (see \ref psmove_orientation_get_quaternion) that represents 
 * the controllers current rotation from the "identity pose".
 * 
 * Historically, the sensor data in the orientation code has been rotated 90 degrees 
 * clockwise about the x-axis. The original Madgwick orientation filter was coded to assume
 * an OpenGL style coordinate system (+x=right, +y=up, +z=out of screen), rather than 
 * than PSMoves coordinate system where:
 *
 * +x = From Select to Start button
 * +y = From Trigger to Move button
 * +z = From glowing orb to USB connector
 *
 * The current default sets the sensor transform to assume an OpenGL style coordinate system
 * in order to maintain reverse compatibility
 *
 * There are the following transforms available:
 * - k_psmove_sensor_transform_identity - Keep the sensor data as it was
 * - k_psmove_sensor_transform_opengl - Rotate 90 degrees about the x-axis (historical default)
 *
 * \param orientation_state A valid \ref PSMoveOrientation handle
 * \param transform A \ref PSMove_3AxisTransform transform to apply to the sensor data
 **/
ADDAPI void
ADDCALL psmove_orientation_set_sensor_data_transform(PSMoveOrientation *orientation_state, const PSMove_3AxisTransform *transform);

ADDAPI PSMove_3AxisVector
ADDCALL psmove_orientation_get_accelerometer_vector(PSMoveOrientation *orientation_state, enum PSMove_Frame frame);

ADDAPI PSMove_3AxisVector
ADDCALL psmove_orientation_get_accelerometer_normalized_vector(PSMoveOrientation *orientation_state, enum PSMove_Frame frame);

ADDAPI PSMove_3AxisVector
ADDCALL psmove_orientation_get_gyroscope_vector(PSMoveOrientation *orientation_state, enum PSMove_Frame frame);

ADDAPI PSMove_3AxisVector
ADDCALL psmove_orientation_get_magnetometer_normalized_vector(PSMoveOrientation *orientation_state);

ADDAPI void
ADDCALL psmove_orientation_update(PSMoveOrientation *orientation_state);

ADDAPI void
ADDCALL psmove_orientation_get_quaternion(PSMoveOrientation *orientation_state,
        float *q0, float *q1, float *q2, float *q3);

ADDAPI void
ADDCALL psmove_orientation_reset_quaternion(PSMoveOrientation *orientation_state);

#ifdef __cplusplus
}
#endif

#endif

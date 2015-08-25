
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


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "psmove.h"
#include "math/psmove_vector.h"
#include "math/psmove_quaternion.hpp"
#include "math/psmove_alignment.hpp"

//#define PSMOVE_NATIVE_ORIENTATION

int
main(int argc, char* argv[])
{
    PSMove *move;

	if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
		fprintf(stderr, "PS Move API init failed (wrong version?)\n");
		exit(1);
	}

    move = psmove_connect();

    if (move == NULL) {
        fprintf(stderr, "Could not connect to controller.\n");
        return EXIT_FAILURE;
    }

    if (psmove_has_calibration(move))
	{
		if (psmove_connection_type(move) == Conn_Bluetooth) 
		{
			Eigen::Quaternionf mg_orientation = Eigen::Quaternionf::Identity();

			#ifdef PSMOVE_NATIVE_ORIENTATION
			psmove_set_calibration_transform(move, k_psmove_identity_pose_upright);
			psmove_set_sensor_data_transform(move, k_psmove_sensor_transform_identity);
			#else 
			// This is the historical default
			psmove_set_calibration_transform(move, k_psmove_identity_pose_laying_flat);
			psmove_set_sensor_data_transform(move, k_psmove_sensor_transform_opengl);
			#endif

			PSMove_3AxisVector calibration_a; 
			psmove_get_transformed_gravity_calibration_direction(move, &calibration_a);
			Eigen::Vector3f gravity_calibration_direction = Eigen::Vector3f(calibration_a.x, calibration_a.y, calibration_a.z);

			PSMove_3AxisVector calibration_m; 
			psmove_get_transformed_magnetometer_calibration_direction(move, &calibration_m);
			Eigen::Vector3f magnetometer_calibration_direction = Eigen::Vector3f(calibration_m.x, calibration_m.y, calibration_m.z);
			
			while ((psmove_get_buttons(move) & Btn_PS) == 0) 
			{
				int res = psmove_poll(move);

				if (res) 
				{
					// Get the sensor measurements in Right-Handed Cartesian Coordinates (a.k.a. OpenGL Coordinates)					
					PSMove_3AxisVector w;
					psmove_get_transformed_gyroscope_frame_3axisvector(move, Frame_SecondHalf, &w);
					Eigen::Vector3f omega = Eigen::Vector3f(w.x, w.y, w.z);

					PSMove_3AxisVector a;
					psmove_get_transformed_accelerometer_frame_3axisvector(move, Frame_SecondHalf, &a);
					Eigen::Vector3f acceleration = Eigen::Vector3f(a.x, a.y, a.z);
					Eigen::Vector3f gravity_direction = acceleration;
					gravity_direction.normalize();

					PSMove_3AxisVector m;
					psmove_get_transformed_magnetometer_direction(move, &m);
					Eigen::Vector3f magnetometer_direction = Eigen::Vector3f(m.x, m.y, m.z);

					// Attempt to compute a quaternion that would align the calibration gravity and magnetometer directions
					// with the currently read gravity and magnetometer directions
					const Eigen::Vector3f* mg_from[2] = { &gravity_calibration_direction, &magnetometer_calibration_direction };
					const Eigen::Vector3f* mg_to[2] = { &gravity_direction, &magnetometer_direction };
					Eigen::Quaternionf new_mg_orientation;
					bool mg_align_success =
						psmove_alignment_quaternion_between_vector_frames(
							mg_from, mg_to, 0.15f, mg_orientation, new_mg_orientation);

					if (mg_align_success)
					{
						float yaw = 0.f, pitch = 0.f, roll = 0.f;

						mg_orientation = new_mg_orientation;
						psmove_quaternion_get_yaw_pitch_roll(mg_orientation, &yaw, &pitch, &roll);

						yaw = radians_to_degrees(yaw);
						pitch = radians_to_degrees(pitch);
						roll = radians_to_degrees(roll);

						printf("A:<%5.2f %5.2f %5.2f> M:<%6.2f %6.2f %6.2f> W:<%6.2f %6.2f %6.2f> Q:<%6.2f %6.2f %6.2f>\n",
							acceleration.x(), acceleration.y(), acceleration.z(),
							magnetometer_direction.x(), magnetometer_direction.y(), magnetometer_direction.z(),
							omega.x(), omega.y(), omega.z(),
							pitch, yaw, roll);
					}
					else
					{
						printf("A:<%5.2f %5.2f %5.2f> M:<%6.2f %6.2f %6.2f> W:<%6.2f %6.2f %6.2f> Q:<- - ->\n",
							acceleration.x(), acceleration.y(), acceleration.z(),
							magnetometer_direction.x(), magnetometer_direction.y(), magnetometer_direction.z(),
							omega.x(), omega.y(), omega.z());
					}
				}
			}
		}
		else
		{
			fprintf(stderr, "Controller isn't connected via bluetooth!\n");
		}
	}
	else
	{
		fprintf(stderr, "Controller doesn't have valid calibration data!\n");
	}

    psmove_disconnect(move);
	psmove_shutdown();

    return EXIT_SUCCESS;
}


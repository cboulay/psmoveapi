
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
#include <stdbool.h>
#include <assert.h>
#include <math.h>

#include "psmove.h"
#include "math/psmove_vector.h"

//-- constants -----
enum eMeasurementState
{
	Measurement_WaitForGravityAlignment,
	Measurement_MeasureGyroscope,
	Measurement_Complete,
};

#define STABILIZE_WAIT_TIME_SECONDS 1.0
#define DESIRED_NOISE_SAMPLE_COUNT 1000
#define DESIRED_SAMPLING_TIME 30.0 // seconds
#define RADIANS_TO_DEGREES (180.0f / (float)M_PI)

//-- prototypes -----
static bool is_move_stable_and_aligned_with_gravity(PSMove *move);

//-- public methods ----
int
main(int argc, char* argv[])
{
    PSMove *move;

	if (!psmove_init(PSMOVE_CURRENT_VERSION)) 
	{
		fprintf(stderr, "PS Move API init failed (wrong version?)\n");
		return EXIT_FAILURE;
	}

    move = psmove_connect();
    if (move == NULL) 
	{
        fprintf(stderr, "Could not connect to controller.\n");
        return EXIT_FAILURE;
    }

	if (!psmove_has_calibration(move))
	{
		fprintf(stderr, "Controller missing sensor calibration.\n");
		return EXIT_FAILURE;
	}

    if (psmove_connection_type(move) == Conn_Bluetooth) 
	{
		float omega_x_samples[DESIRED_NOISE_SAMPLE_COUNT];
		float omega_y_samples[DESIRED_NOISE_SAMPLE_COUNT];
		float omega_z_samples[DESIRED_NOISE_SAMPLE_COUNT];

		float omega_x, omega_y, omega_z;
		double theta_x, theta_y, theta_z;
		double mean_omega_x, mean_omega_y, mean_omega_z;
		double var_omega_x, var_omega_y, var_omega_z;
		double stddev_omega_x, stddev_omega_y, stddev_omega_z;
		double elapsed_time;
		int noise_sample_count;
		int total_sample_count;
		enum eMeasurementState measurement_state = Measurement_WaitForGravityAlignment;

		while (measurement_state != Measurement_Complete)
		{
			theta_x = theta_y = theta_z = 0.f;
			mean_omega_x = mean_omega_y = mean_omega_z = 0.f;
			var_omega_x = var_omega_y =  var_omega_z = 0.f;
			stddev_omega_x = stddev_omega_y = stddev_omega_z = 0.f;
			elapsed_time = 0.f;
			noise_sample_count = 0;
			total_sample_count = 0;

			// State: Measurement_WaitForGravityAlignment
			{
				PSMove_timestamp start_time;
				double stable_duration;

				printf("\n\nStand the controller on a level surface.\n");
				printf("Measurement will start once the controller is aligned with gravity and stable.\n");
				fflush(stdout);

				start_time = psmove_timestamp();
				while (measurement_state == Measurement_WaitForGravityAlignment)
				{
					if (psmove_poll(move))
					{
						if (is_move_stable_and_aligned_with_gravity(move))
						{
							PSMove_timestamp current_time = psmove_timestamp();
							stable_duration= psmove_timestamp_value(psmove_timestamp_diff(current_time, start_time));

							if (stable_duration >= STABILIZE_WAIT_TIME_SECONDS)
							{
								measurement_state = Measurement_MeasureGyroscope;
							}
							else
							{
								printf("\rStable for: %fs/%fs                        ", stable_duration, STABILIZE_WAIT_TIME_SECONDS);
							}
						}
						else
						{
							start_time = psmove_timestamp();
							printf("\rMove Destabilized! Waiting for stabilization.");
						}
					}
				}
			}

			// State: Measurement_MeasureGyroscope
			{
				PSMove_timestamp last_update_time;

				printf("\n\nMove stabilized. Starting gyroscope sampling.\n");
				printf("Sampling gyro for %f seconds...\n", DESIRED_SAMPLING_TIME);
				fflush(stdout);

				last_update_time = psmove_timestamp();

				while (measurement_state == Measurement_MeasureGyroscope)
				{
					if (psmove_poll(move)) 
					{
						if (is_move_stable_and_aligned_with_gravity(move))
						{
							PSMove_timestamp update_time = psmove_timestamp();
							double dt= psmove_timestamp_value(psmove_timestamp_diff(update_time, last_update_time));

							psmove_get_gyroscope_frame(move, Frame_SecondHalf, &omega_x, &omega_y, &omega_z);

							omega_x *= RADIANS_TO_DEGREES;
							omega_y *= RADIANS_TO_DEGREES;
							omega_z *= RADIANS_TO_DEGREES;

							// Integrate the gyros over time to compute
							theta_x += (double)omega_x*dt;
							theta_y += (double)omega_y*dt;
							theta_z += (double)omega_z*dt;
							elapsed_time += dt;

							if (noise_sample_count < DESIRED_NOISE_SAMPLE_COUNT)
							{
								omega_x_samples[noise_sample_count] = omega_x;
								omega_y_samples[noise_sample_count] = omega_y;
								omega_z_samples[noise_sample_count] = omega_z;
								noise_sample_count++;
							}
							total_sample_count++;
							
							if (noise_sample_count >= DESIRED_NOISE_SAMPLE_COUNT && elapsed_time >= DESIRED_SAMPLING_TIME)
							{
								measurement_state= Measurement_Complete;
							}
							else
							{
								printf("\rMeasuring for: %fs/%fs                        ", elapsed_time, DESIRED_SAMPLING_TIME);
							}

							last_update_time = update_time;
						}
						else
						{
							measurement_state = Measurement_WaitForGravityAlignment;
							printf("\rMove Destabilized! Waiting for stabilization.");
						}
					}
				}
			}
		}

		int i;
		double N = (double)total_sample_count;

		// Compute the mean of the samples
		for (i = 0; i < DESIRED_NOISE_SAMPLE_COUNT; i++)
		{
			mean_omega_x += (double)omega_x_samples[i];
			mean_omega_y += (double)omega_y_samples[i];
			mean_omega_z += (double)omega_z_samples[i];
		}
		mean_omega_x /= N;
		mean_omega_y /= N;
		mean_omega_z /= N;

		// Compute the standard deviation of the samples
		for (i = 0; i < DESIRED_NOISE_SAMPLE_COUNT; i++)
		{
			var_omega_x += ((double)omega_x_samples[i] - mean_omega_x)*((double)omega_x_samples[i] - mean_omega_x);
			var_omega_y += ((double)omega_y_samples[i] - mean_omega_y)*((double)omega_y_samples[i] - mean_omega_y);
			var_omega_z += ((double)omega_z_samples[i] - mean_omega_z)*((double)omega_z_samples[i] - mean_omega_z);
		}
		var_omega_x = var_omega_x / (N - 1);
		var_omega_y = var_omega_y / (N - 1);
		var_omega_z = var_omega_z / (N - 1);
		stddev_omega_x = sqrt(var_omega_x);
		stddev_omega_y = sqrt(var_omega_y);
		stddev_omega_z = sqrt(var_omega_z);

		printf("[Gyroscope Statistics]\n");
		printf("Total samples: %d\n", total_sample_count);
		printf("Total sampling time: %fs\n", elapsed_time);
		printf("Total angular drift (deg): : <%f, %f, %f>\n", theta_x, theta_y, theta_z);
		printf("Angular drift rate (deg/s): : <%f, %f, %f>\n",
			fabs(theta_x / elapsed_time), fabs(theta_y / elapsed_time), fabs(theta_z / elapsed_time));
		printf("Mean time delta: %fs\n", elapsed_time / (float)total_sample_count);
		printf("Mean angular velocity (deg/s): <%f, %f, %f>\n", mean_omega_x, mean_omega_y, mean_omega_z);
		printf("Std. dev angular velocity (deg/s): <%f, %f, %f>\n", stddev_omega_x, stddev_omega_y, stddev_omega_z);
		printf("Angular velocity variance (deg/s/s): <%f, %f, %f>\n", var_omega_x, var_omega_y, var_omega_z);
		printf("[Madgwick Parameters]\n");
		printf("[Beta] Max angular velocity variance (deg/s/s): %f\n", fmax(fmax(var_omega_x, var_omega_y), var_omega_z));
		printf("[Zeta] Max drift rate (deg/s): %f\n",
			fmax(fmax(fabs(theta_x / elapsed_time), fabs(theta_y / elapsed_time)), fabs(theta_z / elapsed_time)));
		fflush(stdout);
    }
	else
	{
		fprintf(stderr, "Controller must be connected over bluetooth.\n");
	}

    psmove_disconnect(move);
	psmove_shutdown();

    return EXIT_SUCCESS;
}

//-- private methods -----
static bool
is_move_stable_and_aligned_with_gravity(PSMove *move)
{
	const float k_cosine_10_degrees = 0.984808f;

	PSMove_3AxisVector k_identity_gravity_vector;
	PSMove_3AxisVector acceleration_direction;
	float acceleration_magnitude;
	bool isOk;

	// Get the direction the gravity vector should be pointing 
	// while the controller is in cradle pose.
	psmove_get_identity_gravity_calibration_direction(move, &k_identity_gravity_vector);
	psmove_get_accelerometer_frame(move, Frame_SecondHalf, 
		&acceleration_direction.x, &acceleration_direction.y, &acceleration_direction.z);
	acceleration_magnitude = psmove_3axisvector_normalize_with_default(&acceleration_direction, k_psmove_vector_zero);

	isOk =
		is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
		psmove_3axisvector_dot(&k_identity_gravity_vector, &acceleration_direction) >= k_cosine_10_degrees;

	return isOk;
}
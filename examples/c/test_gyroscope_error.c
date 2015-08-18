
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
/*
#ifdef _MSC_VER
    #include <conio.h>
#else
#include <curses.h>  // Need to link to libcurses
#endif
 */
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "psmove.h"

#define DESIRED_NOISE_SAMPLE_COUNT 1000
#define DESIRED_SAMPLING_TIME 30 // seconds
#define RADIANS_TO_DEGREES (180.0f / (float)M_PI)

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
		double theta_x = 0.f, theta_y = 0.f, theta_z = 0.f;
		double mean_omega_x = 0.f, mean_omega_y = 0.f, mean_omega_z = 0.f;
		double var_omega_x = 0.f, var_omega_y = 0.f, var_omega_z = 0.f;
		double stddev_omega_x = 0.f, stddev_omega_y = 0.f, stddev_omega_z = 0.f;
		double elapsed_time = 0.f;
		int noise_sample_count = 0;
		int total_sample_count = 0;

		fprintf(stdout, "Press any keyboard key once the controller is stable, pointing upright.\n");
        //getch();
		fprintf(stdout, "Sampling gyro for %d seconds...\n", DESIRED_SAMPLING_TIME);

		PSMove_timestamp last_update_time = psmove_timestamp();

		while (noise_sample_count < DESIRED_NOISE_SAMPLE_COUNT || elapsed_time < DESIRED_SAMPLING_TIME)
		{
            int res = psmove_poll(move);

            if (res) 
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

				last_update_time = update_time;
            }
        }

		double N = (double)total_sample_count;

		// Compute the mean of the samples
		for (int i = 0; i < DESIRED_NOISE_SAMPLE_COUNT; i++)
		{
			mean_omega_x += (double)omega_x_samples[i];
			mean_omega_y += (double)omega_y_samples[i];
			mean_omega_z += (double)omega_z_samples[i];
		}
		mean_omega_x /= N;
		mean_omega_y /= N;
		mean_omega_z /= N;

		// Compute the standard deviation of the samples
		for (int i = 0; i < DESIRED_NOISE_SAMPLE_COUNT; i++)
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

		fprintf(stdout, "[Gyroscope Statistics]\n");
		fprintf(stdout, "Total samples: %d\n", 
			total_sample_count);
		fprintf(stdout, "Total sampling time: %fs\n", 
			elapsed_time);
		fprintf(stdout, "Total angular drift (deg): : <%f, %f, %f>\n",
			theta_x, theta_y, theta_z);
		fprintf(stdout, "Angular drift rate (deg/s): : <%f, %f, %f>\n",
			fabs(theta_x / elapsed_time), fabs(theta_y / elapsed_time), fabs(theta_z / elapsed_time));
		fprintf(stdout, "Mean time delta: %fs\n", 
			elapsed_time / (float)total_sample_count);
		fprintf(stdout, "Mean angular velocity (deg/s): <%f, %f, %f>\n", 
			mean_omega_x, mean_omega_y, mean_omega_z);
		fprintf(stdout, "Std. dev angular velocity (deg/s): <%f, %f, %f>\n", 
			stddev_omega_x, stddev_omega_y, stddev_omega_z);
		fprintf(stdout, "Angular velocity variance (deg/s/s): <%f, %f, %f>\n",
			var_omega_x, var_omega_y, var_omega_z);
		fprintf(stdout, "[Madgwick Parameters]\n");
		fprintf(stdout, "[Beta] Max angular velocity variance (deg/s/s): %f\n",
			fmax(fmax(var_omega_x, var_omega_y), var_omega_z));
		fprintf(stdout, "[Zeta] Max drift rate (deg/s): %f\n",
			fmax(fmax(fabs(theta_x / elapsed_time), fabs(theta_y / elapsed_time)), fabs(theta_z / elapsed_time)));
        //getch();
    }
	else
	{
		fprintf(stderr, "Controller must be connected over bluetooth.\n");
	}

    psmove_disconnect(move);
	psmove_shutdown();

    return EXIT_SUCCESS;
}


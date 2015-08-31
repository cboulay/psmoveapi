
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

//-- includes ----
#include <stdio.h>
#include <stdlib.h>

#include "psmove.h"
#include "psmove_tracker.h"
#include "math/psmove_vector.h"

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

//-- constants -----
#define STABILIZE_WAIT_TIME_MS 1000
#define DESIRED_TRACKER_SAMPLE_COUNT 100

enum eCalibrationState
{
	Calibration_WaitForTracking,
	Calibration_WaitForStableAndVisible,
	Calibration_MeasureNoise,
	Calibration_Complete,
};

//-- macros -----
#define LOG_MESSAGE(f_, ...) fprintf(stdout, (f_), __VA_ARGS__);
#define LOG_ERROR(f_, ...) fprintf(stderr, (f_), __VA_ARGS__);

//-- prototypes -----
static bool is_move_stable_and_aligned_with_gravity(PSMove *move);

//-- public methods ----
int
main(int arg, char** args)
{
	PSMoveTracker* tracker = NULL;
	int psmove_count= 0;

	// Make sure the psmove API can be initialized
	if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
		LOG_ERROR("PS Move API init failed (wrong version?)\n");
		goto shutdown;
	}

	// Make sure there is at least one psmove controller attached
	LOG_MESSAGE("Looking for attached controllers...");
    psmove_count = psmove_count_connected();
	if (psmove_count <= 0) {
		LOG_ERROR("  No controllers found!\n");
		goto shutdown;
	}
	LOG_MESSAGE("  Found %d controllers\n", psmove_count);

	// Start capturing video
	{
		LOG_MESSAGE("Initializing PSMoveTracker...");

		// Do re-use color mapping data
		PSMoveTrackerSettings settings;
		psmove_tracker_settings_set_default(&settings);
		settings.color_mapping_max_age= 0;

		tracker = psmove_tracker_new_with_settings(&settings);
	}

    if (tracker)
	{
	    psmove_tracker_set_mirror(tracker, PSMove_True);

		// Do not use any smoothing since we want to sample the raw position data
		// to compute noise parameters used by the smoothing filters
		psmove_tracker_set_smoothing_type(tracker, Smoothing_None);
	}
	else
    {
        fprintf(stderr, "  Could not init PSMoveTracker.\n");
        goto shutdown;
    }
    LOG_MESSAGE("  Tracker OK\n");

	// Only need to run the calibration on one controller
    for (int psmove_index=0; psmove_index<psmove_count; psmove_index++) 
	{
        PSMove *move = psmove_connect_by_id(psmove_index);
		enum eCalibrationState calibration_state = Calibration_WaitForTracking;

		// Make sure accelerometer is calibrated
		// Otherwise we can't tell if the controller is sitting upright
		if (!psmove_has_calibration(move))
		{			
			char *serial= psmove_get_serial(move);

			LOG_ERROR("\nController #%d has bad calibration file (accelerometer values won't be correct).\n", psmove_index);

			if (serial != NULL)
			{
				LOG_ERROR("  Please delete %s.calibration and re-run psmovepair.exe with the controller plugged into usb.\n", serial);
				free(serial);
			}
			else
			{
				LOG_ERROR("  Please re-run psmovepair.exe with the controller plugged into usb.\n");
			}
			
			// Move on to the next controller
			psmove_disconnect(move);
			continue;
		}

		// Make sure the controller is connected via bluetooth
		if (psmove_connection_type(move) != Conn_Bluetooth)
		{
			LOG_ERROR("\nIgnoring non-Bluetooth PS Move #%d\n", psmove_index);

			// Move on to the next controller
			psmove_disconnect(move);
			continue;
		}

		// Calibrate the controller tracking color first
        while (calibration_state == Calibration_WaitForTracking) 
		{
            LOG_MESSAGE("\nCalibrating tracking color for controller %d...\n", psmove_index);
			PSMoveTracker_Status result = psmove_tracker_enable(tracker, move);

            if (result == Tracker_CALIBRATED) 
			{
                psmove_tracker_set_auto_update_leds(tracker, move, PSMove_True);

				// Move on to the next calibration state
                calibration_state= Calibration_WaitForStableAndVisible;
            } 
			else 
			{
                LOG_ERROR("  Failed to calibrate tracking color, retrying...\n");
            }
        }

		// Controller noise sampling
		{
			LOG_MESSAGE("\nStand the controller on a level surface with bolb in view of the camera.\n");
			LOG_MESSAGE("The controller should be about 3/4 of meter from the camera.\n");
			LOG_MESSAGE("Measurement will start once the controller is aligned with gravity and stable.\n");

			int stable_start_time = psmove_util_get_ticks();
			PSMove_3AxisVector accelerometer_samples[DESIRED_TRACKER_SAMPLE_COUNT]; // in g/s^2
			PSMove_3AxisVector position_samples[DESIRED_TRACKER_SAMPLE_COUNT]; // in cm
			int sample_count = 0;

			PSMoveTrackerSmoothingSettings smoothing_settings;
			psmove_tracker_smoothing_settings_set_default(&smoothing_settings);

			while (calibration_state != Calibration_Complete)
			{
				// Sleep 1 ms so that the previous frame can get processed
				cvWaitKey(1);

				// Poll the controller to get accelerometer data 
				if (psmove_poll(move) == 0)
				{
					continue;
				}

				// Grab the next video frame from the camera
				psmove_tracker_update_image(tracker);

				// Compute the raw position of the controller this frame
				psmove_tracker_update_cbb(tracker, move);
				psmove_tracker_annotate(tracker);
				psmove_tracker_get_location(tracker, move, 
					&position_samples[sample_count].x, 
					&position_samples[sample_count].y, 
					&position_samples[sample_count].z);

				// Get the most recent acceleration
				psmove_get_accelerometer_frame(move, Frame_SecondHalf,
					&accelerometer_samples[sample_count].x, 
					&accelerometer_samples[sample_count].y, 
					&accelerometer_samples[sample_count].z);

				// Render the frame to a window
				void *frame = psmove_tracker_get_frame(tracker);
				if (frame) 
				{
					cvShowImage("live camera feed", frame);
				}

				switch(calibration_state)
				{
				case Calibration_WaitForStableAndVisible:
					{
						bool is_visible= psmove_tracker_get_status(tracker, move) == Tracker_TRACKING;
						bool is_stable= is_move_stable_and_aligned_with_gravity(move);

						// Wait for the user set the controller on a stable surface that the camera can see
						if (is_visible && is_stable)
						{
							int current_time = psmove_util_get_ticks();
							int stable_duration = (current_time - stable_start_time);

							if ((current_time - stable_start_time) >= STABILIZE_WAIT_TIME_MS)
							{
								LOG_MESSAGE("\n\nMove stabilized. Starting position and accelerometer sampling.\n");
								calibration_state = Calibration_MeasureNoise;
							}
							else
							{
								LOG_MESSAGE("\rStable for: %dms/%dms                        ", stable_duration, STABILIZE_WAIT_TIME_MS);
							}
						}
						else
						{
							stable_start_time = psmove_util_get_ticks();

							if (!is_stable)
							{
								LOG_MESSAGE("\rMove Destabilized! Waiting for stabilization.");
							}

							if (!is_visible)
							{
								LOG_MESSAGE("\rMove Not Visible! Waiting for visibility.");
							}
						}
					}
					break;
				case Calibration_MeasureNoise:
					{
						bool is_visible= psmove_tracker_get_status(tracker, move) == Tracker_TRACKING;
						bool is_stable= is_move_stable_and_aligned_with_gravity(move);

						if (is_visible && is_stable)
						{
							PSMove_3AxisVector acceleration_mean;
							PSMove_3AxisTransform acceleration_covariance;
							PSMove_3AxisTransform position_covariance;

							sample_count++;

							if (sample_count >= DESIRED_TRACKER_SAMPLE_COUNT)
							{
								// Compute the 3x3 covariance matrix for the position and acceleration noise.
								// This assumes that the controller was stationary during sampling and thus any
								// variation in readings is due to noise in measurement and not some other source.
								psmove_point_cloud_compute_covariance(
									accelerometer_samples, sample_count,
									&acceleration_mean, &acceleration_covariance);
								psmove_point_cloud_compute_covariance(
									position_samples, sample_count,
									NULL, &position_covariance);

								//TODO: Compute the skew in the accelerometer data
								// We know that the controller should be sitting level upright
								// so it should be reading on average <0g, +1g, 0g>, 
								// But for some reason the Y-axis accelerometer tends to read 
								// around 0.91g instead and if you turn the controller upside-down 
								// you read around -1.09g, thus the Y axis values are skewed by about -0.09g
								// The X and Z axis tend to be pretty close to spot on.

								// We only want a single variance value for the acceleration since the 
								// accelerometer /should/ have the same amount of noise across all 3 axes.
								smoothing_settings.acceleration_variance =
										MAX(MAX(acceleration_covariance.row0[0], acceleration_covariance.row1[1]), acceleration_covariance.row2[2]);

								// The positional noise covariance is used in the correction phase of the Kalman filter.
								// We keep this as a matrix because the position variance is different on each axis.
								// The Z-axis (toward the camera) since that is based off the size of the tracking sphere
								// while the X and Y axes (perpendicular to the camera) are based off tracking sphere centroid.
								smoothing_settings.measurement_covariance.row0[0] = position_covariance.row0[0];
								smoothing_settings.measurement_covariance.row1[1] = position_covariance.row1[1];
								smoothing_settings.measurement_covariance.row2[2] = position_covariance.row2[2];

								calibration_state = Calibration_Complete;
							}
							else
							{
								LOG_MESSAGE("\rSample %d/%d                   ", sample_count, DESIRED_TRACKER_SAMPLE_COUNT);
							}
						}
						else
						{
							sample_count= 0;
							calibration_state = Calibration_WaitForStableAndVisible;

							if (!is_stable)
							{
								LOG_MESSAGE("\rMove Destabilized! Waiting for stabilization.");
							}

							if (!is_visible)
							{
								LOG_MESSAGE("\rMove Not Visible! Waiting for visibility.");
							}
						}
					}
					break;
				}				
			}

			// Save the calibration settings
			if (psmove_save_smoothing_settings(&smoothing_settings) == PSMove_False)
			{
				LOG_ERROR("\rFailed to save smoothing settings file!.\n");
			}
		}

        printf("\nFinished PS Move #%d\n", psmove_index);
        psmove_disconnect(move);

		// Only needed to run this successfully on one controller
		break;
    }

shutdown:
	if (tracker != NULL)
	{
		psmove_tracker_free(tracker);
		tracker= NULL;
	}

	psmove_shutdown();

    return 0;
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
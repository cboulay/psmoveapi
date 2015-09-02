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
#include <memory.h>
#include <assert.h>

#include "psmove.h"
#include "psmove_tracker.h"
#include "psmove_kalman_filter.h"
#include "unit_test.h"

//-- macros -----
#ifdef UNIT_TEST_LOGGING
#define LOG_MESSAGE(f_, ...) fprintf(stdout, (f_), ##__VA_ARGS__);
#else
#define LOG_MESSAGE(f_, ...)
#endif

//-- public interface -----
bool run_kalman_filter_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("kalman_filter")
		UNIT_TEST_MODULE_CALL_TEST(psmove_kalman_filter_test_sample_data);
	UNIT_TEST_MODULE_END()
}

//-- private functions -----
bool
psmove_kalman_filter_test_sample_data()
{
	UNIT_TEST_BEGIN("filter sample data")

	PSMove_3AxisVector positionSamples[] = {
		psmove_3axisvector_xyz(-1146.9f, -4807.3f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.3f, 628.1f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.4f),
		psmove_3axisvector_xyz(-1164.9f, -4813.3f, 619.5f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1147.0f, -4807.1f, 627.9f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.8f, -4807.2f, 628.4f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.8f, -4813.1f, 619.8f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.3f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1146.8f, -4807.2f, 628.4f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1129.4f, -4801.2f, 636.3f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.8f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.8f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.1f, 619.7f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.3f, 619.5f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.2f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.3f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.1f, 619.8f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.7f, -4813.1f, 620.3f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.3f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1146.8f, -4807.2f, 628.4f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.3f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1164.8f, -4813.2f, 619.7f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.3f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f),
		psmove_3axisvector_xyz(-1183.0f, -4819.3f, 611.5f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1164.9f, -4813.2f, 619.6f),
		psmove_3axisvector_xyz(-1146.9f, -4807.2f, 628.1f),
		psmove_3axisvector_xyz(-1183.1f, -4819.3f, 611.3f)
	};
	const int sample_count= sizeof( positionSamples ) / sizeof( positionSamples[ 0 ] );

	PSMovePositionKalmanFilter *position_filter= psmove_position_kalman_filter_new();

	PSMoveTrackerSmoothingSettings settings;	
	memset(&settings, 0, sizeof(settings));
	settings.acceleration_variance = 0.f; // 1754.46228f;	

	settings.measurement_covariance.row0[0] = 132.710083f; settings.measurement_covariance.row0[1] = 0.f; settings.measurement_covariance.row0[2] = 0.f;
	settings.measurement_covariance.row1[0] = 0.f; settings.measurement_covariance.row1[1] = 14.8531485f; settings.measurement_covariance.row1[2] = 0.f;
	settings.measurement_covariance.row2[0] = 0.f; settings.measurement_covariance.row2[1] = 0.f; settings.measurement_covariance.row2[2] = 28.6374550f;

	psmove_position_kalman_filter_init(&settings, &positionSamples[0], position_filter);

	// Simulate a run of the filter for 5 seconds
	{
		float SimTime = 0;
		LOG_MESSAGE("    Time\t    RawX\t    RawY\t    RawZ\t       X\t       Y\t       Z\t      VX\t      VY\t      VZ\t    VarX\t    VarY\t    VarZ\t   VarVX\t   VarVY\t   VarVZ\n");

		for (int sampleIndex = 0; sampleIndex < sample_count; ++sampleIndex)
		{
			const float time_delta = 0.03333f; // 30 FPS
			PSMove_3AxisVector raw_position = positionSamples[sampleIndex];
			PSMove_3AxisVector raw_acceleration = psmove_3axisvector_xyz(0.f, 0.f, 0.f);

			psmove_position_kalman_filter_update(
				&settings, 
				&raw_position, 
				&raw_acceleration, 
				position_filter);
			
			PSMove_3AxisVector filtered_position = psmove_position_kalman_filter_get_position(position_filter);
			PSMove_3AxisVector filtered_velocity = psmove_position_kalman_filter_get_velocity(position_filter);
			PSMove_3AxisVector filtered_position_variance = psmove_position_kalman_filter_get_position_variance(position_filter);
			PSMove_3AxisVector filtered_velocity_variance = psmove_position_kalman_filter_get_velocity_variance(position_filter);

			LOG_MESSAGE(
				"%8.2f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\t%8.1f\n",
				SimTime,
				raw_position.x,
				raw_position.y,
				raw_position.z,
				filtered_position.x,
				filtered_position.y,
				filtered_position.z,
				filtered_velocity.x,
				filtered_velocity.y,
				filtered_velocity.z,
				filtered_position_variance.x,
				filtered_position_variance.y,
				filtered_position_variance.z,
				filtered_velocity_variance.x,
				filtered_velocity_variance.y,
				filtered_velocity_variance.z);

			if (psmove_3axisvector_length_between(&filtered_position, &raw_position) > 100.f)
			{
				// Any deviation by more than 100mm means something has gone horribly wrong
				LOG_MESSAGE("Extreme filter position deviation at sample index: %d\n", sampleIndex);
				success = false;
			}

			if (psmove_3axisvector_length(&filtered_velocity) > 100.f)
			{
				// Any deviation by more than 100mm/sec means something has gone horribly wrong
				LOG_MESSAGE("Extreme filter velocity at sample index: %d\n", sampleIndex);
				success = false;
			}

			SimTime += time_delta;
            unsigned long sleep_for = (unsigned long)(time_delta * 1000.f);
            PSMove_timestamp before_sleep = psmove_timestamp();
            psmove_sleep(sleep_for);
            PSMove_timestamp after_sleep = psmove_timestamp();
            float time_diff = (float)psmove_timestamp_value(psmove_timestamp_diff(after_sleep, before_sleep));
            printf("psmove_sleep(%lu) lasted %f seconds.\n", sleep_for, time_diff);
		}
	}

	psmove_position_kalman_filter_free(position_filter);

	assert(success);
	UNIT_TEST_COMPLETE()
}
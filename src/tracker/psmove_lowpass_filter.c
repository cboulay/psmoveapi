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
#include <stdlib.h>

#include "psmove_lowpass_filter.h"
#include "../psmove_private.h"
#include "psmove_tracker.h"
#include "psmove_vector.h"

//-- structures ----
struct _PSMovePositionLowPassFilter {
    // x/y/z coordinates of sphere in cm from camera focal point
    PSMove_3AxisVector position;

    bool is_initialized;
};

//-- public methods -----
PSMovePositionLowPassFilter *
psmove_position_lowpass_filter_new()
{
    PSMovePositionLowPassFilter *filter_state = (PSMovePositionLowPassFilter *)calloc(1, sizeof(PSMovePositionLowPassFilter));

    filter_state->is_initialized = false;
    filter_state->position = psmove_3axisvector_xyz(0.f, 0.f, 0.f);

    return filter_state;
}

void
psmove_position_lowpass_filter_free(PSMovePositionLowPassFilter *filter_state)
{
    psmove_return_if_fail(filter_state != NULL);
    free(filter_state);
}

void
psmove_position_lowpass_filter_init(
    const PSMove_3AxisVector *position,
    PSMovePositionLowPassFilter *filter_state)
{
    filter_state->position= *position;
    filter_state->is_initialized = true;
}

PSMove_3AxisVector
psmove_position_lowpass_filter_get_position(PSMovePositionLowPassFilter *filter_state)
{
    return filter_state->position;
}

void 
psmove_position_lowpass_filter_update(
    const PSMoveTrackerSmoothingSettings *tracker_settings,
    const PSMove_3AxisVector *measured_position,	// The position measured by the sensors.
    PSMovePositionLowPassFilter *filter_state)
{
    if (psmove_3axisvector_is_valid(measured_position))
    {
        if (!filter_state->is_initialized)
        {
            psmove_position_lowpass_filter_init(measured_position, filter_state);
        }
        else
        {
            // Traveling 10 cm in one frame should have 0 smoothing
            // Traveling 0+noise cm in one frame should have
            // 60% xy smoothing, 80% z smoothing
            float distance = psmove_3axisvector_length_between(&filter_state->position, measured_position);
            float new_xy_weight = clampf01(lerpf(0.40f, 1.00f, distance/10.f));
            filter_state->position.x = lerpf(filter_state->position.x, measured_position->x, new_xy_weight);
            filter_state->position.y = lerpf(filter_state->position.y, measured_position->y, new_xy_weight);

            float new_z_weight = clampf01(lerpf(0.20f, 1.0f, distance/10.f));
            filter_state->position.z = lerpf(filter_state->position.z, measured_position->z, new_z_weight);
        }
    }
}
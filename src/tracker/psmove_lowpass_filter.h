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

/* Math functions related to Kalman filtering */
#ifndef __PSMOVE_LOWPASS_FILTER_H
#define __PSMOVE_LOWPASS_FILTER_H

//-- includes -----
#include "psmove_platform_config.h"
#include "psmove_math.h"
#include "psmove_vector.h"
#include "psmove.h"

//-- pre-declarations -----
struct _PSMovePositionLowPassFilter;
typedef struct _PSMovePositionLowPassFilter PSMovePositionLowPassFilter;

struct _PSMoveTrackerSmoothingSettings;
typedef struct _PSMoveTrackerSmoothingSettings PSMoveTrackerSmoothingSettings;

//-- interface -----
#ifdef __cplusplus
extern "C" {
#endif	

ADDAPI PSMovePositionLowPassFilter *
ADDCALL psmove_position_lowpass_filter_new();

ADDAPI void
ADDCALL psmove_position_lowpass_filter_free(PSMovePositionLowPassFilter *filter_state);

ADDAPI void
ADDCALL psmove_position_lowpass_filter_init(
	const PSMove_3AxisVector *position,
	PSMovePositionLowPassFilter *filter_state);

ADDAPI PSMove_3AxisVector
ADDCALL psmove_position_lowpass_filter_get_position(PSMovePositionLowPassFilter *filter_state);

ADDAPI void 
ADDCALL psmove_position_lowpass_filter_update(
	const PSMoveTrackerSmoothingSettings *tracker_settings,
	const PSMove_3AxisVector *measured_position,	// The position measured by the sensors.
	PSMovePositionLowPassFilter *filter_state);

#ifdef __cplusplus
}
#endif

#endif // __PSMOVE_KALMAN_FILTER_H
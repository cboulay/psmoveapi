/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * Copyright (c) 2012 Benjamin Venditt <benjamin.venditti@gmail.com>
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

#include "psmove_platform_config.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>

#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"

#include "psmove_tracker.h"
#include "../psmove_private.h"
#include "psmove_tracker_private.h"
#include "psmove_kalman_filter.h"
#include "psmove_lowpass_filter.h"

#include "camera_control.h"
#include "camera_control_private.h"
#include "tracker_helpers.h"
#include "tracker_trace.h"

#ifdef __linux
#  include "platform/psmove_linuxsupport.h"
#endif

#ifdef _MSC_VER
#include <Windows.h>
#endif

//#define DEBUG_WINDOWS 			// shall additional windows be shown
#define ROIS 4                          // the number of levels of regions of interest (roi)
#define BLINKS 2                        // number of diff images to create during calibration
#define COLOR_MAPPING_RING_BUFFER_SIZE 256  /* Has to be 256, so that next_slot automatically wraps */

#define PSEYE_BACKUP_FILE "PSEye_backup.ini"
#define INTRINSICS_XML "intrinsics.xml"
#define DISTORTION_XML "distortion.xml"
#define COLOR_MAPPING_DAT "colormapping.dat"
#define SMOOTHING_SETTINGS_CSV "smoothing_settings.csv"
#define SPHERE_RADIUS_CM 2.25

// Smoothing defaults
static const PSMoveTrackerSmoothingSettings tracker_smoothing_default_settings = {
    .filter_do_2d_xy = 1,
    .filter_do_2d_r = 1,
    .filter_3d_type = Smoothing_None,
    .acceleration_variance = 0.f,
    .measurement_covariance = { 5.f, 0,   0,
                                0,   5.f, 0,
                                0,   0,  10.f}
};

static const PSMoveTrackerSettings tracker_default_settings = {
    .camera_frame_width = 0,
    .camera_frame_height = 0,
    .camera_frame_rate = 0,
    .camera_auto_gain = PSMove_False,
    .camera_gain = 0,
    .camera_auto_white_balance = PSMove_False,
    .camera_exposure = (255 * 15) / 0xFFFF,
    .camera_brightness = 0,
    .camera_mirror = PSMove_True,
    .exposure_mode = Exposure_LOW,
    .calibration_blink_delay = 200,
    .calibration_diff_t = 20,
    .calibration_min_size = 50,
    .calibration_max_distance = 30,
    .calibration_size_std = 10,
    .color_mapping_max_age = 2 * 60 * 60,
    .dimming_factor = 1.f,
    .color_hue_filter_range = 20,
    .color_saturation_filter_range = 85,
    .color_value_filter_range = 85,
    .use_fitEllipse = 0,
    .color_adaption_quality_t = 35.f,
    .color_update_rate = 1.f,
    .search_tile_width = 0,
    .search_tile_height = 0,
    .search_tiles_horizontal = 0,
    .search_tiles_count = 0,
    .roi_adjust_fps_t = 160,
    .tracker_quality_t1 = 0.3f,
    .tracker_quality_t2 = 0.7f,
    .tracker_quality_t3 = 4.7f,
    .color_update_quality_t1 = 0.8,
    .color_update_quality_t2 = 0.2,
    .color_update_quality_t3 = 6.f,
    .xorigin_cm =  0.f,
    .yorigin_cm = 0.f,
    .zorigin_cm = 0.f
};


#define POSITION_FILTER_RESET_TIME 1.0f // time since last position update after which we just reset the 

/**
 * Syntactic sugar - iterate over all valid controllers of a tracker
 *
 * Usage example:
 *
 *    TrackedController *tc;
 *    for_each_controller (tracker, tc) {
 *        // do something with "tc" here
 *    }
 *
 **/
#define for_each_controller(tracker, var) \
    for (var=tracker->controllers; var<tracker->controllers+PSMOVE_TRACKER_MAX_CONTROLLERS; var++) \
        if (var->move)

struct _TrackedController {
    /* Move controller, or NULL if free slot */
    PSMove* move;

    /* Assigned RGB color of the controller */
    struct PSMove_RGBValue color;

    CvScalar eFColor;			// first estimated color (BGR)
    CvScalar eFColorHSV;		// first estimated color (HSV)

    CvScalar eColor;			// estimated color (BGR)
    CvScalar eColorHSV; 		// estimated color (HSV)

    int roi_x, roi_y;			// x/y - Coordinates of the ROI
    int roi_level; 	 			// the current index for the level of ROI
    enum PSMove_Bool roi_level_fixed;    // true if the ROI level should be fixed
    float mx, my;				// x/y - Coordinates of center of mass of the blob
    float x, y, r;				// x/y - Coordinates of the controllers sphere and its radius
    float old_x, old_y, old_r;
    int search_tile; 			// current search quadrant when controller is not found (reset to 0 if found)
    float rs;					// a smoothed variant of the radius

    // For positional tracker
	void *position_filter;
    PSMove_3AxisVector position_cm;     // x/y/z coordinates of sphere in cm from camera focal point
    PSMove_3AxisVector position_offset; // x/y/z offsets. Can be used to define origin in camera-space.
    float el_major, el_angle;           // For the found ellipse. Used for annotation only.

    float q1, q2, q3;   // Calculated quality criteria from the tracker
    bool quality_check; // If q1, q2, q3 all pass.

	bool was_tracked;				// tracked previous frame
	bool is_tracked;				// tracked this frame
    PSMove_timestamp last_color_update;	// the timestamp when the last color adaption has been performed
    PSMove_timestamp last_position_update; // the timestamp when the last position update has been performed
    enum PSMove_Bool auto_update_leds;
};
typedef struct _TrackedController TrackedController;


/**
 * A ring buffer used for saving color mappings of previous sessions. There
 * is a pointer "next_slot" that will point to the next free slot. From there,
 * new values can be saved (forward) and old values can be searched (backward).
 **/
struct ColorMappingRingBuffer {
    struct {
        /* The RGB LED value */
        struct PSMove_RGBValue from;

        /* The dimming factor for which this mapping is valid */
        unsigned char dimming;

        /* The value of the controller in the camera */
        struct PSMove_RGBValue to;
    } map[COLOR_MAPPING_RING_BUFFER_SIZE];
    unsigned char next_slot;
};

/**
 * Parameters of the Pearson type VII distribution
 * Source: http://fityk.nieto.pl/model.html
 * Used for calculating the distance from the radius
 **/
struct PSMoveTracker_DistanceParameters {
    float height;
    float center;
    float hwhm;
    float shape;
};

/**
 * Experimentally-determined parameters for a PS Eye camera
 * in wide angle mode with a PS Move, color = (255, 0, 255)
 **/
static struct PSMoveTracker_DistanceParameters
pseye_distance_parameters = {
    /* height = */ 517.281f,
    /* center = */ 1.297338f,
    /* hwhm = */ 3.752844f,
    /* shape = */ 0.4762335f,
};

struct _PSMoveTracker {
    CameraControl* cc;

    PSMoveTrackerSettings settings;  // Camera and tracker algorithm settings. Generally do not change after startup & calibration.
	// The type of position smoothing to use
	enum PSMoveTracker_Smoothing_Type smoothing_type;

    // Settings used by the positional smoothing algorithms
	PSMoveTrackerSmoothingSettings smoothing_settings;

    /* Timestamps for performance measurements */
    PSMove_timestamp ts_camera_begin; // when the capture was started
    PSMove_timestamp ts_camera_grab; // when the image was grabbed
    PSMove_timestamp ts_camera_retrieve; // when the image was retrieved
    PSMove_timestamp ts_camera_converted; // when the image was converted

    IplImage* frame; // the current frame of the camera
    IplImage *frame_rgb; // the frame as tightly packed RGB data
    IplImage* roiI[ROIS]; // array of images for each level of roi (colored)
    IplImage* roiM[ROIS]; // array of images for each level of roi (greyscale)
    IplConvKernel* kCalib; // kernel used for morphological operations during calibration
    CvScalar rHSV; // the range of the color filter

    // Parameters for psmove_tracker_distance_from_radius()
    struct PSMoveTracker_DistanceParameters distance_parameters;

    TrackedController controllers[PSMOVE_TRACKER_MAX_CONTROLLERS]; // controller data

    struct ColorMappingRingBuffer color_mapping; // remembered color mappings

    CvMemStorage* storage; // use to store the result of cvFindContour and cvHughCircles
    long duration; // duration of tracking operation, in ms

    // internal variables (debug)
    float debug_fps; // the current FPS achieved by "psmove_tracker_update"
};

/* Preset colors - use them in ascending order if not used yet */
#define N_PRESET_COLORS 5
static const struct PSMove_RGBValue preset_colors[N_PRESET_COLORS] = {
    { 0xFF, 0x00, 0xFF }, /* magenta */
    { 0x00, 0xFF, 0xFF }, /* cyan */
    { 0xFF, 0xFF, 0x00 }, /* yellow */
    { 0xFF, 0x00, 0x00 }, /* red */
    #ifdef __APPLE__
        { 0x00, 0xFF, 0x00 }, /* green */
    #else
        { 0x00, 0x00, 0xFF }, /* blue */
    #endif
};

// -------- START: internal functions only

/**
 * Adapts the cameras exposure to the current lighting conditions
 *
 * This function will find the most suitable exposure.
 *
 * tracker - A valid PSMoveTracker * instance
 * target_luminance - The target luminance value (higher = brighter)
 *
 * Returns: the most suitable exposure
 **/
int psmove_tracker_adapt_to_light(PSMoveTracker *tracker, float target_luminance);

/**
 * Find the TrackedController * for a given PSMove * instance
 *
 * if move == NULL, the next free slot will be returned
 *
 * Returns the TrackedController * instance, or NULL if not found
 **/
TrackedController *
psmove_tracker_find_controller(PSMoveTracker *tracker, PSMove *move);


/**
 * Wait for a given time for a frame from the tracker
 *
 * tracker - A valid PSMoveTracker * instance
 * frame - A pointer to an IplImage * to store the frame
 * delay - The delay to wait for the frame
 **/
void
psmove_tracker_wait_for_frame(PSMoveTracker *tracker, IplImage **frame, int delay);

/**
 * This function switches the sphere of the given PSMove on to the given color and takes
 * a picture via the given capture. Then it switches it of and takes a picture again. A difference image
 * is calculated from these two images. It stores the image of the lit sphere and
 * of the diff-image in the passed parameter "on" and "diff". Before taking
 * a picture it waits for the specified delay (in microseconds).
 *
 * tracker - the tracker that contains the camera control
 * move    - the PSMove controller to use
 * rgb     - the RGB color to use to lit the sphere
 * on	   - the pre-allocated image to store the captured image when the sphere is lit
 * diff    - the pre-allocated image to store the calculated diff-image
 * delay   - the time to wait before taking a picture (in microseconds)
 **/
void
psmove_tracker_get_diff(PSMoveTracker* tracker, PSMove* move,
        struct PSMove_RGBValue rgb, IplImage* on, IplImage* diff, int delay,
        float dimming_factor);

/**
 * This function seths the rectangle of the ROI and assures that the itis always within the bounds
 * of the camera image.
 *
 * tracker          - A valid PSMoveTracker * instance
 * tc         - The TrackableController containing the roi to check & fix
 * roi_x	  - the x-part of the coordinate of the roi
 * roi_y	  - the y-part of the coordinate of the roi
 * roi_width  - the width of the roi
 * roi_height - the height of the roi
 * cam_width  - the width of the camera image
 * cam_height - the height of the camera image
 **/
void psmove_tracker_set_roi(PSMoveTracker* tracker, TrackedController* tc, int roi_x, int roi_y, int roi_width, int roi_height);

/**
 * This function is just the internal implementation of "psmove_tracker_update"
 */
int psmove_tracker_update_controller(PSMoveTracker* tracker, TrackedController *tc);

/**
* This function takes a vetted contour and calculates sphere 2D position & radius, and 3D position.
*/
void psmove_tracker_update_controller_position_from_contour(PSMoveTracker *tracker, TrackedController *tc, CvSeq* contourBest);

/**
* This function filters newly updated sphere 2D position & radius.
*/
void psmove_tracker_filter_2d(PSMoveTracker *tracker, TrackedController *tc, CvSeq* contourBest);

/**
* This function filters a newly-updated sphere position. It uses tracker->settings to determine filtering technique.
*/
void psmove_tracker_filter_3d(PSMoveTracker *tracker, TrackedController *tc);


/*
 *  This finds the biggest contour within the given image.
 *
 *  img  		- (in) 	the binary image to search for contours
 *  stor 		- (out) a storage that can be used to save the result of this function
 *  resContour 	- (out) points to the biggest contour found within the image
 *  resSize 	- (out)	the size of that contour in px²
 */
void psmove_tracker_biggest_contour(IplImage* img, CvMemStorage* stor, CvSeq** resContour, float* resSize);

/*
 * This returns a subjective distance between the first estimated (during calibration process) color and the currently estimated color.
 * Subjective, because it takes the different color components not equally into account.
 *    Result calculates like: abs(c1.h-c2.h) + abs(c1.s-c2.s)*0.5 + abs(c1.v-c2.v)*0.5
 *
 * tc - The controller whose first/current color estimation distance should be calculated.
 *
 * Returns: a subjective distance
 */
float psmove_tracker_hsvcolor_diff(TrackedController* tc);

/*
 * This will estimate the position and the radius of the orb.
 * It will calcualte the radius by findin the two most distant points
 * in the contour. And its by choosing the mid point of those two.
 *
 * cont 	- (in) 	The contour representing the orb.
 * x            - (out) The X coordinate of the center.
 * y            - (out) The Y coordinate of the center.
 * radius	- (out) The radius of the contour that is calculated here.
 */
void
psmove_tracker_estimate_circle_from_contour(CvSeq* cont, float *x, float *y, float* radius);

/*
 * This function return a optimal ROI center point for a given Tracked controller.
 * On very fast movements, it may happen that the orb is visible in the ROI, but resides
 * at its border. This function will simply look for the biggest blob in the ROI and return a
 * point so that that blob would be in the center of the ROI.
 *
 * tc - (in) The controller whose ROI centerpoint should be adjusted.
 * tracker  - (in) The PSMoveTracker to use.
 * center - (out) The better center point for the current ROI
 *
 * Returns: nonzero if a new point was found, zero otherwise
 */
int
psmove_tracker_center_roi_on_controller(TrackedController* tc, PSMoveTracker* tracker, CvPoint *center);

int
psmove_tracker_color_is_used(PSMoveTracker *tracker, struct PSMove_RGBValue color);

enum PSMoveTracker_Status
psmove_tracker_enable_with_color_internal(PSMoveTracker *tracker, PSMove *move, struct PSMove_RGBValue color);

/*
 * This function reads old calibration color values and tries to track the controller with that color.
 * if it works, the function returns 1, 0 otherwise.
 * Can help to speed up calibration process on application startup.
 *
 * tracker     - (in) A valid PSMoveTracker
 * move  - (in) A valid PSMove controller
 * rgb - (in) The color the PSMove controller's sphere will be lit.
 */

int
psmove_tracker_old_color_is_tracked(PSMoveTracker* tracker, PSMove* move, struct PSMove_RGBValue rgb);

/**
 * Lookup a camera-visible color value
 **/
int
psmove_tracker_lookup_color(PSMoveTracker *tracker, struct PSMove_RGBValue rgb, CvScalar *color);

/**
 * Remember a color value after calibration
 **/
void
psmove_tracker_remember_color(PSMoveTracker *tracker, struct PSMove_RGBValue rgb, CvScalar color);

// -------- END: internal functions only

void
psmove_tracker_get_settings(PSMoveTracker *tracker, PSMoveTrackerSettings *settings)
{
    psmove_return_if_fail(tracker != NULL);
    *settings = tracker->settings;
}

void
psmove_tracker_set_settings(PSMoveTracker *tracker, PSMoveTrackerSettings *settings)
{
    psmove_return_if_fail(tracker != NULL);
    tracker->settings = *settings;
}

void
psmove_tracker_settings_set_default(PSMoveTrackerSettings *settings)
{
    *settings = tracker_default_settings;
    printf("Copied default tracker settings to tracker->settings.\n");
}

PSMoveTracker *psmove_tracker_new() {
    PSMoveTrackerSettings settings;
    psmove_tracker_settings_set_default(&settings);
    return psmove_tracker_new_with_settings(&settings);
}

PSMoveTracker *
psmove_tracker_new_with_settings(PSMoveTrackerSettings *settings) {
    int camera = 0;

#if defined(__linux) && defined(PSMOVE_USE_PSEYE)
    /**
     * On Linux, we might have multiple cameras (e.g. most laptops have
     * built-in cameras), so we try looking for the one that is handled
     * by the PSEye driver.
     **/
    camera = linux_find_pseye();
    if (camera == -1) {
        /* Could not find the PSEye - fallback to first camera */
        camera = 0;
    }
#endif

    int camera_env = psmove_util_get_env_int(PSMOVE_TRACKER_CAMERA_ENV);
    if (camera_env != -1) {
        camera = camera_env;
        psmove_DEBUG("Using camera %d (%s is set)\n", camera,
                PSMOVE_TRACKER_CAMERA_ENV);
    }

    return psmove_tracker_new_with_camera_and_settings(camera, settings);
}

void
psmove_tracker_set_auto_update_leds(PSMoveTracker *tracker, PSMove *move,
        enum PSMove_Bool auto_update_leds)
{
    psmove_return_if_fail(tracker != NULL);
    psmove_return_if_fail(move != NULL);
    TrackedController *tc = psmove_tracker_find_controller(tracker, move);
    psmove_return_if_fail(tc != NULL);
    tc->auto_update_leds = auto_update_leds;
}


enum PSMove_Bool
psmove_tracker_get_auto_update_leds(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_val_if_fail(tracker != NULL, PSMove_False);
    psmove_return_val_if_fail(move != NULL, PSMove_False);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);
    psmove_return_val_if_fail(tc != NULL, PSMove_False);
    return tc->auto_update_leds;
}


void
psmove_tracker_set_dimming(PSMoveTracker *tracker, float dimming)
{
    psmove_return_if_fail(tracker != NULL);
    tracker->settings.dimming_factor = dimming;
}

float
psmove_tracker_get_dimming(PSMoveTracker *tracker)
{
    psmove_return_val_if_fail(tracker != NULL, 0);
    return tracker->settings.dimming_factor;
}

void
psmove_tracker_set_exposure(PSMoveTracker *tracker,
        enum PSMoveTracker_Exposure exposure)
{
    psmove_return_if_fail(tracker != NULL);
    tracker->settings.exposure_mode = exposure;

    float target_luminance = 0;
    switch (tracker->settings.exposure_mode) {
        case Exposure_LOW:
            target_luminance = 8;
            break;
        case Exposure_MEDIUM:
            target_luminance = 25;
            break;
        case Exposure_HIGH:
            target_luminance = 100;
            break;
        default:
            psmove_DEBUG("Invalid exposure mode: %d\n", exposure);
            break;
    }

    #if defined(__APPLE__) && !defined(CAMERA_CONTROL_USE_PS3EYE_DRIVER)
        camera_control_initialize();
    #endif

    // Determine the camera exposure setting required to hit the target luminance
    tracker->settings.camera_exposure = psmove_tracker_adapt_to_light(tracker, target_luminance);

    camera_control_set_parameters(tracker->cc, 0, 0, 0, tracker->settings.camera_exposure,
            0, 0xffff, 0xffff, 0xffff, -1, -1);
}

enum PSMoveTracker_Exposure
psmove_tracker_get_exposure(PSMoveTracker *tracker)
{
    psmove_return_val_if_fail(tracker != NULL, Exposure_INVALID);
    return tracker->settings.exposure_mode;
}

void
psmove_tracker_get_smoothing_settings(PSMoveTracker *tracker, PSMoveTrackerSmoothingSettings *smoothing_settings)
{
    psmove_return_if_fail(tracker != NULL);
    *smoothing_settings = tracker->smoothing_settings;
}

void
psmove_tracker_set_smoothing_settings(PSMoveTracker *tracker, PSMoveTrackerSmoothingSettings *smoothing_settings)
{
    psmove_return_if_fail(tracker != NULL);
    tracker->smoothing_settings = *smoothing_settings;
    psmove_tracker_set_smoothing_type(tracker, tracker->smoothing_settings.filter_3d_type);
}

void
psmove_tracker_smoothing_settings_set_default(PSMoveTrackerSmoothingSettings *smoothing_settings)
{
    *smoothing_settings = tracker_smoothing_default_settings;
}

enum PSMove_Bool
psmove_tracker_save_smoothing_settings(PSMoveTrackerSmoothingSettings *smoothing_settings)
{
    psmove_return_val_if_fail(smoothing_settings != NULL, PSMove_False);
    char *filepath = psmove_util_get_file_path(SMOOTHING_SETTINGS_CSV);

	FILE *fp= psmove_file_open(filepath, "w");
    free(filepath);
	psmove_return_val_if_fail(fp != NULL, PSMove_False);

    fprintf(fp, "name,value\n");
    fprintf(fp, "filter_do_2d_xy,%d\n", smoothing_settings->filter_do_2d_xy);
    fprintf(fp, "filter_do_2d_r,%d\n", smoothing_settings->filter_do_2d_r);
    fprintf(fp, "filter_3d_type,%d\n", smoothing_settings->filter_3d_type);
	fprintf(fp, "acceleration_variance,%f\n", smoothing_settings->acceleration_variance);
	fprintf(fp, "measurement_covariance_00,%f\n", smoothing_settings->measurement_covariance.row0[0]);
	fprintf(fp, "measurement_covariance_11,%f\n", smoothing_settings->measurement_covariance.row1[1]);
	fprintf(fp, "measurement_covariance_22,%f\n", smoothing_settings->measurement_covariance.row2[2]);

    psmove_file_close(fp);
	return PSMove_True;
}

enum PSMove_Bool
psmove_tracker_load_smoothing_settings(PSMoveTrackerSmoothingSettings *out_smoothing_settings)
{
	PSMoveTrackerSmoothingSettings smoothing_settings;
	enum PSMove_Bool success = PSMove_False;
	int result = 0;

	psmove_tracker_smoothing_settings_set_default(&smoothing_settings);

	psmove_return_val_if_fail(out_smoothing_settings != NULL, PSMove_False);

	char *filepath = psmove_util_get_file_path(SMOOTHING_SETTINGS_CSV);

	FILE *fp = psmove_file_open(filepath, "r");
    free(filepath);
	psmove_return_val_if_fail(fp != NULL, PSMove_False);

	char s_name[64], s_value[64];
	result = fscanf(fp, "%4s,%5s\n", s_name, s_value);
	psmove_goto_if_fail(result == 2, finish);
	psmove_goto_if_fail(strcmp(s_name, "name") == 0, finish);
	psmove_goto_if_fail(strcmp(s_value, "value") == 0, finish);

    char s_filter_do_2d_xy[64];
    result = fscanf(fp, "%19s,%d\n", &s_filter_do_2d_xy, &smoothing_settings.filter_do_2d_xy);
	psmove_goto_if_fail(result == 2, finish);
    psmove_goto_if_fail(strcmp(s_filter_do_2d_xy, "filter_do_2d_xy") == 0, finish);

    char s_filter_do_2d_r[64];
    result = fscanf(fp, "%18s,%d\n", &s_filter_do_2d_r, &smoothing_settings.filter_do_2d_r);
	psmove_goto_if_fail(result == 2, finish);
    psmove_goto_if_fail(strcmp(s_filter_do_2d_r, "filter_do_2d_r") == 0, finish);

    char s_filter_3d_type[64];
    result = fscanf(fp, "%18s,%d\n", &s_filter_3d_type, &smoothing_settings.filter_3d_type);
    psmove_goto_if_fail(result == 2, finish);
    psmove_goto_if_fail(strcmp(s_filter_3d_type, "filter_3d_type") == 0, finish);

	char s_acceleration_variance[64];
	result = fscanf(fp, "%21s,%f\n", &s_acceleration_variance, &smoothing_settings.acceleration_variance);
	psmove_goto_if_fail(result == 2, finish);
	psmove_goto_if_fail(strcmp(s_acceleration_variance, "acceleration_variance") == 0, finish);

	char s_measurement_covariance_00[64];
	result = fscanf(fp, "%25s,%f\n", &s_measurement_covariance_00, &smoothing_settings.measurement_covariance.row0[0]);
	psmove_goto_if_fail(result == 2, finish);
	psmove_goto_if_fail(strcmp(s_measurement_covariance_00, "measurement_covariance_00") == 0, finish);

	char s_measurement_covariance_11[64];
	result = fscanf(fp, "%25s,%f\n", &s_measurement_covariance_11, &smoothing_settings.measurement_covariance.row1[1]);
	psmove_goto_if_fail(result == 2, finish);
	psmove_goto_if_fail(strcmp(s_measurement_covariance_11, "measurement_covariance_11") == 0, finish);

	char s_measurement_covariance_22[64];
	result = fscanf(fp, "%25s,%f\n", &s_measurement_covariance_22, &smoothing_settings.measurement_covariance.row2[2]);
	psmove_goto_if_fail(result == 2, finish);
	psmove_goto_if_fail(strcmp(s_measurement_covariance_22, "measurement_covariance_22") == 0, finish);

	success = PSMove_True;

finish:
	if (success == PSMove_True)
	{
		*out_smoothing_settings = smoothing_settings;
	}

    psmove_file_close(fp);
	return success;
}

void
psmove_tracker_set_smoothing_type(PSMoveTracker *tracker, enum PSMoveTracker_Smoothing_Type smoothing_type)
{
	psmove_return_if_fail(tracker != NULL);

	if (smoothing_type != tracker->smoothing_type)
	{

		int controller_index;
		for (controller_index = 0; controller_index < PSMOVE_TRACKER_MAX_CONTROLLERS; ++controller_index)
		{
			TrackedController *controller= &tracker->controllers[controller_index];

			// Free the old filter, if any
			if (controller->position_filter != NULL)
			{
				switch (tracker->smoothing_type)
				{
				case Smoothing_None:
					// Nothing to clean up 
					break;
				case Smoothing_LowPass:
					psmove_position_lowpass_filter_free((PSMovePositionLowPassFilter *)controller->position_filter);
					break;
				case Smoothing_Kalman:
					psmove_position_kalman_filter_free((PSMovePositionKalmanFilter *)controller->position_filter);
					break;
				}
			}

			// Create the new filter, if any
			switch (smoothing_type)
			{
			case Smoothing_None:
				// Nothing to clean up 
				break;
			case Smoothing_LowPass:
				controller->position_filter = psmove_position_lowpass_filter_new();
				break;
			case Smoothing_Kalman:
				controller->position_filter = psmove_position_kalman_filter_new();
				break;
			}
		}

		tracker->smoothing_type = smoothing_type;
	}
}

void
psmove_tracker_set_smoothing_2d(PSMoveTracker *tracker, int filter_do_2d_xy, int filter_do_2d_r)
{
	psmove_return_if_fail(tracker != NULL);

    tracker->smoothing_settings.filter_do_2d_xy = filter_do_2d_xy > 0;
    tracker->smoothing_settings.filter_do_2d_r = filter_do_2d_r > 0;
}

void
psmove_tracker_enable_deinterlace(PSMoveTracker *tracker,
        enum PSMove_Bool enabled)
{
    psmove_return_if_fail(tracker != NULL);
    psmove_return_if_fail(tracker->cc != NULL);

    camera_control_set_deinterlace(tracker->cc, enabled);
}

void
psmove_tracker_set_mirror(PSMoveTracker *tracker,
        enum PSMove_Bool enabled)
{
    psmove_return_if_fail(tracker != NULL);

    tracker->settings.camera_mirror = enabled;
}

enum PSMove_Bool
psmove_tracker_get_mirror(PSMoveTracker *tracker)
{
    psmove_return_val_if_fail(tracker != NULL, PSMove_False);

    return tracker->settings.camera_mirror;
}

PSMoveTracker *
psmove_tracker_new_with_camera(int camera) {
    PSMoveTrackerSettings settings;
    psmove_tracker_settings_set_default(&settings);
    return psmove_tracker_new_with_camera_and_settings(camera, &settings);
}

PSMoveTracker *
psmove_tracker_new_with_camera_and_settings(int camera, PSMoveTrackerSettings *settings) {

    PSMoveTracker* tracker = (PSMoveTracker*) calloc(1, sizeof(PSMoveTracker));
    tracker->settings = *settings;
    tracker->rHSV = cvScalar(
        tracker->settings.color_hue_filter_range,
        tracker->settings.color_saturation_filter_range,
        tracker->settings.color_value_filter_range, 0);
    tracker->storage = cvCreateMemStorage(0);

	if (psmove_tracker_load_smoothing_settings(&(tracker->smoothing_settings)) == PSMove_False)
	{
		psmove_tracker_smoothing_settings_set_default(&(tracker->smoothing_settings));
	}

    psmove_tracker_set_smoothing_type(tracker, tracker->smoothing_settings.filter_3d_type);

#if defined(__APPLE__) && !defined(CAMERA_CONTROL_USE_PS3EYE_DRIVER)
    // Assume iSight. Calibration will be done with with sphere against camera
    // to avoid auto-balancing due to background light
    PSMove *move = psmove_connect();
    psmove_set_leds(move, 255, 255, 255);
    psmove_update_leds(move);

    printf("Cover the iSight camera with the sphere and press the Move button\n");
    _psmove_wait_for_button(move, Btn_MOVE);
    psmove_set_leds(move, 0, 0, 0);
    psmove_update_leds(move);
    psmove_set_leds(move, 255, 255, 255);
    psmove_update_leds(move);
#endif

	// start the video capture device for tracking

    // Returns NULL if no control found.
    // e.g. PS3EYE set during compile but not plugged in.
    tracker->cc = camera_control_new_with_settings(camera,
        tracker->settings.camera_frame_width, tracker->settings.camera_frame_height, tracker->settings.camera_frame_rate);
    if (!tracker->cc)
    {
        free(tracker);
        return NULL;
    }
    else { psmove_DEBUG("Successfully initialized camera_control.\n"); }

    psmove_tracker_load_distortion(tracker);

    // backup the systems settings, if not already backuped
    char *filename = psmove_util_get_file_path(PSEYE_BACKUP_FILE);
    camera_control_backup_system_settings(tracker->cc, filename);
    free(filename);

#if !defined(__APPLE__) || defined(CAMERA_CONTROL_USE_PS3EYE_DRIVER)
    // try to load color mapping data (not on Mac OS X for now, because the
    // automatic white balance means we get different colors every time)
    filename = psmove_util_get_file_path(COLOR_MAPPING_DAT);
    FILE *fp = NULL;
    time_t now = time(NULL);
    struct stat st;
    memset(&st, 0, sizeof(st));

    if (stat(filename, &st) == 0 && now != (time_t)-1) {
        if (st.st_mtime >= (now - settings->color_mapping_max_age)) {
            fp = psmove_file_open(filename, "rb");
        } else {
            printf("%s is too old - not restoring colors.\n", filename);
        }
    }

    if (fp) {
        if (!fread(&(tracker->color_mapping),
                    sizeof(struct ColorMappingRingBuffer),
                    1, fp)) {
            psmove_WARNING("Cannot read data from: %s\n", filename);
        } else {
            printf("color mappings restored.\n");
        }

        psmove_file_close(fp);
    }
    free(filename);
#endif

    // Default to the distance parameters for the PS Eye camera
    tracker->distance_parameters = pseye_distance_parameters;

    // Set the exposure to a constant based on a target luminance.
    psmove_DEBUG("Setting exposure: %d\n", tracker->settings.exposure_mode);
    psmove_tracker_set_exposure(tracker, tracker->settings.exposure_mode);

    // just query a frame so that we know the camera works
    IplImage* frame = NULL;
    enum PSMove_Bool new_frame = PSMove_False;
    while (!frame || new_frame == PSMove_False) {
        frame = camera_control_query_frame(tracker->cc, NULL, NULL, &new_frame);
    }

    // prepare ROI data structures

    /* Define the size of the biggest ROI */
    int size = psmove_util_get_env_int(PSMOVE_TRACKER_ROI_SIZE_ENV);

    if (size == -1) {
        size = MIN(frame->width, frame->height) / 2;
    } else {
        psmove_DEBUG("Using ROI size: %d\n", size);
    }

    int w = size, h = size;

    // We need to grab an image from the camera to determine the frame size
    psmove_tracker_update_image(tracker);

    tracker->settings.search_tile_width = w;
    tracker->settings.search_tile_height = h;

    tracker->settings.search_tiles_horizontal = (tracker->frame->width +
        tracker->settings.search_tile_width - 1) / tracker->settings.search_tile_width;
    int search_tiles_vertical = (tracker->frame->height +
        tracker->settings.search_tile_height - 1) / tracker->settings.search_tile_height;

    tracker->settings.search_tiles_count = tracker->settings.search_tiles_horizontal *
        search_tiles_vertical;

    if (tracker->settings.search_tiles_count % 2 == 0) {
        /**
         * search_tiles_count must be uneven, so that when picking every second
         * tile, we still "visit" every tile after two scans when we wrap:
         *
         *  ABA
         *  BAB
         *  ABA -> OK, first run = A, second run = B
         *
         *  ABAB
         *  ABAB -> NOT OK, first run = A, second run = A
         *
         * Incrementing the count will make the algorithm visit the lower right
         * item twice, but will then cause the second run to visit 'B's.
         *
         * We pick every second tile, so that we only need half the time to
         * sweep through the whole image (which usually means faster recovery).
         **/
        tracker->settings.search_tiles_count++;
    }

    // Allocate memory for images of each possible ROI size, in colour and grayscale
	int i;
	for (i = 0; i < ROIS; i++) {
		tracker->roiI[i] = cvCreateImage(cvSize(w,h), frame->depth, 3);  // colour
		tracker->roiM[i] = cvCreateImage(cvSize(w,h), frame->depth, 1);  // grayscale

		/* Smaller rois are always square, and 70% of the previous level */
		h = w = ((MIN(w,h) * 70) / 100);
	}

	// prepare structure used for erode and dilate in calibration process
	int ks = 5; // Kernel Size
	int kc = (ks + 1) / 2; // Kernel Center
	tracker->kCalib = cvCreateStructuringElementEx(ks, ks, kc, kc, CV_SHAPE_RECT, NULL);

#if defined(__APPLE__) && !defined(CAMERA_CONTROL_USE_PS3EYE_DRIVER)
    printf("Move the controller away and press the Move button\n");
    _psmove_wait_for_button(move, Btn_MOVE);
    psmove_set_leds(move, 0, 0, 0);
    psmove_update_leds(move);
    psmove_disconnect(move);
#endif

	return tracker;
}

void
psmove_tracker_load_distortion(PSMoveTracker *tracker)
{
    // Load the camera-calibration file from disk. Sets cc->mapx, mapy, focl_x, focl_y
    char *intrinsics_xml = psmove_util_get_file_path(INTRINSICS_XML);
    char *distortion_xml = psmove_util_get_file_path(DISTORTION_XML);
    camera_control_read_calibration(tracker->cc, intrinsics_xml, distortion_xml);
    free(intrinsics_xml);
    free(distortion_xml);
}

void
psmove_tracker_reset_distortion(PSMoveTracker *tracker)
{
    camera_control_reset_calibration(tracker->cc);
}


enum PSMoveTracker_Status
psmove_tracker_enable(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_val_if_fail(tracker != NULL, Tracker_CALIBRATION_ERROR);
    psmove_return_val_if_fail(move != NULL, Tracker_CALIBRATION_ERROR);

    // Switch off the controller and all others while enabling another one
    TrackedController *tc;
    for_each_controller(tracker, tc) {
        psmove_set_leds(tc->move, 0, 0, 0);
        psmove_update_leds(tc->move);
    }
    psmove_set_leds(move, 0, 0, 0);
    psmove_update_leds(move);

    int i;
    for (i=0; i<ARRAY_LENGTH(preset_colors); i++) {
        if (!psmove_tracker_color_is_used(tracker, preset_colors[i])) {
            return psmove_tracker_enable_with_color_internal(tracker,
                    move, preset_colors[i]);
        }
    }

    /* No colors are available anymore */
    return Tracker_CALIBRATION_ERROR;
}

int
psmove_tracker_old_color_is_tracked(PSMoveTracker* tracker, PSMove* move, struct PSMove_RGBValue rgb)
{
    CvScalar color;

    if (!psmove_tracker_lookup_color(tracker, rgb, &color)) {
        return 0;
    }

    TrackedController *tc = psmove_tracker_find_controller(tracker, NULL);

    if (!tc) {
        return 0;
    }

    tc->move = move;
    tc->color = rgb;
    tc->auto_update_leds = PSMove_True;

    tc->eColor = tc->eFColor = color;
    tc->eColorHSV = tc->eFColorHSV = th_brg2hsv(tc->eFColor);

    /* Try to track the controller, give up after 100 iterations */
    int i;
    for (i=0; i<100; i++) {
        psmove_set_leds(move,
            (unsigned char)(rgb.r * tracker->settings.dimming_factor),
            (unsigned char)(rgb.g * tracker->settings.dimming_factor),
            (unsigned char)(rgb.b * tracker->settings.dimming_factor));
        psmove_update_leds(move);
        psmove_usleep(1000 * 10); // wait 10ms - ok, since we're not blinking
        psmove_tracker_update_image(tracker);
        psmove_tracker_update(tracker, move);

        if (tc->is_tracked) {
            // TODO: Verify quality criteria to avoid bogus tracking
            return 1;
        }
    }

    psmove_tracker_disable(tracker, move);
    return 0;
}

int
psmove_tracker_lookup_color(PSMoveTracker *tracker, struct PSMove_RGBValue rgb, CvScalar *color)
{
    unsigned char current = tracker->color_mapping.next_slot - 1;
    unsigned char dimming = (unsigned char)(255 * tracker->settings.dimming_factor);

    while (current != tracker->color_mapping.next_slot) {
        if (memcmp(&rgb, &(tracker->color_mapping.map[current].from),
                    sizeof(struct PSMove_RGBValue)) == 0 &&
                tracker->color_mapping.map[current].dimming == dimming) {
            struct PSMove_RGBValue to = tracker->color_mapping.map[current].to;
            color->val[0] = to.r;
            color->val[1] = to.g;
            color->val[2] = to.b;
            return 1;
        }

        current--;
    }

    return 0;
}

void
psmove_tracker_remember_color(PSMoveTracker *tracker, struct PSMove_RGBValue rgb, CvScalar color)
{
    unsigned char dimming = 255 * tracker->settings.dimming_factor;

    struct PSMove_RGBValue to;
    to.r = (unsigned char)(color.val[0]);
    to.g = (unsigned char)(color.val[1]);
    to.b = (unsigned char)(color.val[2]);

    unsigned char slot = tracker->color_mapping.next_slot++;
    tracker->color_mapping.map[slot].from = rgb;
    tracker->color_mapping.map[slot].dimming = dimming;
    tracker->color_mapping.map[slot].to = to;

    char *filename = psmove_util_get_file_path(COLOR_MAPPING_DAT);
    FILE *fp = psmove_file_open(filename, "wb");
    if (fp) {
        if (!fwrite(&(tracker->color_mapping),
                    sizeof(struct ColorMappingRingBuffer),
                    1, fp)) {
            psmove_WARNING("Cannot write data to: %s\n", filename);
        } else {
            printf("color mappings saved.\n");
        }

        psmove_file_close(fp);
    }
    free(filename);
}

enum PSMoveTracker_Status
psmove_tracker_enable_with_color(PSMoveTracker *tracker, PSMove *move,
        unsigned char r, unsigned char g, unsigned char b)
{
    psmove_return_val_if_fail(tracker != NULL, Tracker_CALIBRATION_ERROR);
    psmove_return_val_if_fail(move != NULL, Tracker_CALIBRATION_ERROR);

    struct PSMove_RGBValue rgb = { r, g, b };
    return psmove_tracker_enable_with_color_internal(tracker, move, rgb);
}

enum PSMove_Bool
psmove_tracker_blinking_calibration(PSMoveTracker *tracker, PSMove *move,
        struct PSMove_RGBValue rgb, CvScalar *color, CvScalar *hsv_color)
{
    char *color_str = psmove_util_get_env_string(PSMOVE_TRACKER_COLOR_ENV);
    if (color_str != NULL) {
        int r, g, b;
        if (sscanf(color_str, "%02x%02x%02x", &r, &g, &b) == 3) {
            printf("r: %d, g: %d, b: %d\n", r, g, b);
            *color = cvScalar(r, g, b, 0);
            *hsv_color = th_brg2hsv(*color);
            return PSMove_True;
        } else {
            psmove_WARNING("Cannot parse color: '%s'\n", color_str);
        }
        free(color_str);
    }

    psmove_tracker_update_image(tracker);
    IplImage* frame = tracker->frame;
    assert(frame != NULL);

    // Switch off all other controllers for better measurements
    TrackedController *tc;
    for_each_controller(tracker, tc) {
        psmove_set_leds(tc->move, 0, 0, 0);
        psmove_update_leds(tc->move);
    }

    // clear the calibration html trace
    psmove_html_trace_clear();

    IplImage *mask = NULL;
    IplImage *images[BLINKS]; // array of images saved during calibration for estimation of sphere color
    IplImage *diffs[BLINKS]; // array of masks saved during calibration for estimation of sphere color
    CvSize image_size;
    image_size.width = frame->width;
    image_size.height = frame->height;
    int i;
    for (i = 0; i < BLINKS; i++) {
        // allocate the images
        images[i] = cvCreateImage(image_size, frame->depth, 3);
        diffs[i] = cvCreateImage(image_size, frame->depth, 1);
    }
    double sizes[BLINKS]; // array of blob sizes saved during calibration for estimation of sphere color
    float sizeBest = 0;
    CvSeq *contourBest = NULL;
    float lastSat;
    float lastDimming = -1;

    // DEBUG log the assigned color
    psmove_html_trace_put_color_var("assignedColor", cvScalar(rgb.b, rgb.g, rgb.r, 0));
    
    float dimming = 1.0;
    
    // If previously set, use that
    if (tracker->settings.dimming_factor > 0) {
        dimming = tracker->settings.dimming_factor;
    }
    
    while (1) {
        for (i = 0; i < BLINKS; i++) {
            // create a diff image
            psmove_tracker_get_diff(tracker, move, rgb, images[i], diffs[i], tracker->settings.calibration_blink_delay, dimming);

            // DEBUG log the diff image and the image with the lit sphere
            psmove_html_trace_image_at(images[i], i, "originals");
            psmove_html_trace_image_at(diffs[i], i, "rawdiffs");

            // threshold it to reduce image noise
            cvThreshold(diffs[i], diffs[i], tracker->settings.calibration_diff_t, 0xFF /* white */, CV_THRESH_BINARY);

            // DEBUG log the thresholded diff image
            psmove_html_trace_image_at(diffs[i], i, "threshdiffs");

            // use morphological operations to further remove noise
            cvErode(diffs[i], diffs[i], tracker->kCalib, 1);
            cvDilate(diffs[i], diffs[i], tracker->kCalib, 1);

            // DEBUG log the even more cleaned up diff-image
            psmove_html_trace_image_at(diffs[i], i, "erodediffs");
        }

        // put the diff images together to get hopefully only one intersection region
        // the region at which the controllers sphere resides.
        mask = diffs[0];
        for (i=1; i<BLINKS; i++) {
            cvAnd(mask, diffs[i], mask, NULL);
        }

        // find the biggest contour and repaint the blob where the sphere is expected
        //psmove_tracker_biggest_contour(diffs[0], tracker->storage, &contourBest, &sizeBest);
        psmove_tracker_biggest_contour(mask, tracker->storage, &contourBest, &sizeBest);
        cvSet(mask, TH_COLOR_BLACK, NULL);
        if (contourBest) {
            cvDrawContours(mask, contourBest, TH_COLOR_WHITE, TH_COLOR_WHITE, -1, CV_FILLED, 8, cvPoint(0, 0));
        }
        cvClearMemStorage(tracker->storage);

        // DEBUG log the final mask used for color estimation
        psmove_html_trace_image_at(mask, 0, "finaldiff");

        // CHECK if the blob contains a minimum number of pixels
        if (cvCountNonZero(mask) < tracker->settings.calibration_min_size) {
            psmove_html_trace_put_log_entry("WARNING", "The final mask my not be representative for color estimation.");
        }

        // calculate the average color from the first image
        *color = cvAvg(images[0], mask);
        *hsv_color = th_brg2hsv(*color);
        psmove_DEBUG("Dimming: %.2f, H: %.2f, S: %.2f, V: %.2f\n", dimming,
                hsv_color->val[0], hsv_color->val[1], hsv_color->val[2]);
        
        if (lastDimming >= 0                    // We have a previous dimming
            && (hsv_color->val[1] < lastSat)    // Now less saturated than last
            && hsv_color->val[1] > 32)          // Above bare minimum
        {
            tracker->settings.dimming_factor = lastDimming;
            break;
        }
        
        lastDimming = dimming;
        lastSat = hsv_color->val[1];
        
        if (tracker->settings.dimming_factor == 0.) {  // If not previously set
            if (hsv_color->val[1] > 128) {        // If sat > sat_thresh
                tracker->settings.dimming_factor = dimming;    // save and break
                break;
            } else if (dimming < 0.01) {          // If at minimum dimming
                break;                                // break, no save
            }
        } else {
            break;
        }

        dimming *= 0.3f;
    }

    psmove_html_trace_put_color_var("estimatedColor", *color);
    psmove_html_trace_put_int_var("estimated_hue", (int)((*hsv_color).val[0]));

    int valid_countours = 0;

    // calculate upper & lower bounds for the color filter
    CvScalar min = th_scalar_sub(*hsv_color, tracker->rHSV);
    CvScalar max = th_scalar_add(*hsv_color, tracker->rHSV);
    min.val[0] = MAX(min.val[0], 0);
    min.val[1] = MAX(min.val[1], 0);
    min.val[2] = MAX(min.val[2], 0);
    max.val[0] = MIN(max.val[0], 180);
    max.val[1] = MIN(max.val[1], 255);
    max.val[2] = MIN(max.val[2], 255);
    psmove_DEBUG("filter min: H: %.2f, S: %.2f, V: %.2f\n",
                 min.val[0], min.val[1], min.val[2]);
    psmove_DEBUG("filter max: H: %.2f, S: %.2f, V: %.2f\n",
                 max.val[0], max.val[1], max.val[2]);

    CvPoint firstPosition;
    for (i=0; i<BLINKS; i++) {
        // Convert to HSV, then apply the color range filter to the mask
        cvCvtColor(images[i], images[i], CV_BGR2HSV);
        cvInRangeS(images[i], min, max, mask);

        // use morphological operations to further remove noise
        cvErode(mask, mask, tracker->kCalib, 1);
        cvDilate(mask, mask, tracker->kCalib, 1);

        // DEBUG log the color filter and
        psmove_html_trace_image_at(mask, i, "filtered");

        // find the biggest contour in the image and save its location and size
        psmove_tracker_biggest_contour(mask, tracker->storage, &contourBest, &sizeBest);
        sizes[i] = 0;
        float dist = FLT_MAX;
        CvRect bBox;
        if (contourBest) {
            bBox = cvBoundingRect(contourBest, 0);
            if (i == 0) {
                firstPosition = cvPoint(bBox.x, bBox.y);
            }
            dist = (float)sqrt(pow(firstPosition.x - bBox.x, 2) + pow(firstPosition.y - bBox.y, 2));
            sizes[i] = sizeBest;
        }

        // CHECK for errors (no contour, more than one contour, or contour too small)
        if (!contourBest) {
            psmove_html_trace_array_item_at(i, "contours", "no contour");
        }
        else if (sizes[i] <= tracker->settings.calibration_min_size) {
            psmove_html_trace_array_item_at(i, "contours", "too small");
        }
        else if (dist >= tracker->settings.calibration_max_distance) {
            psmove_html_trace_array_item_at(i, "contours", "too far apart");
        } else {
            psmove_html_trace_array_item_at(i, "contours", "OK");
            // all checks passed, increase the number of valid contours
            valid_countours++;
        }
        cvClearMemStorage(tracker->storage);

    }

    // clean up all temporary images
    for (i=0; i<BLINKS; i++) {
        cvReleaseImage(&images[i]);
        cvReleaseImage(&diffs[i]);
    }

    // CHECK if sphere was found in each BLINK image
    if (valid_countours < BLINKS) {
        psmove_html_trace_put_log_entry("ERROR", "The sphere could not be found in all images.");
        return PSMove_False;
    }

    // CHECK if the size of the found contours are similar
    double sizeVariance, sizeAverage;
    th_stats(sizes, BLINKS, &sizeVariance, &sizeAverage);
    if (sqrt(sizeVariance) >= (sizeAverage / 100.0 * tracker->settings.calibration_size_std)) {
        psmove_html_trace_put_log_entry("ERROR", "The spheres found differ too much in size.");
        return PSMove_False;
    }

    return PSMove_True;
}


enum PSMoveTracker_Status
psmove_tracker_enable_with_color_internal(PSMoveTracker *tracker, PSMove *move,
        struct PSMove_RGBValue rgb)
{
    // check if the controller is already enabled!
    if (psmove_tracker_find_controller(tracker, move)) {
        return Tracker_CALIBRATED;
    }

    // cannot use the same color for two different controllers
    if (psmove_tracker_color_is_used(tracker, rgb)) {
        return Tracker_CALIBRATION_ERROR;
    }

    // try to track the controller with the old color, if it works we are done
    if (psmove_tracker_old_color_is_tracked(tracker, move, rgb)) {
        return Tracker_CALIBRATED;
    }

    CvScalar color;
    CvScalar hsv_color;
    if (psmove_tracker_blinking_calibration(tracker, move, rgb, &color, &hsv_color)) {
        psmove_DEBUG("Result of calib: r %d, g %d, b %d, h %f, s %f, v %f\n",
            rgb.r, rgb.g, rgb.b, hsv_color.val[0], hsv_color.val[1], hsv_color.val[2]);
        // Find the next free slot to use as TrackedController
        TrackedController *tc = psmove_tracker_find_controller(tracker, NULL);

        if (tc != NULL) {
            tc->move = move;
            tc->color = rgb;
            tc->auto_update_leds = PSMove_True;

            psmove_tracker_remember_color(tracker, rgb, color);
            tc->eColor = tc->eFColor = color;
            tc->eColorHSV = tc->eFColorHSV = hsv_color;
            psmove_DEBUG("Stored color: h %f, s %f, v %f\n",
                tc->eColorHSV.val[0], tc->eColorHSV.val[1], tc->eColorHSV.val[2]);

            tc->position_offset = psmove_3axisvector_xyz(tracker->settings.xorigin_cm, tracker->settings.yorigin_cm, tracker->settings.zorigin_cm);

            return Tracker_CALIBRATED;
        }
    }

    return Tracker_CALIBRATION_ERROR;
}

int
psmove_tracker_get_color(PSMoveTracker *tracker, PSMove *move,
        unsigned char *r, unsigned char *g, unsigned char *b)
{
    psmove_return_val_if_fail(tracker != NULL, 0);
    psmove_return_val_if_fail(move != NULL, 0);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);

    if (tc) {
        *r = (unsigned char)(tc->color.r * tracker->settings.dimming_factor);
        *g = (unsigned char)(tc->color.g * tracker->settings.dimming_factor);
        *b = (unsigned char)(tc->color.b * tracker->settings.dimming_factor);

        return 1;
    }

    return 0;
}

int
psmove_tracker_get_camera_color(PSMoveTracker *tracker, PSMove *move,
        unsigned char *r, unsigned char *g, unsigned char *b)
{
    psmove_return_val_if_fail(tracker != NULL, 0);
    psmove_return_val_if_fail(move != NULL, 0);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);

    if (tc) {
        *r = (unsigned char)(tc->eColor.val[0]);
        *g = (unsigned char)(tc->eColor.val[1]);
        *b = (unsigned char)(tc->eColor.val[2]);

        return 1;
    }

    return 0;
}

int
psmove_tracker_set_camera_color(PSMoveTracker *tracker, PSMove *move,
        unsigned char r, unsigned char g, unsigned char b)
{
    psmove_return_val_if_fail(tracker != NULL, 0);
    psmove_return_val_if_fail(move != NULL, 0);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);

    if (tc) {
        /* Update the current color */
        tc->eColor.val[0] = r;
        tc->eColor.val[1] = g;
        tc->eColor.val[2] = b;
        tc->eColorHSV = th_brg2hsv(tc->eColor);

        /* Update the "first" color (to avoid re-adaption to old color) */
        tc->eFColor = tc->eColor;
        tc->eFColorHSV = tc->eColorHSV;

        return 1;
    }

    return 0;
}

int
psmove_tracker_cycle_color(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_val_if_fail(tracker != NULL, 0);
    psmove_return_val_if_fail(move != NULL, 0);
    int next_color_index = 0;
    struct PSMove_RGBValue rgb;

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);
    if (tc) {
        rgb.r = (unsigned char)(tc->color.r);
        rgb.g = (unsigned char)(tc->color.g);
        rgb.b = (unsigned char)(tc->color.b);

        int i, next_ind;
        for (i = 0; i < N_PRESET_COLORS; i++) {
            next_ind = (i + 1) % N_PRESET_COLORS;
            if (preset_colors[i].r == rgb.r &&
                preset_colors[i].g == rgb.g &&
                preset_colors[i].b == rgb.b &&
                !psmove_tracker_color_is_used(tracker, preset_colors[next_ind]))
            {
                next_color_index = next_ind;
                break;
            }
        }

        psmove_tracker_set_dimming(tracker, 0.0);  // Set dimming to 0 to trigger blinking calibration.
        psmove_set_leds(move, 0, 0, 0);         // Turn off the LED to make sure it isn't trackable until new colour set.
        psmove_update_leds(move);

        CvScalar color;
        CvScalar hsv_color;
        if (psmove_tracker_blinking_calibration(tracker, move, preset_colors[next_color_index], &color, &hsv_color))
        {
            tc->move = move;
            tc->color = preset_colors[next_color_index];
            tc->auto_update_leds = PSMove_True;
            psmove_tracker_remember_color(tracker, preset_colors[next_color_index], color);
            tc->eColor = tc->eFColor = color;
            tc->eColorHSV = tc->eFColorHSV = hsv_color;
        }
        return 1;
    }
    return 0;
}


void
psmove_tracker_disable(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_if_fail(tracker != NULL);
    psmove_return_if_fail(move != NULL);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);

    if (tc) {
        // Clear the tracked controller state - also sets move = NULL
        memset(tc, 0, sizeof(TrackedController));

        // XXX: If we "defrag" tracker->controllers to avoid holes with NULL
        // controllers, we can simplify psmove_tracker_find_controller() and
        // abort search at the first encounter of a NULL controller
    }
}

enum PSMoveTracker_Status
psmove_tracker_get_status(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_val_if_fail(tracker != NULL, Tracker_CALIBRATION_ERROR);
    psmove_return_val_if_fail(move != NULL, Tracker_CALIBRATION_ERROR);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);

    if (tc) {
        if (tc->is_tracked) {
            return Tracker_TRACKING;
        } else {
            return Tracker_CALIBRATED;
        }
    }

    return Tracker_NOT_CALIBRATED;
}

void*
psmove_tracker_get_frame(PSMoveTracker *tracker) {
	return tracker->frame;
}

PSMoveTrackerRGBImage
psmove_tracker_get_image(PSMoveTracker *tracker)
{
    PSMoveTrackerRGBImage result = { NULL, 0, 0 };

    if (tracker != NULL) {
        result.width = tracker->frame->width;
        result.height = tracker->frame->height;

        if (tracker->frame_rgb == NULL) {
            tracker->frame_rgb = cvCreateImage(cvSize(result.width, result.height),
                    IPL_DEPTH_8U, 3);
        }

        cvCvtColor(tracker->frame, tracker->frame_rgb, CV_BGR2RGB);
        result.data = tracker->frame_rgb->imageData;
    }

    return result;
}

void psmove_tracker_update_image(PSMoveTracker *tracker) {

    enum PSMove_Bool new_frame = PSMove_False;
    psmove_return_if_fail(tracker != NULL);

    tracker->ts_camera_begin = psmove_timestamp();
    tracker->frame = camera_control_query_frame(tracker->cc,
            &(tracker->ts_camera_grab), &(tracker->ts_camera_retrieve), &new_frame);
    if (new_frame == PSMove_True && tracker->settings.camera_mirror) {
        /**
         * Mirror image on the X axis (works for me with the PS Eye on Linux,
         * although the OpenCV docs say the third parameter should be 0 for X
         * axis mirroring)
         *
         * See also:
         * http://cv-kolaric.blogspot.com/2007/07/effects-of-cvflip.html
         **/
        cvFlip(tracker->frame, NULL, 1);
    }
    tracker->ts_camera_converted = psmove_timestamp();
}

int
psmove_tracker_update_controller(PSMoveTracker *tracker, TrackedController *tc)
{
    // Tell the LEDs to keep on keeping on.
    if (tc->auto_update_leds) {
        unsigned char r, g, b;
        psmove_tracker_get_color(tracker, tc->move, &r, &g, &b);
        psmove_set_leds(tc->move, r, g, b);
        psmove_update_leds(tc->move);
    }

    // calculate upper & lower bounds for the color filter
    CvScalar min = th_scalar_sub(tc->eColorHSV, tracker->rHSV);
    CvScalar max = th_scalar_add(tc->eColorHSV, tracker->rHSV);

    // Used for cvMorphologyEx
    //IplConvKernel *convKernel = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT, NULL);

    // Tracking algorithm outline:
    // 1.  Try to find the blob contour in the colour-filtered ROI
    // 2a. The contour is found - Go to 3
    // 2b. The contour is not found
    //          - Then enlarge the ROI and try again. Go to 1
    //          - ROI cannot be enlarged anymore. break.
    // 3.   (the contour is found) Smooth contour (though I find it gives a poor distance estimate)
    // 4.   Re-center ROI on middle of bounding rect of countour and re-size to smallest ROI > 3x bounding rect, not extending past image
    // 5.   Re-get contour with new ROI
    // 6.   Get 2d pixel position, radius, and 3D position
    // 7.   Filter 2D position and do quality checks
    // 8.   Filter 3D position
    // 9.   Colour adaptation

    int sphere_found = 0;
    int roi_recentered = 0;
    int roi_exhausted = 0;
    enum PSMove_Bool contour_is_ok = PSMove_True;

    while (!sphere_found && !roi_exhausted) { // Keep going until sphere_found or roi_exhausted

        // get pointers to image-holders for the given ROI-Level
        IplImage *roi_i = tracker->roiI[tc->roi_level];  // Colour From largest (480x480 at tc->roi_level==0) to smallest (0.7*0.7*0.7*480 at tc->roi_level==3)
        IplImage *roi_m = tracker->roiM[tc->roi_level];  // Grayscale ''

        // Prepare the ROI image
        cvSetImageROI(tracker->frame, cvRect(tc->roi_x, tc->roi_y, roi_i->width, roi_i->height)); // Set the image roi -> limits processing to this region
        cvCvtColor(tracker->frame, roi_i, CV_BGR2HSV); // Convert the ROI colour space in frame's roi, copy result to roi_i
        cvInRangeS(roi_i, min, max, roi_m);  // apply colour filter, copy result to roi_m (grayscale)
        cvSmooth(roi_m, roi_m, CV_GAUSSIAN, 3, 3, 0, 0); // smooth shrinks the blob's found contour, giving the wrong depth.
        //cvMorphologyEx(roi_m, roi_m, NULL, NULL, CV_MOP_CLOSE, 1 ); // Shrinks the blob slightly, also slow.

        // Get the contour in the ROI
        float sizeBest = 0;
        CvSeq* contourBest = NULL;
        psmove_tracker_biggest_contour(roi_m, tracker->storage, &contourBest, &sizeBest);  // get the biggest contour in roi_m

        if (CV_IS_SEQ(contourBest)) {
            if (contourBest->total < 6) {
                psmove_DEBUG("contourBest->total = %d\n", contourBest->total);
                contour_is_ok = PSMove_False;
            }
        }
        else {
            psmove_DEBUG("contourBest is not a CvSeq.\n");
            contour_is_ok = PSMove_False;
        }

        if (contourBest && contour_is_ok == PSMove_True) {
            // We found a contour in our ROI, and we didn't already determine that the contour with this ROI was bad.

            // Quickly evaluate the quality of the contour. We want to guard against garbage data.
            // e.g.,
            // Temporarily picking up a different light source (huge jump in px position)
            // Partial occlusion (eventually this might be relaxed with custom ellipse fitting)
            CvRect br = cvBoundingRect(contourBest, 0); // Contour bounding rectangle
            float rectRatio = (float)MAX(br.width, br.height) / (float)MIN(br.width, br.height);
            float delta_px = 0;
            if (tc->is_tracked){
                float d_x = (tc->roi_x + br.x + br.width / 2) - tc->x;
                float d_y = (tc->roi_y + br.y + br.height / 2) - tc->y;
                delta_px = sqrt(d_x*d_x + d_y*d_y);
            }
            contour_is_ok = (rectRatio >= 1.25 || delta_px >= 400 || (br.width*br.height)<25) ? PSMove_False : PSMove_True;

            if (!roi_recentered && contour_is_ok == PSMove_True) {  //TODO: And only if we have a reasonable FPS
                // Recenter the ROI on the middle of the bounding rectangle, at smallest ROI >= 3x br size, limited by image size.

                // Determine the minimum size of the new ROI
                int min_length = MAX(br.width, br.height) * 3;

                // find a suitable ROI level (smallest ROI with edges bigger than min_length)
                int i = 0;
                for (i = 0; i < ROIS; i++) {
                    if (tracker->roiI[i]->width < min_length || tracker->roiI[i]->height < min_length) {
                        break; // roiI[i] is too small. Try again with larger roi
                    }
                    if (tc->roi_level_fixed) {
                        tc->roi_level = 0;
                    }
                    else {
                        tc->roi_level = i; // roiI[i] is not too small. Use i
                    }
                    // update easy accessors
                    roi_i = tracker->roiI[tc->roi_level];
                    roi_m = tracker->roiM[tc->roi_level];
                }

                // Set the new ROI.
                psmove_tracker_set_roi(tracker, tc, br.x + br.width / 2 + tc->roi_x - roi_i->width / 2, br.y + br.height / 2 + tc->roi_y - roi_i->height / 2, roi_i->width, roi_i->height);

                roi_recentered = 1;

            }
            else if (contour_is_ok == PSMove_True) // ROI already recentered
            {
                /* These next few lines call separate functions to do tracking and filtering. */
                // Get the tc->x, y, r, position_cm
                psmove_tracker_update_controller_position_from_contour(tracker, tc, contourBest);
                // Filter results, if required
                psmove_tracker_filter_2d(tracker, tc, contourBest);  // Also does quality check.
                psmove_tracker_filter_3d(tracker, tc);

                // Adaptive sphere colour filtering
                double time_since_last_color_update = psmove_timestamp_value(psmove_timestamp_diff(tracker->ts_camera_converted, tc->last_color_update));
                if (tracker->settings.color_update_rate > 0 && time_since_last_color_update > (double)tracker->settings.color_update_rate)
                {
                    // Cutout only the tracked contour from our frame.
                    cvSet(roi_m, TH_COLOR_BLACK, NULL);  // Set the whole ROI to black
                    cvDrawContours(roi_m, contourBest, TH_COLOR_WHITE, TH_COLOR_WHITE, -1, CV_FILLED, 8, cvPoint(0, 0)); // Set a white disc.
                    CvScalar newColor = cvAvg(tracker->frame, roi_m);
                    tc->eColor = th_scalar_mul(th_scalar_add(tc->eColor, newColor), 0.5);
                    tc->eColorHSV = th_brg2hsv(tc->eColor);
                    tc->last_color_update = tracker->ts_camera_converted;
                    // CHECK if the current estimate is too far away from its original estimation
                    if (psmove_tracker_hsvcolor_diff(tc) > tracker->settings.color_adaption_quality_t) {
                        tc->eColor = tc->eFColor;
                        tc->eColorHSV = tc->eFColorHSV;
                        sphere_found = 0;
                    }
                }

                sphere_found = 1; // breaks out of while loop
            }
        }
        else if (tc->roi_level>0) {
            // No contour, but we can try a bigger ROI

            // Shift the position by half-window
            tc->roi_x += roi_i->width / 2;
            tc->roi_y += roi_i->height / 2;

            // Decrement roi_level to use a bigger ROI 
            if (tc->roi_level_fixed) {
                tc->roi_level = 0;
            }
            else {
                tc->roi_level = tc->roi_level - 1;
            }

            // update easy accessors
            roi_i = tracker->roiI[tc->roi_level];
            roi_m = tracker->roiM[tc->roi_level];

            // Set the new ROI. It automatically shifts away from edge if ROI is outside the bounds of the camera.
            psmove_tracker_set_roi(tracker, tc, tc->roi_x - roi_i->width / 2, tc->roi_y - roi_i->height / 2, roi_i->width, roi_i->height);
            contour_is_ok = PSMove_True; // Reset because we're going to use a new ROI

        }
        else {  //!(contourBest && contour_is_junk == PSMove_False) && !(tc->roi_level>0)
            // Contour is junk and we are already at the largest ROI

            // What's going on here?
            int rx = tracker->settings.search_tile_width * (tc->search_tile % tracker->settings.search_tiles_horizontal);
            int ry = tracker->settings.search_tile_height * (int)(tc->search_tile / tracker->settings.search_tiles_horizontal);
            tc->search_tile = ((tc->search_tile + 2) % tracker->settings.search_tiles_count);

            tc->roi_level = 0; // Shouldn't be necessary, already at biggest.
            psmove_tracker_set_roi(tracker, tc, rx, ry, tracker->roiI[tc->roi_level]->width, tracker->roiI[tc->roi_level]->height);

            roi_exhausted = 1; // breaks out of while loop
        }
        cvClearMemStorage(tracker->storage);
        cvResetImageROI(tracker->frame);  // Remove ROI from the image
    }

    // remember if the sphere was found
    tc->was_tracked = tc->is_tracked;
    tc->is_tracked = sphere_found && tc->quality_check;
    return tc->is_tracked;
}

void
psmove_tracker_update_controller_position_from_contour(PSMoveTracker *tracker, TrackedController *tc, CvSeq* contourBest)
{
    tc->old_x = tc->x;
    tc->old_y = tc->y;
    tc->old_r = tc->r;

    float a;  // Half the long-axis for the ellipse, or the radius for circle fitting.
    if (tracker->settings.use_fitEllipse)
    {
        // Fit an ellipse to our contour
        CvBox2D ellipse = cvFitEllipse2(contourBest); // TODO: Roll our own with some constraints.
        // cvFitEllipse2 can't work with less than 5 points in the cvSeq, but there's no easy way
        // to count the number of points from a CvSeq**

        // Offset ellipse by our ROI to get in full image coordinates.
        ellipse.center.x += tc->roi_x;
        ellipse.center.y += tc->roi_y;

        // Update pixel positions and other variables used for annotations and quality check.
        tc->x = ellipse.center.x;
        tc->y = ellipse.center.y;
        tc->r = ellipse.size.height / 2; // aka el_minor.
        tc->el_major = ellipse.size.width / 2;
        tc->el_angle = ellipse.angle;

        a = ellipse.size.width / 2;
    }
    else 
    {
        // estimate x/y position and radius of the sphere
        psmove_tracker_estimate_circle_from_contour(contourBest, &tc->x, &tc->y, &tc->r);
        tc->x += tc->roi_x;
        tc->y += tc->roi_y;
        a = tc->r;
    }

    /* 3D positional estimation based on trigonometry of the ellipse/circle
    * See: https://github.com/cboulay/psmove-ue4/wiki/Tracker-Algorithm
    */
    float f_0 = tracker->cc->focl_x;
    float x_i = tc->x - tracker->frame->width / 2;  // x-origin from left to middle.
    float y_i = tracker->frame->height / 2 - tc->y; // y-origin from top to middle, with +y up

    // Calculations
    float L_i = sqrt(x_i*x_i + y_i*y_i);
    float j = (L_i + a) / f_0;
    float k = L_i / f_0;
    float l = (j - k) / (1 + j*k);
    float D_0 = SPHERE_RADIUS_CM * sqrt(1 + l*l) / l;
    float fl = f_0 / L_i; // used several times.
    float z = D_0 * fl / sqrt(1 + fl*fl);
    float L = sqrt(D_0*D_0 - z*z);
    float x = L*x_i / L_i;
    float y = L*y_i / L_i;

    tc->position_cm = psmove_3axisvector_xyz(x, y, z);
}

void
psmove_tracker_filter_2d(PSMoveTracker *tracker, TrackedController *tc, CvSeq* contourBest)
{
    // Restore the contour to roi_m
    IplImage *roi_m = tracker->roiM[tc->roi_level];
    cvSet(roi_m, TH_COLOR_BLACK, NULL);
    cvDrawContours(roi_m, contourBest, TH_COLOR_WHITE, TH_COLOR_WHITE, -1, CV_FILLED, 8, cvPoint(0, 0));
    // Get the mass from the image moments of the restored contour.
    CvMoments mu;
    cvMoments(roi_m, &mu, 0);
    CvPoint com = cvPoint((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));

    CvPoint oldMCenter = cvPoint((int)tc->mx, (int)tc->my);  // Store the previous center of mass.
    tc->mx = (float)(com.x + tc->roi_x);
    tc->my = (float)(com.y + tc->roi_y);
    CvPoint newMCenter = cvPoint((int)tc->mx, (int)tc->my);

    // apply radius-smoothing if enabled
    if (tracker->smoothing_settings.filter_do_2d_r) {
        // calculate the difference between calculated radius and the smoothed radius of the past
        float rDiff = fabsf(tc->r - tc->old_r);
        // calcualte a adaptive smoothing factor
        // a big distance leads to no smoothing, a small one to strong smoothing
        float rf = MIN(rDiff / 4.f + 0.15f, 1);

        // apply adaptive smoothing of the radius
        tc->r = tc->old_r * (1 - rf) + tc->r * rf;
    }

    // tc->q1 is the ratio of the area of the blob to the area of the perfect circle with given radius.
    // If the blob is closed, then this is the same as short_axis / long_axis
    // Higher tc->q1 means more complete and circular blob.
    int pixelInBlob = cvCountNonZero(roi_m);
    float pixelInResult = tc->r * tc->r * (float)M_PI;
    tc->q1 = pixelInBlob / pixelInResult;

    if (tc->q1 > 0.85) {
        // If the blob is approaching a perfect circle, just use the center of mass for x and y.
        tc->x = tc->mx;
        tc->y = tc->my;
    }
    else if (tracker->smoothing_settings.filter_do_2d_xy) {
        // apply x/y coordinate smoothing if enabled
        // a big distance between the old and new center of mass results in no smoothing
        // a little one to strong smoothing
        float diff = sqrtf((float)th_dist_squared(oldMCenter, newMCenter));
        float f = MIN(diff / 7.f + 0.15f, 1);
        // apply adaptive smoothing
        tc->x = tc->old_x * (1 - f) + tc->x * f;
        tc->y = tc->old_y * (1 - f) + tc->y * f;
    }

    tc->q3 = tc->r;
    int radius_shape_and_size_check =
        tc->q1 > tracker->settings.tracker_quality_t1 &&
        tc->q3 > tracker->settings.tracker_quality_t3;

    int radius_delta_check = 1;
    // only perform check if we already found the sphere once
    if (tc->old_r > 0 && tc->search_tile == 0) {
        tc->q2 = fabsf(tc->old_r - tc->r) / (tc->old_r + FLT_EPSILON);
        // additionally check for too big changes
        radius_delta_check = tc->q2 < tracker->settings.tracker_quality_t2;
    }
    else {
        tc->q2 = FLT_MAX;
    }

    tc->quality_check = radius_shape_and_size_check && radius_delta_check;
}

void
psmove_tracker_filter_3d(PSMoveTracker *tracker, TrackedController *tc)
{
    PSMove_3AxisVector measured_position = tc->position_cm;  // Un-filtered version

    // Apply the positional filter to the currently computed position
    if (psmove_3axisvector_is_valid(&measured_position))
    {
        // Compute how long it has been since our last successful position update
        double time_delta = psmove_timestamp_value(psmove_timestamp_diff(tracker->ts_camera_converted, tc->last_position_update));

        // Re-initialize the positional filter if:
        // * It has been too long since the last update
        // * The position wasn't tracked the previous update
        bool reinitialize_filter = time_delta > POSITION_FILTER_RESET_TIME || !tc->was_tracked;

        switch (tracker->smoothing_type)
        {
        case Smoothing_None:	// Don't use any smoothing
        {
            tc->position_cm = measured_position;
        }
        break;
        case Smoothing_LowPass:	// A basic low pass filter (default)
        {
            if (reinitialize_filter)
            {
                psmove_position_lowpass_filter_init(
                    &measured_position, (PSMovePositionLowPassFilter *)tc->position_filter);
            }
            else
            {
                psmove_position_lowpass_filter_update(
                    &tracker->smoothing_settings,
                    &measured_position,
                    (PSMovePositionLowPassFilter *)tc->position_filter);
            }

            tc->position_cm = psmove_position_lowpass_filter_get_position(
                (PSMovePositionLowPassFilter *)tc->position_filter);
        }
        break;
        case Smoothing_Kalman:	// A more expensive Kalman filter 
        {
            if (reinitialize_filter)
            {
                psmove_position_lowpass_filter_init(
                    &measured_position, (PSMovePositionLowPassFilter *)tc->position_filter);
            }
            else
            {
                // TODO: Project accelerometer measurement into camera space
                PSMove_3AxisVector acceleration_control = psmove_3axisvector_xyz(0.f, 0.f, 0.f);

                psmove_position_kalman_filter_update(
                    &tracker->smoothing_settings,
                    &measured_position,
                    &acceleration_control,
                    time_delta,
                    (PSMovePositionKalmanFilter *)tc->position_filter);
            }

            tc->position_cm = psmove_position_kalman_filter_get_position(
                (PSMovePositionKalmanFilter *)tc->position_filter);
        }
        break;
        }

        // Update the position update timestamp
        tc->last_position_update = tracker->ts_camera_converted;
    }
}

int
psmove_tracker_update(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_val_if_fail(tracker->frame != NULL, 0);

    int spheres_found = 0;

    long started = psmove_util_get_ticks();

    TrackedController *tc;
    for_each_controller(tracker, tc) {
        if (move == NULL || tc->move == move) {
            spheres_found += psmove_tracker_update_controller(tracker, tc);
        }
    }

    tracker->duration = psmove_util_get_ticks() - started;

    return spheres_found;
}

int
psmove_tracker_get_position(PSMoveTracker *tracker, PSMove *move,
        float *x, float *y, float *radius)
{
    psmove_return_val_if_fail(tracker != NULL, 0);
    psmove_return_val_if_fail(move != NULL, 0);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);

    if (tc) {
        if (x) {
            *x = tc->x;
        }
        if (y) {
            *y = tc->y;
        }
        if (radius) {
            *radius = tc->r;
        }

        // TODO: return age of tracking values (if possible)
        return 1;
    }

    return 0;
}

int
psmove_tracker_get_location(PSMoveTracker *tracker, PSMove *move, float *xcm, float *ycm, float *zcm)
{
    psmove_return_val_if_fail(tracker != NULL, 0);
    psmove_return_val_if_fail(move != NULL, 0);
    TrackedController *tc = psmove_tracker_find_controller(tracker, move);

    if (tc) 
	{
        if (xcm) 
		{
            *xcm = tc->position_cm.x - tc->position_offset.x;
        }

        if (ycm) 
		{
            *ycm = tc->position_cm.y - tc->position_offset.y;
        }

        if (zcm) 
		{
            *zcm = tc->position_cm.z - tc->position_offset.z;
        }
    
		// TODO: return age of tracking values (if possible)
        return 1;
    }

    return 0;
}

void
psmove_tracker_reset_location(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_if_fail(tracker != NULL);
    psmove_return_if_fail(move != NULL);

    TrackedController *tc = psmove_tracker_find_controller(tracker, move);
    if (tc) 	
	{
		tc->position_offset = tc->position_cm;
    }
}

void
psmove_tracker_get_size(PSMoveTracker *tracker,
        int *width, int *height)
{
    psmove_return_if_fail(tracker != NULL);
    psmove_return_if_fail(tracker->frame != NULL);

    *width = tracker->frame->width;
    *height = tracker->frame->height;
}

void
psmove_tracker_free(PSMoveTracker *tracker)
{
    psmove_return_if_fail(tracker != NULL);

    if (tracker->frame_rgb != NULL) {
        cvReleaseImage(&tracker->frame_rgb);
    }

    char *filename = psmove_util_get_file_path(PSEYE_BACKUP_FILE);
    camera_control_restore_system_settings(tracker->cc, filename);
    free(filename);

    cvReleaseMemStorage(&tracker->storage);

    int i;
    for (i=0; i < ROIS; i++) {
        cvReleaseImage(&tracker->roiM[i]);
        cvReleaseImage(&tracker->roiI[i]);
    }
    cvReleaseStructuringElement(&tracker->kCalib);

    camera_control_delete(tracker->cc);

    PSMoveTrackerSmoothingSettings smoothing_settings;
    psmove_tracker_get_smoothing_settings(tracker, &smoothing_settings);
    psmove_tracker_save_smoothing_settings(&smoothing_settings);
    free(tracker);
}

// -------- Implementation: internal functions only
int
psmove_tracker_adapt_to_light(PSMoveTracker *tracker, float target_luminance)
{
    float minimum_exposure = 0;  // Why 2051 (~3%) on the lower end?
    float maximum_exposure = 65535;
    float current_exposure = minimum_exposure;
    float last_exposure = current_exposure;
    //float current_exposure = (maximum_exposure + minimum_exposure) / 2.;
    float last_saturation = 0.0;
    float next_step = 0;
    float step_size = maximum_exposure - minimum_exposure;
    CvScalar imgColor;
    CvScalar imgHSV;

    /*
    if (target_luminance == 0) {
        return minimum_exposure;
    }
     */
    
    // Switch off the controllers' LEDs for proper environment measurements
    TrackedController *tc;
    for_each_controller(tracker, tc) {
        psmove_set_leds(tc->move, 0, 0, 0);
        psmove_update_leds(tc->move);
    }

    int i;
    for (i=0; i<10; i++) {
        camera_control_set_parameters(tracker->cc, 0, 0, 0,
                (int)current_exposure, 0, 0xffff, 0xffff, 0xffff, -1, -1);

        IplImage* frame;
        psmove_tracker_wait_for_frame(tracker, &frame, 50);
        assert(frame != NULL);

        // calculate the average color and luminance (energy)
        imgColor = cvAvg(frame, NULL);
        imgHSV = th_brg2hsv(imgColor);
        float luminance = (float)th_color_avg(imgColor);

        psmove_DEBUG("Exposure: %.2f, Luminance: %.2f, Saturation: %.2f\n",
                     current_exposure, luminance, imgHSV.val[1]);
        
        /*
        if (fabsf(luminance - target_luminance) < 1) {
            break;
        }
         */

        // Binary search for the best exposure setting
        if ((imgHSV.val[1] / MAX(fabs(luminance - target_luminance),1.0)) > last_saturation)  // Getting better!
        {
            if (current_exposure > last_exposure) // due to increase
            {
                next_step = step_size; // keep increasing
            }
            else  // due to decreae
            {
                next_step = -step_size; // keep decreasing
            }
        }
        else  // Getting worse!
        {
            if (current_exposure > last_exposure) // due to increase
            {
                next_step = -step_size; // try decreasing
            }
            else  // due to decreae
            {
                next_step = step_size; // try increasing
            }
        }
        last_exposure = current_exposure;
        last_saturation = imgHSV.val[1] / MAX(fabs(luminance - target_luminance), 1.0);
        
        // Prepare for next step
        current_exposure += next_step;
        current_exposure = MAX(current_exposure, minimum_exposure);
        current_exposure = MIN(current_exposure, maximum_exposure);
        step_size /= 2.;
    }

    return (int)current_exposure;
}


TrackedController *
psmove_tracker_find_controller(PSMoveTracker *tracker, PSMove *move)
{
    psmove_return_val_if_fail(tracker != NULL, NULL);

    int i;
    for (i=0; i<PSMOVE_TRACKER_MAX_CONTROLLERS; i++) {
        if (tracker->controllers[i].move == move) {
            return &(tracker->controllers[i]);
        }

        // XXX: Assuming a "defragmented" list of controllers, we could stop our
        // search here if we arrive at a controller where move == NULL and admit
        // failure immediately. See the comment in psmove_tracker_disable() for
        // what we would have to do to always keep the list defragmented.
    }

    return NULL;
}

void
psmove_tracker_wait_for_frame(PSMoveTracker *tracker, IplImage **frame, int delay)
{
    int elapsed_time = 0;
    int step = 10;

    while (elapsed_time < delay) {
        enum PSMove_Bool new_frame = PSMove_False;
        psmove_usleep(1000 * step);
        *frame = camera_control_query_frame(tracker->cc, NULL, NULL, &new_frame);
        elapsed_time += step;
    }
}

void psmove_tracker_get_diff(PSMoveTracker* tracker, PSMove* move,
        struct PSMove_RGBValue rgb, IplImage* on, IplImage* diff, int delay,
        float dimming_factor)
{
    // the time to wait for the controller to set the color up
    IplImage* frame;
    // switch the LEDs ON and wait for the sphere to be fully lit
    rgb.r = (unsigned char)(dimming_factor * rgb.r);
    rgb.g = (unsigned char)(dimming_factor * rgb.g);
    rgb.b = (unsigned char)(dimming_factor * rgb.b);
    psmove_set_leds(move, rgb.r, rgb.g, rgb.b);
    psmove_update_leds(move);

    // take the first frame (sphere lit)
    psmove_tracker_wait_for_frame(tracker, &frame, delay);
    cvCopy(frame, on, NULL);

    // switch the LEDs OFF and wait for the sphere to be off
    psmove_set_leds(move, 0, 0, 0);
    psmove_update_leds(move);

    // take the second frame (sphere iff)
    psmove_tracker_wait_for_frame(tracker, &frame, delay);

    // convert both to grayscale images
    IplImage* grey1 = cvCloneImage(diff);
    IplImage* grey2 = cvCloneImage(diff);
    cvCvtColor(frame, grey1, CV_BGR2GRAY);
    cvCvtColor(on, grey2, CV_BGR2GRAY);

    // calculate the diff of to images and save it in "diff"
    cvAbsDiff(grey1, grey2, diff);

    // clean up
    cvReleaseImage(&grey1);
    cvReleaseImage(&grey2);
}

void psmove_tracker_set_roi(PSMoveTracker* tracker, TrackedController* tc, int roi_x, int roi_y, int roi_width, int roi_height) {
	tc->roi_x = roi_x;
	tc->roi_y = roi_y;
	
	if (tc->roi_x < 0)
		tc->roi_x = 0;
	if (tc->roi_y < 0)
		tc->roi_y = 0;

	if (tc->roi_x + roi_width > tracker->frame->width)
		tc->roi_x = tracker->frame->width - roi_width;
	if (tc->roi_y + roi_height > tracker->frame->height)
		tc->roi_y = tracker->frame->height - roi_height;
}

void psmove_tracker_annotate(PSMoveTracker* tracker) {
	CvPoint p;
	IplImage* frame = tracker->frame;

    CvFont fontSmall = cvFont(0.8, 1);
    CvFont fontNormal = cvFont(1, 1);

	char text[256];
	CvScalar c;
	CvScalar avgC;
	float avgLum = 0;
	int roi_w = 0;
	int roi_h = 0;

	// general statistics
	avgC = cvAvg(frame, 0x0);
    avgLum = (float)th_color_avg(avgC);
	cvRectangle(frame, cvPoint(0, 0), cvPoint(frame->width, 25), TH_COLOR_BLACK, CV_FILLED, 8, 0);
	sprintf(text, "fps:%.0f", tracker->debug_fps);
	cvPutText(frame, text, cvPoint(10, 20), &fontNormal, TH_COLOR_WHITE);
    if (tracker->duration) {
        tracker->debug_fps = (0.85f * tracker->debug_fps + 0.15f *
            (1000.f / (float)tracker->duration));
    }
	sprintf(text, "avg(lum):%.0f", avgLum);
	cvPutText(frame, text, cvPoint(255, 20), &fontNormal, TH_COLOR_WHITE);


    // draw all/one controller information to camera image
    TrackedController *tc;
    for_each_controller(tracker, tc) {
        if (tc->is_tracked) {

            // controller specific statistics
            p.x = (int)tc->x;
            p.y = (int)tc->y;

            // Draw the ROI
            roi_w = tracker->roiI[tc->roi_level]->width;
            roi_h = tracker->roiI[tc->roi_level]->height;
            c = tc->eColor;

            cvRectangle(frame, cvPoint(tc->roi_x, tc->roi_y), cvPoint(tc->roi_x + roi_w, tc->roi_y + roi_h), TH_COLOR_WHITE, 3, 8, 0);
            cvRectangle(frame, cvPoint(tc->roi_x, tc->roi_y), cvPoint(tc->roi_x + roi_w, tc->roi_y + roi_h), TH_COLOR_RED, 1, 8, 0);
            cvRectangle(frame, cvPoint(tc->roi_x, tc->roi_y - 45), cvPoint(tc->roi_x + roi_w, tc->roi_y - 5), TH_COLOR_BLACK, CV_FILLED, 8, 0);

            int vOff = 0;
            if (roi_h == frame->height) {
                vOff = roi_h;
            }

            sprintf(text, "RGB:%x,%x,%x", (int)c.val[2], (int)c.val[1], (int)c.val[0]);
            cvPutText(frame, text, cvPoint(tc->roi_x, tc->roi_y + vOff - 5), &fontSmall, c);

            sprintf(text, "ROI:%dx%d", roi_w, roi_h);
            cvPutText(frame, text, cvPoint(tc->roi_x, tc->roi_y + vOff - 15), &fontSmall, c);

            double distance = psmove_tracker_distance_from_radius(tracker, tc->r);

            sprintf(text, "radius: %.2f", tc->r);
            cvPutText(frame, text, cvPoint(tc->roi_x, tc->roi_y + vOff - 35), &fontSmall, c);
            sprintf(text, "dist: %.2f cm", distance);
            cvPutText(frame, text, cvPoint(tc->roi_x, tc->roi_y + vOff - 25), &fontSmall, c);

            sprintf(text, "z: %.2f cm", tc->position_cm.z);
            cvPutText(frame, text, cvPoint(tc->roi_x, tc->roi_y + vOff - 45), &fontSmall, c);

            // Draw the circle
            cvCircle(frame, p, (int)tc->r, TH_COLOR_WHITE, 1, 8, 0);

            // Draw the ellipse
            cvEllipse(frame, cvPoint(tc->x, tc->y), cvSize(tc->el_major, tc->r), tc->el_angle, 0, 360, TH_COLOR_YELLOW, 1, 8, 0);

        } else {
            roi_w = tracker->roiI[tc->roi_level]->width;
            roi_h = tracker->roiI[tc->roi_level]->height;
            cvRectangle(frame, cvPoint(tc->roi_x, tc->roi_y), cvPoint(tc->roi_x + roi_w, tc->roi_y + roi_h), tc->eColor, 3, 8, 0);
        }
    }
}

float
psmove_tracker_hsvcolor_diff(TrackedController* tc) {
    double diff = 0;
    diff += fabs(tc->eFColorHSV.val[0] - tc->eColorHSV.val[0]) * 1.;  // diff of HUE is very important
    diff += fabs(tc->eFColorHSV.val[1] - tc->eColorHSV.val[1]) * 0.5; // saturation and value not so much
    diff += fabs(tc->eFColorHSV.val[2] - tc->eColorHSV.val[2]) * 0.5;
    return (float)diff;
}

void
psmove_tracker_biggest_contour(IplImage* img, CvMemStorage* stor, CvSeq** resContour, float* resSize) {
    CvSeq* contour;
    *resSize = 0;
    *resContour = 0;

    cvFindContours(img, stor, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

    for (; contour; contour = contour->h_next) {
        float f = (float)cvContourArea(contour, CV_WHOLE_SEQ, 0);
        if (f > *resSize) {
            *resSize = f;
            *resContour = contour;
        }
    }
}

void
psmove_tracker_estimate_circle_from_contour(CvSeq* cont, float *x, float *y, float* radius)
{
    psmove_return_if_fail(cont != NULL);
    psmove_return_if_fail(x != NULL && y != NULL && radius != NULL);

    int i, j;
    float d = 0;
    float cd = 0;
    CvPoint m1 = cvPoint( 0, 0 );
    CvPoint m2 = cvPoint( 0, 0 );
    CvPoint * p1;
    CvPoint * p2;
    int found = 0;

	int step = MAX(1,cont->total/20);

	// compare every two points of the contour (but not more than 20)
	// to find the most distant pair
	for (i = 0; i < cont->total; i += step) {
		p1 = (CvPoint*) cvGetSeqElem(cont, i);
		for (j = i + 1; j < cont->total; j += step) {
			p2 = (CvPoint*) cvGetSeqElem(cont, j);
			cd = (float)th_dist_squared(*p1,*p2);
			if (cd > d) {
				d = cd;
				m1 = *p1;
				m2 = *p2;
                                found = 1;
			}
		}
	}
    // calculate center of that pair
    if (found) {
        *x = 0.5f * (float)(m1.x + m2.x);
        *y = 0.5f * (float)(m1.y + m2.y);
    }
    // calcualte the radius
    *radius = sqrtf(d) / 2;
}

int
psmove_tracker_center_roi_on_controller(TrackedController* tc, PSMoveTracker* tracker, CvPoint *center)
{
    psmove_return_val_if_fail(tc != NULL, 0);
    psmove_return_val_if_fail(tracker != NULL, 0);
    psmove_return_val_if_fail(center != NULL, 0);

	CvScalar min = th_scalar_sub(tc->eColorHSV, tracker->rHSV);
        CvScalar max = th_scalar_add(tc->eColorHSV, tracker->rHSV);

	IplImage *roi_i = tracker->roiI[tc->roi_level];
	IplImage *roi_m = tracker->roiM[tc->roi_level];

	// cut out the roi!
	cvSetImageROI(tracker->frame, cvRect(tc->roi_x, tc->roi_y, roi_i->width, roi_i->height));
	cvCvtColor(tracker->frame, roi_i, CV_BGR2HSV);

	// apply color filter
	cvInRangeS(roi_i, min, max, roi_m);
	
	float sizeBest = 0;
	CvSeq* contourBest = NULL;
	psmove_tracker_biggest_contour(roi_m, tracker->storage, &contourBest, &sizeBest);
	if (contourBest) {
		cvSet(roi_m, TH_COLOR_BLACK, NULL);
		cvDrawContours(roi_m, contourBest, TH_COLOR_WHITE, TH_COLOR_WHITE, -1, CV_FILLED, 8, cvPoint(0, 0));
		// calucalte image-moments to estimate the better ROI center
		CvMoments mu;
		cvMoments(roi_m, &mu, 0);

        *center = cvPoint((int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00));
		center->x += tc->roi_x - roi_m->width / 2;
		center->y += tc->roi_y - roi_m->height / 2;
	}
	cvClearMemStorage(tracker->storage);
	cvResetImageROI(tracker->frame);

        return (contourBest != NULL);
}

float
psmove_tracker_distance_from_radius(PSMoveTracker *tracker, float radius)
{
    psmove_return_val_if_fail(tracker != NULL, 0.);

    double height = (double)tracker->distance_parameters.height;
    double center = (double)tracker->distance_parameters.center;
    double hwhm = (double)tracker->distance_parameters.hwhm;
    double shape = (double)tracker->distance_parameters.shape;
    double x = (double)radius;

    /**
     * Pearson type VII distribution
     * http://fityk.nieto.pl/model.html
     **/
    double a = pow((x - center) / hwhm, 2.);
    double b = pow(2., 1. / shape) - 1.;
    double c = 1. + a * b;
    double distance = height / pow(c, shape);

    return (float)distance;
}

void
psmove_tracker_set_distance_parameters(PSMoveTracker *tracker,
        float height, float center, float hwhm, float shape)
{
    psmove_return_if_fail(tracker != NULL);

    tracker->distance_parameters.height = height;
    tracker->distance_parameters.center = center;
    tracker->distance_parameters.hwhm = hwhm;
    tracker->distance_parameters.shape = shape;
}


int
psmove_tracker_color_is_used(PSMoveTracker *tracker, struct PSMove_RGBValue color)
{
    psmove_return_val_if_fail(tracker != NULL, 1);

    TrackedController *tc;
    for_each_controller(tracker, tc) {
        if (memcmp(&tc->color, &color, sizeof(struct PSMove_RGBValue)) == 0) {
            return 1;
        }
    }

    return 0;
}

void
_psmove_tracker_retrieve_stats(PSMoveTracker *tracker,
        PSMove_timestamp *ts_begin, PSMove_timestamp *ts_grab,
        PSMove_timestamp *ts_retrieve, PSMove_timestamp *ts_converted)
{
    psmove_return_if_fail(tracker != NULL);

    *ts_begin = tracker->ts_camera_begin;
    *ts_grab = tracker->ts_camera_grab;
    *ts_retrieve = tracker->ts_camera_retrieve;
    *ts_converted = tracker->ts_camera_converted;
}

void
_psmove_tracker_fix_roi_size(PSMoveTracker *tracker)
{
    psmove_return_if_fail(tracker != NULL);

    TrackedController *tc;
    for_each_controller (tracker, tc) {
        tc->roi_level_fixed = PSMove_True;
    }
}


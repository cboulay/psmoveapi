
 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * Copyright (c) 2015 Chadwick Boulay <chadwick.boulay@gmail.com>
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

#include <time.h>
#include <assert.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#include "psmove.h"
#include "psmove_tracker.h"
#include "psmove_fusion.h"


int main(int arg, char** args) {

    if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
        fprintf(stderr, "PS Move API init failed (wrong version?)\n");
        exit(1);
    }

    int i;
    int count = psmove_count_connected();
    //PSMove* controllers[count];

    printf("### Found %d controllers.\n", count);
    if (count == 0) {
        return 1;
    }

    PSMove **controllers = (PSMove **)calloc(count, sizeof(PSMove *));

    void *frame;
    int result;

    fprintf(stderr, "Trying to init PSMoveTracker...");
    PSMoveTracker* tracker = psmove_tracker_new();
    if (!tracker)
    {
        fprintf(stderr, "Could not init PSMoveTracker.\n");
        return 1;
    }
    psmove_tracker_set_mirror(tracker, PSMove_True);
    fprintf(stderr, "OK\n");

    for (i=0; i<count; i++) {
        printf("Opening controller %d\n", i);
        controllers[i] = psmove_connect_by_id(i);
        assert(controllers[i] != NULL);

        while (1) {
            printf("Calibrating controller %d...", i);
            fflush(stdout);
            if (i == 0 && arg >= 3)
            {
                result = psmove_tracker_enable_with_color(tracker, controllers[i],
                    atoi(args[1]), atoi(args[2]), atoi(args[3]));
                printf("Setting LEDS for controller 1 from command-line r: %i, g: %i, b: %i\n",
                    atoi(args[1]), atoi(args[2]), atoi(args[3]));
            }
            else {
                result = psmove_tracker_enable(tracker, controllers[i]);
            }

            if (result == Tracker_CALIBRATED) {
                enum PSMove_Bool auto_update_leds =
                    psmove_tracker_get_auto_update_leds(tracker,
                            controllers[i]);
                printf("OK, auto_update_leds is %s\n",
                        (auto_update_leds == PSMove_True)?"enabled":"disabled");
                break;
            } else {
                printf("ERROR - retrying\n");
            }
        }
    }

    PSMoveFusion *fusion = psmove_fusion_new(tracker, 1., 1000.);

    // You can also add an additional transform now or at any point
    float add_pos[3] = { 0, 0, 0 };
    float add_quat[4] = { 1, 0, 0, 0 };
    psmove_fusion_update_transform(fusion, add_pos, add_quat);

    // For example, if you have a dk2_camera_pose as follows:
    // (px, py, pz, qw, qx, qy, qz)
    // 1.7896  -13.1017  -86.5323    0.9998    0.0041   -0.0158    0.0108
    add_pos[0] = 1.7896; add_pos[1] = -13.1017; add_pos[2] = -86.5323;
    add_quat[0] = 0.9998; add_quat[1] = 0.0041; add_quat[2] = -0.0158; add_quat[3] = 0.0108;
    psmove_fusion_update_transform(fusion, add_pos, add_quat);

    while ((cvWaitKey(1) & 0xFF) != 27) {
        psmove_tracker_update_image(tracker);
        psmove_tracker_update_cbb(tracker, NULL);
        psmove_tracker_annotate(tracker);

        frame = psmove_tracker_get_frame(tracker);
        if (frame) {
            cvShowImage("live camera feed", frame);
        }

        for (i=0; i<count; i++) {
            /* Optional and not required by default (see auto_update_leds above)
            unsigned char r, g, b;
            psmove_tracker_get_color(tracker, controllers[i], &r, &g, &b);
            psmove_set_leds(controllers[i], r, g, b);
            psmove_update_leds(controllers[i]);
            */

            /* This works but the position in pixels is not so interesting.
            float x, y, r;
            psmove_tracker_get_position(tracker, controllers[i], &x, &y, &r);
            printf("x: %10.2f, y: %10.2f, r: %10.2f\n", x, y, r);
            */
            
            float xcm, ycm, zcm;
            psmove_fusion_get_position(fusion, controllers[i], &xcm, &ycm, &zcm);

            float xf_xcm, xf_ycm, xf_zcm;
            psmove_fusion_get_transformed_position(fusion, controllers[i], &xf_xcm, &xf_ycm, &xf_zcm);
            printf("PSEye: %5.2f, %5.2f, %5.2f\t Transformed: %5.2f, %5.2f, %5.2f\n", xcm, ycm, zcm, xf_xcm, xf_ycm, xf_zcm);
        }
    }

    for (i=0; i<count; i++) {
        psmove_disconnect(controllers[i]);
    }

    psmove_fusion_free(fusion);
    psmove_tracker_free(tracker);
    free(controllers);
    psmove_shutdown();
    return 0;
}


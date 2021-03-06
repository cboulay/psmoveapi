
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

#include <time.h>
#include <assert.h>
#include <math.h>

#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "psmove_examples_opengl.h"

#include "psmove.h"
#include "psmove_tracker.h"
#include "psmove_fusion.h"

enum {
    NOTHING,
    WIRE_CUBE,
    SOLID_CUBE,
    ITEM_MAX,
};

class Point3D {
    public:
        Point3D(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}

        float x;
        float y;
        float z;
};

class Tracker {
    public:
        Tracker();
        ~Tracker();
        void update();

        void init();
        void render();

    private:
        PSMove **m_moves;
        int *m_items;
        int m_count;

        std::vector<Point3D> m_trace;
        float m_rotation;
        Point3D m_offset;
        Point3D m_last_offset;
        bool m_has_last_offset;

        PSMoveTracker *m_tracker;
        PSMoveFusion *m_fusion;
        GLuint m_texture;
};

Tracker::Tracker()
    : m_moves(NULL),
      m_count(0),
      m_tracker(NULL),
      m_fusion(NULL)
{

    if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
        fprintf(stderr, "PS Move API init failed (wrong version?)\n");
        exit(1);
    }

    m_count = psmove_count_connected();

    PSMoveTrackerSettings settings;
    psmove_tracker_settings_set_default(&settings);
    settings.color_mapping_max_age = 0;
    settings.exposure_mode = Exposure_LOW;
    settings.camera_mirror = PSMove_True;
    settings.use_fitEllipse = 1;
    settings.color_save_colormapping = PSMove_False;
    settings.color_list_start_ind = 0;  // Start with magenta if available.
    m_tracker = psmove_tracker_new_with_settings(&settings);
    if (m_tracker == NULL) {
        fprintf(stderr, "No tracker available! (Missing camera?)\n");
        exit(1);
    }

    PSMoveTrackerSmoothingSettings smoothing_settings;
    psmove_tracker_get_smoothing_settings(m_tracker, &smoothing_settings);
    smoothing_settings.filter_do_2d_r = 0;
    smoothing_settings.filter_do_2d_xy = 0;
    smoothing_settings.filter_3d_type = Smoothing_LowPass;
    psmove_tracker_set_smoothing_settings(m_tracker, &smoothing_settings);
    
    m_fusion = psmove_fusion_new(m_tracker, 0.001, 500.);

    m_moves = (PSMove**)calloc(m_count, sizeof(PSMove*));
    m_items = (int*)calloc(m_count, sizeof(int));
    for (int i=0; i<m_count; i++) {
        m_moves[i] = psmove_connect_by_id(i);
        m_items[i] = WIRE_CUBE;

        psmove_enable_orientation(m_moves[i], PSMove_True);
        assert(psmove_has_orientation(m_moves[i]));

        while (psmove_tracker_enable(m_tracker, m_moves[i]) != Tracker_CALIBRATED);
    }
}

Tracker::~Tracker()
{
    psmove_fusion_free(m_fusion);
    psmove_tracker_free(m_tracker);
    for (int i=0; i<m_count; i++) {
        psmove_disconnect(m_moves[i]);
    }
    free(m_items);
    free(m_moves);
    psmove_shutdown();
}

void
Tracker::update()
{
    psmove_tracker_update_image(m_tracker);
    psmove_tracker_update(m_tracker, NULL);

    for (int i=0; i<m_count; i++) {
        while (psmove_poll(m_moves[i]));

        float x, y, z;
        psmove_fusion_get_position(m_fusion, m_moves[i],
                &x, &y, &z);

        int buttons = psmove_get_buttons(m_moves[i]);
        if (buttons & Btn_MOVE) {
            psmove_reset_orientation(m_moves[i]);
        } else if (buttons & Btn_PS) {
            exit(0);
        } else if (buttons & Btn_SELECT) {
            m_rotation += 2.;
        } else if (buttons & Btn_CROSS) {
            m_trace.push_back(Point3D(x, y, z));
        }

        if (buttons & Btn_START) {
            if (m_has_last_offset) {
                m_offset = Point3D(m_offset.x + x - m_last_offset.x,
                        m_offset.y + y - m_last_offset.y,
                        m_offset.z + z - m_last_offset.z);
            } else {
                m_has_last_offset = true;
            }
            m_last_offset = Point3D(x, y, z);
        } else {
            m_has_last_offset = false;
        }

        unsigned int pressed, released;
        psmove_get_button_events(m_moves[i], &pressed, &released);
        if (pressed & Btn_SQUARE) {
            m_items[i] -= 1;
            if (m_items[i] < 0) m_items[i] = ITEM_MAX - 1;
        } else if (pressed & Btn_TRIANGLE) {
            m_items[i] += 1;
            if (m_items[i] == ITEM_MAX) m_items[i] = 0;
        } else if (pressed & Btn_CIRCLE) {
            m_trace.clear();
            m_rotation = 0.;
            m_offset = Point3D(0., 0., 0.);
        }
    }
}

void
Tracker::init()
{
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

void
Tracker::render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    PSMoveTrackerRGBImage image = psmove_tracker_get_image(m_tracker);

    glEnable(GL_TEXTURE_2D);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
            0, GL_RGB, GL_UNSIGNED_BYTE, image.data);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* Draw the camera image, filling the screen */
    glColor3f(1., 1., 1.);
    glBegin(GL_QUADS);
    glTexCoord2f(0., 1.);
    glVertex2f(-1., -1.);
    glTexCoord2f(1., 1.);
    glVertex2f(1., -1.);
    glTexCoord2f(1., 0.);
    glVertex2f(1., 1.);
    glTexCoord2f(0., 0.);
    glVertex2f(-1., 1.);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    /* Clear the depth buffer to allow overdraw */
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(psmove_fusion_get_projection_matrix(m_fusion));

    /* Render the trace (when cross is pressed) */
    if (m_trace.size()) {
        Point3D center = m_trace[0];
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(center.x + m_offset.x, center.y + m_offset.y, center.z + m_offset.z);
        glRotatef(m_rotation, 0., 1., 0.);

        std::vector<Point3D>::iterator it;
        glColor3f(1., 0., 0.);
        glEnable(GL_LIGHTING);
        //glBegin(GL_TRIANGLE_STRIP);
        for (it=m_trace.begin(); it != m_trace.end(); ++it) {
            Point3D point = *it;
            Point3D moved(point.x - center.x,
                    point.y - center.y,
                    point.z - center.z);
            //glVertex3f(moved.x, moved.y, moved.z);
            glPushMatrix();
            glTranslatef(moved.x, moved.y, moved.z);
            drawSolidCube(.5f);
            glPopMatrix();
        }
        //glEnd();
        glDisable(GL_LIGHTING);
    }

    /* Render the controllers */
    for (int i=0; i<m_count; i++) {
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(psmove_fusion_get_modelview_matrix(m_fusion, m_moves[i]));

        if (m_items[i] == WIRE_CUBE) {
            // Draw small box around controller position
            glColor3f(1., 0., 0.);
            drawWireCube(1.f);

			// Draw OpenGL axes
			glBegin(GL_LINES);
            
            // Draw line through controller X (select->start button)
			glColor3f(1., 0., 0.);
			glVertex3f(0.0, 0.0, 0.0); glVertex3f(2, 0, 0);

            // Draw line through controller y (trigger->move button)
			glColor3f(0., 1., 0.);
			glVertex3f(0.0, 0.0, 0.0); glVertex3f(0, 2, 0);

            // Draw line through controller z (sphere -> usb connector)
			glColor3f(0., 0., 1.);
			glVertex3f(0.0, 0.0, 0.0); glVertex3f(0, 0, 2);

			glEnd();

            // Draw handle
            glColor3f(0., 1., 0.);
            glPushMatrix();
            glScalef(3., 3., 15.);
            glTranslatef(0., 0., .65);
            drawWireCube(1.);
            glPopMatrix();

            // Draw sphere
            glColor3f(0., 0., 1.);
            drawWireCube(5.f);

        } else if (m_items[i] == SOLID_CUBE) {
            glEnable(GL_LIGHTING);
            drawSolidCube(2.f);
            glDisable(GL_LIGHTING);
        }
    }
}


class Renderer {
    public:
        Renderer(Tracker &tracker);
        ~Renderer();

        void init();
        void render();
    private:
        SDL_Window *m_window;
        SDL_GLContext m_glContext;
        Tracker &m_tracker;
};

Renderer::Renderer(Tracker &tracker)
    : m_window(NULL),
      m_glContext(NULL),
      m_tracker(tracker)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        sdlDie("Unable to initialize SDL");
    }
    m_window = SDL_CreateWindow("OpenGL Test1",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        640, 480,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (m_window == NULL) {
        sdlDie("Unable to initialize SDL");
    }
    checkSDLError(__LINE__);
    m_glContext = SDL_GL_CreateContext(m_window);
    checkSDLError(__LINE__);
}

Renderer::~Renderer()
{
    SDL_GL_DeleteContext(m_glContext);
    SDL_Quit();
}

void
Renderer::init()
{
    glClearColor(0., 0., 0., 1.);

    glViewport(0, 0, 640, 480);

    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
}

void
Renderer::render()
{
    m_tracker.render();
    SDL_GL_SwapWindow(m_window);
}

class Main {
    public:
        Main(Tracker &tracker, Renderer &renderer);
        int exec();
    private:
        Tracker &m_tracker;
        Renderer &m_renderer;
};

Main::Main(Tracker &tracker, Renderer &renderer)
    : m_tracker(tracker),
      m_renderer(renderer)
{
}

int
Main::exec()
{
    m_renderer.init();
    m_tracker.init();

    SDL_Event e;
    while (true) {
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                break;
            }
        }
        m_tracker.update();
        m_renderer.render();
    }

    return 0;
}

extern "C" int main(int argc, char *argv[])
{
    Tracker tracker;
    Renderer renderer(tracker);
    Main main(tracker, renderer);

    return main.exec();
}


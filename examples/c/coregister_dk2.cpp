//-- includes -----
#include <stdio.h>  // printf
#include <stdlib.h> // calloc, free
#include <assert.h>


#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "psmove_examples_opengl.h"
#include "psmove.h"
#include "psmove_tracker.h"

#include "OVR_CAPI.h"
#include "Extras/OVR_Math.h"

#include "DK2_3dmodel.h"

#include <Eigen/Dense>

//-- macros -----
#define Log_INFO(section, msg, ...) \
    fprintf(stdout, "INFO [" section "] " msg "\n", ## __VA_ARGS__)

#define Log_ERROR(section, msg, ...) \
    fprintf(stderr, "ERROR [" section "] " msg "\n", ## __VA_ARGS__)

//-- typedefs -----
#ifndef OVR_OS_WIN32
//#define ovr_Initialize ovrHmd_Initialize
#define ovr_ConfigureTracking ovrHmd_ConfigureTracking
#define ovr_GetTrackingState ovrHmd_GetTrackingState
#define ovr_RecenterPose ovrHmd_RecenterPose
#define ovr_Destroy ovrHmd_Destroy
#endif

//-- constants -----
#define NPOSES 300

//-- definitions -----
class PSMoveContext 
{
public:
    PSMoveContext();
    ~PSMoveContext();

    bool init(int argc, char** argv);
    void destroy();
    void update();

private:
    bool m_psmoveapi_initialized;
    PSMove *m_move;
    PSMoveTracker *m_tracker;
    enum PSMoveTracker_Status m_tracking_status;
    PSMove_3AxisVector m_tracker_position;
    unsigned int m_buttons_down;
    unsigned int m_buttons_pressed;
    unsigned int m_buttons_released;
};

class DK2Context 
{
public:
    DK2Context();
    ~DK2Context();

    bool init();
    void destroy();
    void update();

private:
    bool m_oculusapi_initialized;
    ovrHmd m_HMD;
    ovrTrackingState m_dk2state;
    OVR::Posef m_dk2pose;               // The DK2 pose
    OVR::Matrix4f m_dk2mat;             // The DK2 HMD pose in 4x4
#if defined(OVR_OS_WIN32)
    ovrGraphicsLuid m_luid;
#endif
};

class Renderer 
{
public:
    Renderer();
    ~Renderer();

    bool init();
    void destroy();

    void render_begin();
    void render_end();

private:
    bool m_sdlapi_initialized;
    SDL_Window *m_window;
    SDL_GLContext m_glContext;
};

class App
{
public:
    App();

    int exec(int argc, char** argv);

protected:
    bool init(int argc, char** argv);
    void destroy();
    void update();
    void render();

private:
    PSMoveContext m_psmove_context;
    DK2Context m_dk2_context;
    Renderer m_renderer;
};

//-- prototypes -----
OVR::Matrix4f getDK2CameraInv44(ovrHmd HMD);
void ovrmat2eigmat(OVR::Matrix4f& in_ovr, Eigen::Matrix4f& in_eig);

//-- entry point -----
extern "C" int main(int argc, char *argv[])
{
    App app;

    return app.exec(argc, argv);
}

//-- implementation -----

//-- App --
App::App()
    : m_psmove_context()
    , m_dk2_context()
    , m_renderer()
{
}

bool
App::init(int argc, char** argv)
{
    bool success= true;

    if (success && !m_renderer.init())
    {
        Log_ERROR("App::init", "Failed to initialize renderer!");
        success= false;
    }

    if (success && !m_dk2_context.init())
    {
        Log_ERROR("App::init", "Failed to initialize Oculus tracking context!");
        success= false;
    }

    if (success && !m_psmove_context.init(argc, argv))
    {
        Log_ERROR("App::init", "Failed to initialize PSMove tracking context!");
        success= false;
    }

    return success;
}

void
App::destroy()
{
    m_psmove_context.destroy();
    m_dk2_context.destroy();
    m_renderer.destroy();
}

void
App::update()
{
    m_psmove_context.update();
    m_dk2_context.update();
}

void
App::render()
{
    m_renderer.render_begin();
    //TODO: Render the current calibration stage
    m_renderer.render_end();
}

int
App::exec(int argc, char** argv)
{
    int result= 0;

    if (init(argc, argv))
    {
        SDL_Event e;

        while (true) 
        {
            if (SDL_PollEvent(&e)) 
            {
                if (e.type == SDL_QUIT) 
                {
                    Log_INFO("App::exec", "QUIT message received");
                    break;
                }
            }

            update();
            render();
        }
    }
    else
    {
        Log_ERROR("App::exec", "Failed to initialize application!");
        result= -1;
    }

    destroy();

    return result;
}

//-- PSMoveContext -----
PSMoveContext::PSMoveContext() 
    : m_psmoveapi_initialized(false)
    , m_move(NULL)
    , m_tracker(NULL)
    , m_tracking_status(Tracker_NOT_CALIBRATED)
    , m_buttons_down(0)
    , m_buttons_pressed(0)
    , m_buttons_released(0)
{
    m_tracker_position.x= m_tracker_position.y = m_tracker_position.z= 0.f;

}

PSMoveContext::~PSMoveContext()
{
    assert(!m_psmoveapi_initialized);
}

bool PSMoveContext::init(int argc, char** argv)
{
    bool success= true;

    Log_INFO("PSMoveContext::init()", "Initializing PSMove Context");

    if (psmove_init(PSMOVE_CURRENT_VERSION))
    {
        m_psmoveapi_initialized= true;
    }
    else
    {
        Log_ERROR("PSMoveContext::init()", "PS Move API init failed (wrong version?)");
        success = false;
    }

    if (success && psmove_count_connected() < 1)
    {
        Log_ERROR("PSMoveContext::init()", "No PSMove controllers connected!");
        success = false;
    }

    Log_INFO("PSMoveContext::init()", "Turning on PSMove Tracking Camera");
    if (success)
    {
        PSMoveTrackerSettings settings;
        psmove_tracker_settings_set_default(&settings);
        settings.color_mapping_max_age = 0;
        settings.exposure_mode = Exposure_LOW;
        settings.camera_mirror = PSMove_True;
        settings.use_fitEllipse = 1;

        m_tracker = psmove_tracker_new_with_settings(&settings);
        if (m_tracker != NULL) 
        {
            Log_INFO("PSMoveContext::init()", "Tracking camera initialized");
        }
        else
        {
            Log_ERROR("PSMoveContext::init()", "No tracker available! (Missing PS3Eye camera?)");
            success= false;
        }
    }

    Log_INFO("PSMoveContext::init()", "Calibrating PSMove tracking color...");
    if (success)
    {
        enum PSMoveTracker_Status tracking_status = Tracker_TRACKING;
        int result;
        m_move = psmove_connect();

        if (success && m_move == NULL)
        {
            Log_ERROR("PSMoveContext::init()", "Failed to connect psmove controller. Is it turned on?");
            success= false;
        }

        if (success && !psmove_has_calibration(m_move))
        {
            Log_ERROR("PSMoveContext::init()", "Controller had invalid USB calibration blob. Re-Pair with the PC?");
            success= false;
        }

        if (success)
        {
            psmove_enable_orientation(m_move, PSMove_True);  // Though we don't actually use it.

            if (!psmove_has_orientation(m_move))
            {
                Log_ERROR("PSMoveContext::init()", "Failed to initialize PSMove orientation update.");
                success= false;
            }
        }

        while (success) 
        {
            if (argc >= 3) 
            {
                unsigned char r= (unsigned char)atoi(argv[1]);
                unsigned char g= (unsigned char)atoi(argv[2]);
                unsigned char b= (unsigned char)atoi(argv[3]);

                Log_INFO("PSMoveContext::init()", "Setting LEDS for controller 1 from command-line r: %i, g: %i, b: %i", r, g, b);
                result = psmove_tracker_enable_with_color(m_tracker, m_move, r, g, b);
            }
            else 
            {
                result = psmove_tracker_enable(m_tracker, m_move);
            }
            
            if (result == Tracker_CALIBRATED) 
            {
                Log_INFO("PSMoveContext::init()", "Successfully calibrated tracking color");
                break;
            } 
            else 
            {
                Log_ERROR("PSMoveContext::init()", "Failed to calibrate color. Is PSMove in frame? Retrying...");
            }
        }
    }

    return success;
}

void PSMoveContext::destroy()
{
    if (m_move != NULL)
    {
        psmove_disconnect(m_move);
        m_move= NULL;
    }

    if (m_tracker != NULL)
    {
        psmove_tracker_free(m_tracker);
        m_tracker= NULL;
    }

    if (m_psmoveapi_initialized)
    {
        psmove_shutdown();
        m_psmoveapi_initialized= false;
    }
}

void PSMoveContext::update()
{
    assert(m_tracker != NULL);
    assert(m_move != NULL);

    psmove_tracker_update_image(m_tracker);
    psmove_tracker_update(m_tracker, m_move);
    m_tracking_status = psmove_tracker_get_status(m_tracker, m_move);

    while (psmove_poll(m_move));
    
    psmove_tracker_get_location(m_tracker, m_move, 
        &m_tracker_position.x, &m_tracker_position.y, &m_tracker_position.z);

    m_buttons_down = psmove_get_buttons(m_move);
    psmove_get_button_events(m_move, &m_buttons_pressed, &m_buttons_released);
}
 
//-- DK2Context -----
DK2Context::DK2Context()
    : m_oculusapi_initialized(false)
    , m_HMD(NULL)
    , m_dk2pose()
    , m_dk2mat()
{
    memset(&m_dk2state, 0, sizeof(ovrTrackingState));
#if defined(OVR_OS_WIN32)
    memset(&m_luid, 0, sizeof(ovrGraphicsLuid));
#endif

}

DK2Context::~DK2Context()
{
    assert(!m_oculusapi_initialized);
}

bool DK2Context::init()
{
    bool success= true;

    Log_INFO("DK2Context::init()", "Initializing DK2 Context");

    if (ovr_Initialize(0) == ovrSuccess)
    {
        m_oculusapi_initialized= true;
    }
    else
    {
        Log_ERROR("DK2Context::init()", "Oculus API init failed (different SDK installed?)");
        success = false;
    }

    if (success)
    {
#if defined(OVR_OS_WIN32)
        success= (ovr_Create(&m_HMD, &m_luid) == ovrSuccess);
#elif defined(OVR_OS_MAC)
        m_HMD = ovrHmd_Create(0);
        success= (m_HMD != NULL);
#endif

        if (!success)
        {
            Log_ERROR("DK2Context::init()", "Failed to create HMD context");
            success = false;
        }
    }

    if (success && 
        ovr_ConfigureTracking(
            m_HMD,
            ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 
            0) != ovrSuccess)
    {
        Log_ERROR("DK2Context::init()", "Failed to configure tracking");
        success = false;
    }

    return success;
}

void DK2Context::destroy()
{
    if (m_HMD != NULL)
    {
        ovr_Destroy(m_HMD);
        m_HMD= NULL;
    }

    if (m_oculusapi_initialized)
    {
        ovr_Shutdown();
        m_oculusapi_initialized= false;
    }
}

void DK2Context::update()
{
    // Get DK2 tracking state (contains pose)
    m_dk2state = ovr_GetTrackingState(m_HMD, 0.0);
    m_dk2pose = m_dk2state.HeadPose.ThePose;
    m_dk2pose.Rotation.Normalize();
    m_dk2pose.Translation *= 100.0;
}

//-- Renderer -----
Renderer::Renderer()
    : m_sdlapi_initialized(false)
    , m_window(NULL)
    , m_glContext(NULL)
{
}

Renderer::~Renderer()
{
    assert(!m_sdlapi_initialized);
}

bool Renderer::init()
{
    bool success = true;

    Log_INFO("Renderer::init()", "Initializing Renderer Context");

    if (SDL_Init(SDL_INIT_VIDEO) == 0) 
    {
        m_sdlapi_initialized= true;
    }
    else
    {
        Log_ERROR("Renderer::init", "Unable to initialize SDL: %s", SDL_GetError());
        success= false;
    }

    if (success)
    {
        m_window = SDL_CreateWindow("PSMove Coregistration",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            640, 480,
            SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
        if (m_window == NULL) 
        {
            Log_ERROR("Renderer::init", "Unable to initialize window: %s", SDL_GetError());
            success= false;
        }
    }

    if (success)
    {
        m_glContext = SDL_GL_CreateContext(m_window);
        if (m_glContext == NULL) 
        {
            Log_ERROR("Renderer::init", "Unable to initialize window: %s", SDL_GetError());
            success= false;
        }
    }

    if (success)
    {
        glClearColor(7.f/255.f, 34.f/255.f, 66.f/255.f, 1.f);
        glViewport(0, 0, 640, 480);

        glEnable(GL_LIGHT0);
        glEnable(GL_DEPTH_TEST);
    }

    return success;
}

void Renderer::destroy()
{
    if (m_glContext != NULL)
    {
        SDL_GL_DeleteContext(m_glContext);
        m_glContext= NULL;
    }

    if (m_window != NULL)
    {
        SDL_DestroyWindow(m_window);
        m_window= NULL;
    }

    if (m_sdlapi_initialized)
    {
        SDL_Quit();
        m_sdlapi_initialized= false;
    }
}

void Renderer::render_begin()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::render_end()
{
    SDL_GL_SwapWindow(m_window);
}

int old_main(int arg, char** args) 
{

    // Setup PSMove
    int count = psmove_count_connected();
    PSMove **controllers = (PSMove **)calloc(count, sizeof(PSMove *));

    PSMoveTrackerSettings settings;
    psmove_tracker_settings_set_default(&settings);
    settings.color_mapping_max_age = 0;
    settings.exposure_mode = Exposure_LOW;
    settings.camera_mirror = PSMove_True;
    settings.use_fitEllipse = 1;
    PSMoveTracker* tracker = psmove_tracker_new_with_settings(&settings);
    if (tracker == NULL) {
        fprintf(stderr, "No tracker available! (Missing camera?)\n");
        exit(1);
    }

    PSMoveTrackerSmoothingSettings smoothing_settings;
    psmove_tracker_get_smoothing_settings(tracker, &smoothing_settings);
    smoothing_settings.filter_do_2d_r = 0;
    smoothing_settings.filter_do_2d_xy = 0;
    smoothing_settings.filter_3d_type = Smoothing_LowPass;
    psmove_tracker_set_smoothing_settings(tracker, &smoothing_settings);

    enum PSMoveTracker_Status tracking_status = Tracker_TRACKING;
    int result;
    int i = 0;
    controllers[i] = psmove_connect_by_id(i);
    while (1) {
        if (i == 0 && arg >= 3) {
            result = psmove_tracker_enable_with_color(tracker, controllers[i],
                                                      atoi(args[1]), atoi(args[2]), atoi(args[3]));
            printf("Setting LEDS for controller 1 from command-line r: %i, g: %i, b: %i\n",
                   atoi(args[1]), atoi(args[2]), atoi(args[3]));
        }
        else {
            result = psmove_tracker_enable(tracker, controllers[i]);
        }
        if (result == Tracker_CALIBRATED) {
            break;
        } else {
            printf("ERROR - retrying\n");
        }
    }
    assert(psmove_has_calibration(controllers[i]));
    psmove_enable_orientation(controllers[i], PSMove_True);  // Though we don't actually use it.
    assert(psmove_has_orientation(controllers[i]));
    int buttons = psmove_get_buttons(controllers[i]);
    
    // Setup DK2
    ovrBool ovrresult;
    ovrHmd HMD;
    ovrTrackingState dk2state;
    ovrresult = ovr_Initialize(0);
#if defined(OVR_OS_WIN32)
    ovrGraphicsLuid luid;
    ovr_Create(&HMD, &luid);
#elif defined(OVR_OS_MAC)
    HMD = ovrHmd_Create(0);
#endif
    ovrresult = ovr_ConfigureTracking(HMD,
                ovrTrackingCap_Orientation |
                ovrTrackingCap_MagYawCorrection |
                ovrTrackingCap_Position, 0);  //

    // Initialize variables for our loop.
    OVR::Posef dk2pose;               // The DK2 pose
    OVR::Matrix4f dk2mat;             // The DK2 HMD pose in 4x4
    OVR::Posef psmovepose;            // The psmove pose
    OVR::Matrix4f psmovemat;          // The PSMove pose in 4x4
    OVR::Matrix4f camera_invxform;    // The DK2 camera pose inverse in 4x4

    psmovepose.Rotation = OVR::Quatf::Identity();  // PSMove orientation not used by this algorithm.
    
    int p = 0;                          // NPOSES counter
    Eigen::MatrixXf A(NPOSES * 3, 15);  // X = A/b
    Eigen::VectorXf b(NPOSES * 3);
    Eigen::Matrix4f dk2eig;             // DK2 pose in Eigen 4x4 mat
    Eigen::Matrix3f RMi;                // Transpose of inner 3x3 of DK2 pose
    
    // Start with current camera pose inverse
    camera_invxform = getDK2CameraInv44(HMD);

    // Print the column headers
    char *output_fpath = psmove_util_get_file_path("output.txt");
    FILE *output_fp = fopen(output_fpath, "w");
    free(output_fpath);
    fprintf(output_fp, "psm_px,psm_py,psm_pz,psm_ow,psm_ox,psm_oy,psm_oz,dk2_px,dk2_py,dk2_pz,dk2_ow,dk2_ox,dk2_oy,dk2_oz\n");
    printf("Hold the PSMove controller firmly against the DK2.\n");
    printf("Move them together through the workspace and press the Move button to sample (%d samples required).\n", NPOSES);
    fflush(stdout);
    while (p < NPOSES)
    {
        // Get PSMove position
        psmove_tracker_update_image(tracker);               // Refresh camera
        psmove_tracker_get_location(tracker, controllers[i],  // Copy location to psmovepose
            &psmovepose.Translation.x, &psmovepose.Translation.y, &psmovepose.Translation.z);
        tracking_status = psmove_tracker_get_status(tracker, controllers[i]);
        
        if (tracking_status != Tracker_TRACKING) {
            printf("PSMove tracker failed.\n");
        }

        // Get PSMove buttons
        while (psmove_poll(controllers[i]));
        buttons = psmove_get_buttons(controllers[i]);
        
        // If circle button is pressed on PSMove then recenter HMD
        if (buttons & Btn_CIRCLE)
        {
            ovr_RecenterPose(HMD);
            camera_invxform = getDK2CameraInv44(HMD);
        }

        // Get DK2 tracking state (contains pose)
        dk2state = ovr_GetTrackingState(HMD, 0.0);
        dk2pose = dk2state.HeadPose.ThePose;
        dk2pose.Rotation.Normalize();
        dk2pose.Translation *= 100.0;
        
        // If MOVE button is pressed on PSMove, sample the position
        if (buttons & Btn_MOVE && tracking_status == Tracker_TRACKING)
        {
            fprintf(output_fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                psmovepose.Translation.x, psmovepose.Translation.y, psmovepose.Translation.z,
                psmovepose.Rotation.w, psmovepose.Rotation.x, psmovepose.Rotation.y, psmovepose.Rotation.z,
                dk2pose.Translation.x, dk2pose.Translation.y, dk2pose.Translation.z,
                dk2pose.Rotation.w, dk2pose.Rotation.x, dk2pose.Rotation.y, dk2pose.Rotation.z);

            dk2mat = OVR::Matrix4f(dk2pose);
            dk2mat = camera_invxform * dk2mat;  // Make the camera pose the new origin, so dk2 is returned relative to that.
            psmovemat = OVR::Matrix4f(psmovepose);

            /*
            if (p == 0)
            {
                printf("PSMove pose V7:\n");
                printf("%f, %f, %f, %f, %f, %f, %f\n",
                    psmovepose.Translation.x, psmovepose.Translation.y, psmovepose.Translation.z,
                    psmovepose.Rotation.w, psmovepose.Rotation.x, psmovepose.Rotation.y, psmovepose.Rotation.z);
                printf("PSMove pose 4x4:\n");
                printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
                    psmovemat.M[0][0], psmovemat.M[0][1], psmovemat.M[0][2], psmovemat.M[0][3],
                    psmovemat.M[1][0], psmovemat.M[1][1], psmovemat.M[1][2], psmovemat.M[1][3],
                    psmovemat.M[2][0], psmovemat.M[2][1], psmovemat.M[2][2], psmovemat.M[2][3],
                    psmovemat.M[3][0], psmovemat.M[3][1], psmovemat.M[3][2], psmovemat.M[3][3]);
            }
            */

            ovrmat2eigmat(dk2mat, dk2eig);
            RMi = dk2eig.topLeftCorner(3, 3).transpose();           // inner 33 transposed

            /*
            int i, j;
            for (i = 0; i < 4; i++)
            {
                printf("\n");
                for (j = 0; j < 4; j++)
                {
                    printf("%5.2f,", psmovemat(i, j));
                }
                printf("\t\t");
                for (j = 0; j < 4; j++)
                {
                    printf("%5.2f,", dk2mat(i, j));
                }
            }
            */

            A.block<3, 3>(p * 3, 0) = RMi * psmovemat.M[0][3];
            A.block<3, 3>(p * 3, 3) = RMi * psmovemat.M[1][3];
            A.block<3, 3>(p * 3, 6) = RMi * psmovemat.M[2][3];
            A.block<3, 3>(p * 3, 9) = RMi;
            A.block<3, 3>(p * 3, 12) = -Eigen::Matrix3f::Identity();
            b.segment(p * 3, 3) = RMi * dk2eig.block<3, 1>(0, 3);
            p++;

            printf("\rSampled %d / %d poses.", p, NPOSES);
            fflush(stdout);
        }

        if (buttons & Btn_SELECT)
            break;
    }

    if (p == NPOSES)
    {
        // TODO: Remove outliers

        /*
        for (p = 0; p < NPOSES * 3; p++)
        {
            printf("%4d: %5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\t%5.2f\n", p,
                A(p, 0), A(p, 1), A(p, 2), A(p, 3), A(p, 4),
                A(p, 5), A(p, 6), A(p, 7), A(p, 8), A(p, 9),
                A(p, 10), A(p, 11), A(p, 12), A(p, 13), A(p, 14), b(p));
        }
        */
        Eigen::VectorXf x(15);
        x = A.colPivHouseholderQr().solve(b);
        //globalxfm = reshape(x(1:12), 3, 4);
        //localxfm = [1 0 0 x(12); 0 1 0 x(13); 0 0 1 x(14); 0 0 0 1];
        printf("\nglobalxfm:\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
            x(0), x(3), x(6), x(9),
            x(1), x(4), x(7), x(10),
            x(2), x(5), x(8), x(11));
        printf("\nlocalxfm:\n%f,%f,%f\n", x(12), x(13), x(14));

        // Save XML to home directory
        char *fpath = psmove_util_get_file_path("transform.csv");
        FILE *fp = fopen(fpath, "w");
        free(fpath);

        // Print XML
        /*
        fprintf(fp, "< ? xml version = \"1.0\" encoding = \"UTF - 8\" ? >\n");
        fprintf(fp, "<globalxform>\n");
        int i, j;
        for (i = 0; i < 3; i++)
        {
            for (j = 0; j < 4; j++)
            {
                fprintf(fp, "    <value row=%d column=%d>%f</value>\n", i, j, x(j * 3 + i));
            }
        }
        fprintf(fp, "</globalxform>\n");
        */

        // Print simple csv
        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
            x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11));

        fclose(fp);

    }

    // Cleanup psmove
    psmove_disconnect(controllers[i]);
    psmove_tracker_free(tracker);
    free(controllers);
    psmove_shutdown();

    // Cleanup OVR
    ovr_Destroy(HMD);
    ovr_Shutdown();

    return 0;
}

//-- helper functions -----
OVR::Matrix4f getDK2CameraInv44(ovrHmd HMD)
{
    ovrTrackingState dk2state;
    
    dk2state = ovr_GetTrackingState(HMD, 0.0);
    OVR::Posef campose(dk2state.CameraPose);
    campose.Rotation.Normalize();  // Probably does nothing as the SDK returns normalized quats anyway.
    campose.Translation *= 100.0;  // m -> cm
    
    // Print to file - for testing in Matlab
    char *fpath = psmove_util_get_file_path("output_camerapose.csv");
    FILE *fp = fopen(fpath, "w");
    free(fpath);
    fprintf(fp, "%f, %f, %f, %f, %f, %f, %f\n",
        campose.Translation.x, campose.Translation.y, campose.Translation.z,
        campose.Rotation.w, campose.Rotation.x, campose.Rotation.y, campose.Rotation.z);
    fclose(fp);

    OVR::Matrix4f camMat(campose);
    
    printf("Camera pose 4x4:\n");
    printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        camMat.M[0][0], camMat.M[0][1], camMat.M[0][2], camMat.M[0][3],
        camMat.M[1][0], camMat.M[1][1], camMat.M[1][2], camMat.M[1][3],
        camMat.M[2][0], camMat.M[2][1], camMat.M[2][2], camMat.M[2][3],
        camMat.M[3][0], camMat.M[3][1], camMat.M[3][2], camMat.M[3][3]);

    camMat.InvertHomogeneousTransform();
    printf("Inverted camera pose 4x4:\n");
    printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        camMat.M[0][0], camMat.M[0][1], camMat.M[0][2], camMat.M[0][3],
        camMat.M[1][0], camMat.M[1][1], camMat.M[1][2], camMat.M[1][3],
        camMat.M[2][0], camMat.M[2][1], camMat.M[2][2], camMat.M[2][3],
        camMat.M[3][0], camMat.M[3][1], camMat.M[3][2], camMat.M[3][3]);

    return camMat;
}

void ovrmat2eigmat(OVR::Matrix4f& in_ovr, Eigen::Matrix4f& in_eig)
{
    // The following can probably be done with a simple memcpy but oh well.
    int row, col;
    for (row = 0; row < 4; row++)
    {
        for (col = 0; col < 4; col++)
        {
            in_eig(row, col) = in_ovr.M[row][col];
        }
    }
}
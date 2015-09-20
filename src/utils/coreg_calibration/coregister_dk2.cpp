//-- includes -----
#include <stdio.h>  // printf
#include <stdlib.h> // calloc, free
#include <assert.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "SDL.h"
#include "SDL_opengl.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include "stb_image.h"

#include "psmove.h"
#include "psmove_tracker.h"

#include "OVR_CAPI.h"
#include "Extras/OVR_Math.h"

#include <Eigen/Dense>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "dk2_3dmodel.h"

#ifdef _WIN32
    #pragma comment (lib, "winmm.lib")     /* link with Windows MultiMedia lib */
    #pragma comment (lib, "opengl32.lib")  /* link with Microsoft OpenGL lib */
    #pragma comment (lib, "glu32.lib")     /* link with OpenGL Utility lib */

    #pragma warning (disable:4244)	/* Disable bogus conversion warnings. */
    #pragma warning (disable:4305)  /* VC++ 5.0 version of above warning. */
#endif

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

//-- predeclarations -----
class App;
class AssetManager;

//-- constants -----
#define NPOSES 300
#define METERS_TO_CENTIMETERS 100

static const size_t k_kilo= 1<<10;
static const size_t k_meg= 1<<20;

static const int k_window_pixel_width= 800;
static const int k_window_pixel_height= 600;

static const float k_camera_vfov= 35.f;
static const float k_camera_z_near= 0.1f;
static const float k_camera_z_far= 5000.f;

static const float k_camera_mouse_zoom_scalar= 50.f;
static const float k_camera_mouse_pan_scalar= 0.5f;
static const float k_camera_min_zoom= 100.f;

static const char *k_dk2_texture_filename= "./assets/textures/DK2diffuse.jpg";

static const char *k_default_font_filename= "./assets/fonts/OpenSans-Regular.ttf";
static const float k_default_font_pixel_height= 32.f;

enum eAppStageType
{
    _appStageNone,
    _appStageSetup,
    _appStageComputeCoreg,
    _appStateTestCoreg
};

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

struct DK2TrackingCameraFrustum
{
    glm::vec3 origin;
    glm::vec3 forward, left, up;
    float HFOV, VFOV;
    float zNear, zFar;
};

class DK2Context 
{
public:
    DK2Context();
    ~DK2Context();

    bool init();
    void destroy();
    void update();

    void getTrackingCameraFrustum(DK2TrackingCameraFrustum &frustum) const;
    glm::mat4 getHMDTransform() const;

private:
    bool m_oculusapi_initialized;
    ovrHmd m_HMD;
    ovrHmdDesc m_HMDDesc;
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

    void renderBegin();
    void renderStageBegin();
    void renderStageEnd();
    void renderUIBegin();
    void renderUIEnd();
    void renderEnd();

    static bool getIsRenderingStage() 
    { return m_instance != NULL && m_instance->m_isRenderingStage; }
    static bool getIsRenderingUI()
    { return m_instance != NULL && m_instance->m_isRenderingUI; }
    float getWindowAspectRatio() const
    { return m_windowWidth / m_windowHeight; }

    void setProjectionMatrix(const glm::mat4 &matrix)
    { m_projectionMatrix= matrix; }
    void setCameraViewMatrix(const glm::mat4 &matrix)
    { m_cameraViewMatrix= matrix; }

    void renderText(float x, float y, char *text);

private:
    bool m_sdlapi_initialized;
    
    SDL_Window *m_window;
    int m_windowWidth, m_windowHeight;

    SDL_GLContext m_glContext;

    glm::mat4 m_projectionMatrix;
    glm::mat4 m_cameraViewMatrix;

    bool m_isRenderingStage;
    bool m_isRenderingUI;

    static Renderer *m_instance;
};
Renderer *Renderer::m_instance= NULL;

class AssetManager
{
public:
    struct FontAsset
    {
        GLuint textureId;
        int textureWidth, textureHeight;
        float glyphPixelHeight;
        stbtt_bakedchar cdata[96]; // ASCII 32..126 is 95 glyphs

        FontAsset()
        {
            memset(this, 0, sizeof(FontAsset));
        }
    };

    AssetManager();
    ~AssetManager();

    bool init();
    void destroy();

    static AssetManager *getInstance()
    { return m_instance; }

    GLuint getDK2TextureId()
    { return m_dk2TextureId; }
    const FontAsset *getDefaultFont()
    { return &m_defaultFont; }

private:
    bool loadTexture(const char *filename, GLuint *textureId);
    bool loadFont(const char *filename, float pixelHeight, AssetManager::FontAsset *fontAsset);

    // Utility Textures
    GLuint m_dk2TextureId;

    // Font Rendering
    FontAsset m_defaultFont;

    static AssetManager *m_instance;
};
AssetManager *AssetManager::m_instance= NULL;

class AppStage
{
public:
    AppStage(App *app) 
        : m_app(app)
    { }

    virtual void onMouseMotion(int deltaX, int deltaY) {}
    virtual void onMouseButtonDown(int buttonIndex) {}
    virtual void onMouseButtonUp(int buttonIndex) {}
    virtual void onMouseWheel(int scrollAmount) {}

    virtual void enter() {}
    virtual void exit() {}
    virtual void update() {}
    virtual void render() = 0;
    virtual void renderUI() {}

protected:
    App *m_app;
};

class SetupStage : public AppStage
{
public:
    SetupStage(App *app) 
        : AppStage(app)
        , m_isPanningOrbitCamera(false)
        , m_cameraOrbitYawDegrees(0.f)
        , m_cameraOrbitPitchDegrees(0.f)
        , m_cameraOrbitRadius(100.f)
        , m_cameraPosition(0.f, 0.f, 100.f)
    { }

    virtual void onMouseMotion(int deltaX, int deltaY);
    virtual void onMouseButtonDown(int buttonIndex);
    virtual void onMouseButtonUp(int buttonIndex);
    virtual void onMouseWheel(int scrollAmount);

    virtual void enter();
    virtual void exit();
    virtual void update();
    virtual void render();
    virtual void renderUI();

protected:
    void setCameraOrbitLocation(float yawDegrees, float pitchDegrees, float radius);

protected:
    bool m_isPanningOrbitCamera;
    float m_cameraOrbitYawDegrees;
    float m_cameraOrbitPitchDegrees;
    float m_cameraOrbitRadius;
    glm::vec3 m_cameraPosition;
};

class ComputeCoregistrationStage : public AppStage
{
public:
    ComputeCoregistrationStage(App *app)
        : AppStage(app)
    { }

    virtual void update();
    virtual void render();
};

class TestCoregistrationStage : public AppStage
{
public:
    TestCoregistrationStage(App *app)
        : AppStage(app)
    { }

    virtual void update();
    virtual void render();
};

class App
{
public:
    App();

    Renderer *getRenderer()
    { return &m_renderer; }
    DK2Context *getDK2Context()
    { return &m_dk2Context; }
    PSMoveContext *getPSMoveContext()
    { return &m_psmoveContext; }
    AssetManager *getAssetManager()
    { return &m_assetManager; }

    int exec(int argc, char** argv);

protected:
    bool init(int argc, char** argv);
    void destroy();
    
    void onSDLEvent(const SDL_Event &e);
    void update();
    void render();

    void setAppStage(eAppStageType appStageType);

private:
    // Contexts
    PSMoveContext m_psmoveContext;
    DK2Context m_dk2Context;
    Renderer m_renderer;
    AssetManager m_assetManager;

    // App Stages
    eAppStageType m_appStageType;
    AppStage *m_appStage;
    SetupStage m_setupStage;
    ComputeCoregistrationStage m_computeCoregistrationStage;
    TestCoregistrationStage m_testCoregistrationStage;
};

//-- prototypes -----
OVR::Matrix4f getDK2CameraInv44(ovrHmd HMD);
void ovrMatrix4ToEigenMatrix4(const OVR::Matrix4f& in_ovr, Eigen::Matrix4f& in_eig);
glm::mat4 ovrMatrix4fToGlmMat4(const OVR::Matrix4f& ovr_mat4);
glm::vec3 ovrVector3ToGlmVec3(const OVR::Vector3f &v);

void drawTransformedAxes(const glm::mat4 &transform, float scale);
void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &half_extents, const glm::vec3 &color);
void drawTransformedTexturedCube(const glm::mat4 &transform, int textureId, float scale);
void drawDK2Frustum(DK2TrackingCameraFrustum &frustum);
void drawDK2Model(const glm::mat4 &transform);

//-- entry point -----
extern "C" int main(int argc, char *argv[])
{
    App app;

    return app.exec(argc, argv);
}

//-- implementation -----

//-- App --
App::App()
    : m_psmoveContext()
    , m_dk2Context()
    , m_renderer()
    , m_assetManager()
    , m_appStageType(_appStageSetup)
    , m_appStage(NULL)
    , m_setupStage(this)
    , m_computeCoregistrationStage(this)
    , m_testCoregistrationStage(this)
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

    if (success && !m_assetManager.init())
    {
        Log_ERROR("App::init", "Failed to initialize asset manager!");
        success= false;
    }

    if (success && !m_dk2Context.init())
    {
        Log_ERROR("App::init", "Failed to initialize Oculus tracking context!");
        success= false;
    }

    //if (success && !m_psmoveContext.init(argc, argv))
    //{
    //    Log_ERROR("App::init", "Failed to initialize PSMove tracking context!");
    //    success= false;
    //}

    if (success)
    {
        setAppStage(_appStageSetup);
    }

    return success;
}

void 
App::setAppStage(eAppStageType appStageType)
{
    if (m_appStage != NULL)
    {
        m_appStage->exit();
    }

    switch (appStageType)
    {
    case _appStageNone:
        m_appStage= NULL;
        break;
    case _appStageSetup:
        m_appStage = &m_setupStage;
        break;
    case _appStageComputeCoreg:
        m_appStage = &m_computeCoregistrationStage;
        break;
    case _appStateTestCoreg:
        m_appStage = &m_testCoregistrationStage;
        break;
    }

    m_appStageType= appStageType;

    if (m_appStage != NULL)
    {
        m_appStage->enter();
    }
}

void
App::destroy()
{
    setAppStage(_appStageNone);
    m_psmoveContext.destroy();
    m_dk2Context.destroy();
    m_assetManager.destroy();
    m_renderer.destroy();
}

void
App::onSDLEvent(const SDL_Event &e)
{
    if (m_appStage != NULL)
    {
        switch(e.type)
        {
        case SDL_MOUSEMOTION:
            m_appStage->onMouseMotion((int)e.motion.xrel, (int)e.motion.yrel);
            break;
        case SDL_MOUSEBUTTONDOWN:
            m_appStage->onMouseButtonDown((int)e.button.button);
            break;
        case SDL_MOUSEBUTTONUP:
            m_appStage->onMouseButtonUp((int)e.button.button);
            break;
        case SDL_MOUSEWHEEL:
            m_appStage->onMouseWheel((int)e.wheel.y);
            break;
        }
    }
}

void
App::update()
{
    //m_psmoveContext.update();
    m_dk2Context.update();

    if (m_appStage != NULL)
    {
        m_appStage->update();
    }
}

void
App::render()
{
    m_renderer.renderBegin();

    m_renderer.renderStageBegin();
    if (m_appStage != NULL)
    {
        m_appStage->render();
    }
    m_renderer.renderStageEnd();

    m_renderer.renderUIBegin();
    if (m_appStage != NULL)
    {
        m_appStage->renderUI();
    }
    m_renderer.renderUIEnd();

    m_renderer.renderEnd();
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
                else 
                {
                    onSDLEvent(e);
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

//-- AppStage : IntroState -----
void SetupStage::onMouseMotion(int deltaX, int deltaY)
{
    if (m_isPanningOrbitCamera)
    {
        const float deltaYaw= -(float)deltaX * k_camera_mouse_pan_scalar;
        const float deltaPitch= (float)deltaY * k_camera_mouse_pan_scalar;

        setCameraOrbitLocation(
            m_cameraOrbitYawDegrees+deltaYaw, 
            m_cameraOrbitPitchDegrees+deltaPitch, 
            m_cameraOrbitRadius);
    }
}

void SetupStage::onMouseButtonDown(int buttonIndex)
{
    if (buttonIndex == SDL_BUTTON_LEFT)
    {
        m_isPanningOrbitCamera= true;
    }
}

void SetupStage::onMouseButtonUp(int buttonIndex)
{
    if (buttonIndex == SDL_BUTTON_LEFT)
    {
        m_isPanningOrbitCamera= false;
    }
}

void SetupStage::onMouseWheel(int scrollAmount)
{
    const float deltaRadius= (float)scrollAmount * k_camera_mouse_zoom_scalar;

    setCameraOrbitLocation(
        m_cameraOrbitYawDegrees, 
        m_cameraOrbitPitchDegrees, 
        m_cameraOrbitRadius+deltaRadius);
}

void SetupStage::enter()
{
    Renderer *renderer= m_app->getRenderer();
    const float aspect= renderer->getWindowAspectRatio();

    renderer->setProjectionMatrix(
        glm::perspective(k_camera_vfov, aspect, k_camera_z_near, k_camera_z_far));

    setCameraOrbitLocation(-45.f, 0.f, 1000.f); // degrees, degrees, cm
}

void SetupStage::exit()
{
}

void SetupStage::update()
{
}

void SetupStage::render()
{
    DK2Context *dk2Context= m_app->getDK2Context();

    drawTransformedAxes(glm::mat4(1.0f), 100.f);

    {
        DK2TrackingCameraFrustum frustum;

        dk2Context->getTrackingCameraFrustum(frustum);
        drawDK2Frustum(frustum);
    }
    
    {
        glm::mat4 transform= dk2Context->getHMDTransform();

        drawDK2Model(transform);

        drawTransformedAxes(transform, 10.f);
        //drawTransformedBox(transform, glm::vec3(9.f, 4.5f, 6.5f), glm::vec3(1.f, 0.f, 0.f));
    }
}

void SetupStage::renderUI()
{
    m_app->getRenderer()->renderText(100, 100, "This is some text!\nOn a new line as well.");
}

void SetupStage::setCameraOrbitLocation(float yawDegrees, float pitchDegrees, float radius)
{
    Renderer *renderer= m_app->getRenderer();

    m_cameraOrbitYawDegrees= fmodf(yawDegrees + 360.f, 360.f);
    m_cameraOrbitPitchDegrees= OVR::OVRMath_Max(OVR::OVRMath_Min(pitchDegrees, 60.f), 0.f);
    m_cameraOrbitRadius= OVR::OVRMath_Max(radius, k_camera_min_zoom);

    const float yawRadians= OVR::DegreeToRad(m_cameraOrbitYawDegrees);
    const float pitchRadians= OVR::DegreeToRad(m_cameraOrbitPitchDegrees);
    const float xzRadiusAtPitch= m_cameraOrbitRadius*cosf(pitchRadians);

    m_cameraPosition= glm::vec3(
        xzRadiusAtPitch*sinf(yawRadians),
        m_cameraOrbitRadius*sinf(pitchRadians),
        xzRadiusAtPitch*cosf(yawRadians));

    renderer->setCameraViewMatrix(
        glm::lookAt(
            m_cameraPosition,
            glm::vec3(0, 0, 0), // Look at tracking origin
            glm::vec3(0, 1, 0)));    // Up is up.
}

//-- AppStage : ComputeCoregistrationStage -----
void ComputeCoregistrationStage::update()
{
}

void ComputeCoregistrationStage::render()
{
    
}

//-- AppStage : TestCoregistrationStage -----
void TestCoregistrationStage::update()
{
}

void TestCoregistrationStage::render()
{
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

    if (success)
    {
        m_HMDDesc= ovr_GetHmdDesc(m_HMD);
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
    m_dk2pose.Translation *= METERS_TO_CENTIMETERS;    
    m_dk2mat= OVR::Matrix4f(m_dk2pose);
}

void DK2Context::getTrackingCameraFrustum(
    DK2TrackingCameraFrustum &frustum) const
{
    const ovrPosef &cameraPose= m_dk2state.CameraPose;
    const ovrQuatf &q= m_dk2state.CameraPose.Orientation;
    OVR::Matrix3f cameraMatrix(OVR::Quatf(q.x, q.y, q.z, q.w));

    frustum.origin= ovrVector3ToGlmVec3(cameraPose.Position);
    frustum.origin*= METERS_TO_CENTIMETERS;

    frustum.forward= ovrVector3ToGlmVec3(cameraMatrix.Col(OVR::Axis_Z));
    frustum.left= ovrVector3ToGlmVec3(cameraMatrix.Col(OVR::Axis_X));
    frustum.up= ovrVector3ToGlmVec3(cameraMatrix.Col(OVR::Axis_Y));

    frustum.HFOV= m_HMDDesc.CameraFrustumHFovInRadians;
    frustum.VFOV= m_HMDDesc.CameraFrustumVFovInRadians;
    frustum.zNear= m_HMDDesc.CameraFrustumNearZInMeters*METERS_TO_CENTIMETERS;
    frustum.zFar= m_HMDDesc.CameraFrustumFarZInMeters*METERS_TO_CENTIMETERS;
}

glm::mat4 DK2Context::getHMDTransform() const
{
    return ovrMatrix4fToGlmMat4(m_dk2mat);
}

//-- Renderer -----
Renderer::Renderer()
    : m_sdlapi_initialized(false)
    , m_window(NULL)
    , m_windowWidth(0)
    , m_windowHeight(0)
    , m_glContext(NULL)
    , m_projectionMatrix()
    , m_cameraViewMatrix()
    , m_isRenderingStage(false)
    , m_isRenderingUI(false)
{
}

Renderer::~Renderer()
{
    assert(!m_sdlapi_initialized);
    assert(m_instance == NULL);
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
            k_window_pixel_width, k_window_pixel_height,
            SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
        m_windowWidth= k_window_pixel_width;
        m_windowHeight= k_window_pixel_height;

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
        glViewport(0, 0, m_windowWidth, m_windowHeight);

        glEnable(GL_LIGHT0);
        glEnable(GL_TEXTURE_2D);
        //glShadeModel(GL_SMOOTH);
        //glClearDepth(1.0f);
        glEnable(GL_DEPTH_TEST);
        //glDepthFunc(GL_LEQUAL);
        //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        m_instance= this;
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

    m_instance= NULL;
}

void Renderer::renderBegin()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::renderStageBegin()
{
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(m_projectionMatrix));

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(m_cameraViewMatrix));

    m_isRenderingStage= true;
}

void Renderer::renderStageEnd()
{
    m_isRenderingStage= false;
}

void Renderer::renderUIBegin()
{
    const glm::mat4 ortho_projection= glm::ortho(
        0.f, (float)m_windowWidth, // left, right
        (float)m_windowHeight, 0.f, // bottom, top
        -1.0f, 1.0f); // zNear, zFar

    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(ortho_projection));

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    m_isRenderingUI= true;
}

void Renderer::renderUIEnd()
{
    m_isRenderingUI= false;
}

void Renderer::renderEnd()
{
    SDL_GL_SwapWindow(m_window);
}

void Renderer::renderText(float x, float y, char *text)
{
    assert(m_isRenderingUI); // Don't call this outside of the renderUI() callback

    const AssetManager::FontAsset *font= AssetManager::getInstance()->getDefaultFont();
    const float initial_x= x;

    // assume orthographic projection with units = screen pixels, origin at top left
    glBindTexture(GL_TEXTURE_2D, font->textureId);
    glColor3f(1.f, 1.f, 1.f);

    glBegin(GL_QUADS);

    while (*text) 
    {
        char ascii_character= *text;

        if (ascii_character >= 32 && ascii_character < 128) 
        {
            stbtt_aligned_quad glyph_quad;
            int char_index= (int)ascii_character - 32;

            stbtt_GetBakedQuad(
                const_cast<stbtt_bakedchar *>(font->cdata), 
                font->textureWidth, font->textureHeight, 
                char_index, 
                &x, &y, // x position advances with character by the glyph pixel width
                &glyph_quad,
                1); // opengl_fillrule= true
            glTexCoord2f(glyph_quad.s0,glyph_quad.t0); glVertex2f(glyph_quad.x0,glyph_quad.y0);
            glTexCoord2f(glyph_quad.s1,glyph_quad.t0); glVertex2f(glyph_quad.x1,glyph_quad.y0);
            glTexCoord2f(glyph_quad.s1,glyph_quad.t1); glVertex2f(glyph_quad.x1,glyph_quad.y1);
            glTexCoord2f(glyph_quad.s0,glyph_quad.t1); glVertex2f(glyph_quad.x0,glyph_quad.y1);
        }
        else if (ascii_character == '\n')
        {
            x= initial_x;
            y+= font->glyphPixelHeight;
        }

        ++text;
    }
    glEnd();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0);
}

//-- AssetManager -----
AssetManager::AssetManager()
    : m_dk2TextureId(0)
    //, m_defaultFont()
{
}

AssetManager::~AssetManager()
{
    assert(m_instance== NULL);
}

bool AssetManager::init()
{
    bool success= true;

    if (success)
    {
        success= loadTexture(k_dk2_texture_filename, &m_dk2TextureId);
    }

    if (success)
    {
        success= loadFont(k_default_font_filename, k_default_font_pixel_height, &m_defaultFont);
    }

    if (success)
    {
        m_instance= this;
    }

    return success;
}

void AssetManager::destroy()
{
    if (m_dk2TextureId != 0)
    {
        glDeleteTextures(1, &m_dk2TextureId);
        m_dk2TextureId= 0;
    }

    if (m_defaultFont.textureId != 0)
    {
        glDeleteTextures(1, &m_defaultFont.textureId);
        m_defaultFont.textureId= 0;
    }

    m_instance= NULL;
}

bool AssetManager::loadTexture(const char *filename, GLuint *textureId)
{
    bool success= false;

    int pixelWidth=0, pixelHeight=0, channelCount=0;
    stbi_uc *image_buffer= stbi_load(filename, &pixelWidth, &pixelHeight, &channelCount, 3);

    if (image_buffer != NULL)
    {
        GLint glPixelFormat= -1;

        if (channelCount == 3)
        {
            glGenTextures(1, textureId);

            // Typical Texture Generation Using Data From The Bitmap
            glBindTexture(GL_TEXTURE_2D, *textureId);
            glTexImage2D(GL_TEXTURE_2D, 0, 3, pixelWidth, pixelHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image_buffer);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

            success= true;
        }
        else
        {
            Log_ERROR("AssetManager::loadTexture", "Image isn't RGB24 pixel format!");
        }

        stbi_image_free(image_buffer);
    }
    else
    {
        Log_ERROR("AssetManager::loadTexture", "Failed to load: %s(%s)", filename, SDL_GetError());
    }

    return success;
}

bool AssetManager::loadFont(const char *filename, const float pixelHeight, AssetManager::FontAsset *fontAsset)
{
    unsigned char *temp_ttf_buffer = NULL;
    unsigned char *temp_bitmap = NULL;

    bool success= true;

    // For now assume all font sprite sheets fit in a 512x512 texture
    fontAsset->textureWidth= 512;
    fontAsset->textureHeight= 512;
    fontAsset->glyphPixelHeight= pixelHeight;

    // Allocate scratch buffers
    temp_ttf_buffer = NULL;
    temp_bitmap = new unsigned char[fontAsset->textureWidth*fontAsset->textureHeight];

    // Load the True Type Font data into memory
    if (success)
    {
        FILE *fp= fopen(k_default_font_filename, "rb");
        if (fp != NULL)
        {
            // obtain file size
            fseek (fp , 0 , SEEK_END);
            size_t fileSize = ftell (fp);
            rewind (fp);

            if (fileSize > 0 && fileSize < 10*k_meg)
            {
                temp_ttf_buffer= new unsigned char[fileSize];
                size_t bytes_read= fread(temp_ttf_buffer, 1, fileSize, fp);

                if (bytes_read != fileSize)
                {
                    Log_ERROR("AssetManager::loadFont", "Failed to load font (%s): failed to read expected # of bytes.", filename);
                    success= false;
                }
            }
            else
            {
                Log_ERROR("AssetManager::loadFont", "Failed to load font (%s): file size invalid", filename);
                success= false;
            }

            fclose(fp);
        }
        else
        {
            Log_ERROR("AssetManager::loadFont", "Failed to open font file (%s)", filename);
            success= false;
        }
    }

    // Build the sprite sheet for the font
    if (success)
    {
        if (stbtt_BakeFontBitmap(
            temp_ttf_buffer, 0, 
            pixelHeight, 
            temp_bitmap, fontAsset->textureWidth, fontAsset->textureHeight, 
            32,96, fontAsset->cdata) <= 0)
        {
            Log_ERROR("AssetManager::loadFont", "Failed to fit font(%s) into %dx%d sprite texture", 
                filename, fontAsset->textureWidth, fontAsset->textureHeight);
            success= false;
        }
    }
    
    // Generate the texture for the font sprite sheet
    if (success)
    {
        glGenTextures(1, &fontAsset->textureId);
        glBindTexture(GL_TEXTURE_2D, fontAsset->textureId);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, 512,512, 0, GL_ALPHA, GL_UNSIGNED_BYTE, temp_bitmap);            
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }

    // free scratch buffers
    delete[] temp_bitmap;
    if (temp_ttf_buffer != NULL) delete[] temp_ttf_buffer;

    return success;
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

            ovrMatrix4ToEigenMatrix4(dk2mat, dk2eig);
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

//-- debug render helper functions -----
void drawDK2Frustum(DK2TrackingCameraFrustum &frustum)
{
    assert(Renderer::getIsRenderingStage());

    const float HRatio= tanf(frustum.HFOV/2.f);
    const float VRatio= tanf(frustum.VFOV/2.f);

    glm::vec3 nearX= frustum.left*frustum.zNear*HRatio;
    glm::vec3 farX= frustum.left*frustum.zFar*HRatio;

    glm::vec3 nearY= frustum.up*frustum.zNear*VRatio;
    glm::vec3 farY= frustum.up*frustum.zFar*VRatio;

    glm::vec3 nearZ= frustum.forward*frustum.zNear;
    glm::vec3 farZ= frustum.forward*frustum.zFar;

    glm::vec3 nearCenter= frustum.origin + nearZ;
    glm::vec3 near0= frustum.origin + nearX + nearY + nearZ;
    glm::vec3 near1= frustum.origin - nearX + nearY + nearZ;
    glm::vec3 near2= frustum.origin - nearX - nearY + nearZ;
    glm::vec3 near3= frustum.origin + nearX - nearY + nearZ;

    glm::vec3 far0= frustum.origin + farX + farY + farZ;
    glm::vec3 far1= frustum.origin - farX + farY + farZ;
    glm::vec3 far2= frustum.origin - farX - farY + farZ;
    glm::vec3 far3= frustum.origin + farX - farY + farZ;
    
    glBegin(GL_LINES);

    glColor3ub(255, 201, 14);

    glVertex3fv(glm::value_ptr(near0)); glVertex3fv(glm::value_ptr(near1));
    glVertex3fv(glm::value_ptr(near1)); glVertex3fv(glm::value_ptr(near2));
    glVertex3fv(glm::value_ptr(near2)); glVertex3fv(glm::value_ptr(near3));
    glVertex3fv(glm::value_ptr(near3)); glVertex3fv(glm::value_ptr(near0));

    glVertex3fv(glm::value_ptr(far0)); glVertex3fv(glm::value_ptr(far1));
    glVertex3fv(glm::value_ptr(far1)); glVertex3fv(glm::value_ptr(far2));
    glVertex3fv(glm::value_ptr(far2)); glVertex3fv(glm::value_ptr(far3));
    glVertex3fv(glm::value_ptr(far3)); glVertex3fv(glm::value_ptr(far0));

    glVertex3fv(glm::value_ptr(frustum.origin)); glVertex3fv(glm::value_ptr(far0));
    glVertex3fv(glm::value_ptr(frustum.origin)); glVertex3fv(glm::value_ptr(far1));
    glVertex3fv(glm::value_ptr(frustum.origin)); glVertex3fv(glm::value_ptr(far2));
    glVertex3fv(glm::value_ptr(frustum.origin)); glVertex3fv(glm::value_ptr(far3));

    glVertex3fv(glm::value_ptr(frustum.origin));
    glColor3ub(0, 255, 0);
    glVertex3fv(glm::value_ptr(nearCenter));

    glEnd();
}

void drawTransformedAxes(const glm::mat4 &transform, float scale)
{
    assert(Renderer::getIsRenderingStage());

    glm::vec3 origin(0.f, 0.f, 0.f);
    glm::vec3 xAxis(scale, 0.f, 0.f);
    glm::vec3 yAxis(0.f, scale, 0.f);
    glm::vec3 zAxis(0.f, 0.f, scale);
   
    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glBegin(GL_LINES);

        glColor3ub(255, 0, 0);
        glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(xAxis));

        glColor3ub(0, 255, 0);
        glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(yAxis));

        glColor3ub(0, 0, 255);
        glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(zAxis));

        glEnd();
    glPopMatrix();
}

void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &half_extents, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    glm::vec3 v0(half_extents.x, half_extents.y, half_extents.z);
    glm::vec3 v1(-half_extents.x, half_extents.y, half_extents.z);
    glm::vec3 v2(-half_extents.x, half_extents.y, -half_extents.z);
    glm::vec3 v3(half_extents.x, half_extents.y, -half_extents.z);
    glm::vec3 v4(half_extents.x, -half_extents.y, half_extents.z);
    glm::vec3 v5(-half_extents.x, -half_extents.y, half_extents.z);
    glm::vec3 v6(-half_extents.x, -half_extents.y, -half_extents.z);
    glm::vec3 v7(half_extents.x, -half_extents.y, -half_extents.z);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glColor3fv(glm::value_ptr(color));

        glBegin(GL_LINES);

        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v1));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v2));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v3));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v0));

        glVertex3fv(glm::value_ptr(v4)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v5)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v6)); glVertex3fv(glm::value_ptr(v7));
        glVertex3fv(glm::value_ptr(v7)); glVertex3fv(glm::value_ptr(v4));

        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v4));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v7));

        glEnd();
    glPopMatrix();
}

void drawTransformedTexturedCube(const glm::mat4 &transform, int textureId, float scale)
{
    assert(Renderer::getIsRenderingStage());

    glBindTexture(GL_TEXTURE_2D, textureId);
    glColor3f(1.f, 1.f, 1.f);

    glBegin(GL_QUADS);
        glMultMatrixf(glm::value_ptr(transform));
        // Front Face
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-scale, -scale,  scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f( scale, -scale,  scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f( scale,  scale,  scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-scale,  scale,  scale);
        // Back Face
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-scale, -scale, -scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f(-scale,  scale, -scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f( scale,  scale, -scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f( scale, -scale, -scale);
        // Top Face
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-scale,  scale, -scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-scale,  scale,  scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f( scale,  scale,  scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f( scale,  scale, -scale);
        // Bottom Face
        glTexCoord2f(1.0f, 1.0f); glVertex3f(-scale, -scale, -scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f( scale, -scale, -scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f( scale, -scale,  scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-scale, -scale,  scale);
        // Right face
        glTexCoord2f(1.0f, 0.0f); glVertex3f( scale, -scale, -scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f( scale,  scale, -scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f( scale,  scale,  scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f( scale, -scale,  scale);
        // Left Face
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-scale, -scale, -scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-scale, -scale,  scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f(-scale,  scale,  scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-scale,  scale, -scale);
    glEnd();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

void drawDK2Model(const glm::mat4 &transform)
{
    assert(Renderer::getIsRenderingStage());

    int textureID= AssetManager::getInstance()->getDK2TextureId();

    glBindTexture(GL_TEXTURE_2D, textureID);
    glColor3f(1.f, 1.f, 1.f);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, DK2Verts);
        glTexCoordPointer(2, GL_FLOAT, 0, DK2TexCoords);
        glDrawArrays(GL_TRIANGLES, 0, DK2NumVerts);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glPopMatrix();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

//-- math helper functions -----
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

void ovrMatrix4ToEigenMatrix4(const OVR::Matrix4f& in_ovr, Eigen::Matrix4f& in_eig)
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

glm::mat4 ovrMatrix4fToGlmMat4(const OVR::Matrix4f& ovr_mat4)
{
    // ovr matrices are stored row-major in memory
    // glm matrices are stored colomn-major in memory
    // Thus the transpose
    return glm::transpose(glm::make_mat4((const float *)ovr_mat4.M));
}

glm::vec3 ovrVector3ToGlmVec3(const OVR::Vector3f &v)
{
    return glm::vec3(v.x, v.y, v.z);
}
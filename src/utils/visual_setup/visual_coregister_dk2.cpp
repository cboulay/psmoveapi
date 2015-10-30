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
#include "psmove_fusion.h"
#include "math/psmove_math.h"

#include "OVR_CAPI.h"
#include "Extras/OVR_Math.h"

#include <Eigen/Dense>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "dk2_3dmodel.h"
#include "psmove_3dmodel.h"

#ifdef _WIN32
    #pragma comment (lib, "winmm.lib")     /* link with Windows MultiMedia lib */
    #pragma comment (lib, "opengl32.lib")  /* link with Microsoft OpenGL lib */
    #pragma comment (lib, "glu32.lib")     /* link with OpenGL Utility lib */

    #pragma warning (disable:4244)	/* Disable bogus conversion warnings. */
    #pragma warning (disable:4305)  /* VC++ 5.0 version of above warning. */
#endif

//-- macros -----
#define Log_INFO(section, format, ...) \
    fprintf(stdout, "INFO [" section "] " format "\n", ## __VA_ARGS__)

#define Log_ERROR(section, format, ...) \
    fprintf(stderr, "ERROR [" section "] " format "\n", ## __VA_ARGS__)

//-- typedefs -----
#ifndef OVR_OS_WIN32
//#define ovr_Initialize ovrHmd_Initialize
#define ovr_ConfigureTracking ovrHmd_ConfigureTracking
#define ovr_GetTrackingState ovrHmd_GetTrackingState
#define ovr_RecenterPose ovrHmd_RecenterPose
#define ovr_Destroy ovrHmd_Destroy
#define ovrSuccess ovrTrue
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
static const float k_default_font_pixel_height= 24.f;

static const glm::vec3 k_dk2_frustum_color= glm::vec3(1.f, 0.788f, 0.055f);
static const glm::vec3 k_psmove_frustum_color= glm::vec3(0.1f, 0.7f, 0.3f);

enum eAppStageType
{
    _appStageNone,
    _appStageOrbitCameraSetup,
    _appStagePSMoveSetup,
    _appStageComputeCoreg,
    _appStateTestCoreg
};

enum eCameraType
{
    _cameraNone,
    _cameraOrbit,
    _cameraFixed
};

//-- definitions -----
struct TrackingCameraFrustum
{
    glm::vec3 origin;
    glm::vec3 forward, left, up;
    float HFOV, VFOV;
    float zNear, zFar;
};

struct FrustumBounds
{
    glm::vec3 origin;
    glm::vec3 forward, left, up;
    float hAngleMin, hAngleMax; // radians
    float vAngleMin, vAngleMax; // radians
    float zNear, zFar;
};

class PSMoveContext 
{
public:
    PSMoveContext();
    ~PSMoveContext();

    bool init(int argc, char** argv);
    void destroy();
    void update();

    bool connectController();
    bool initTracker();
    bool calibrateTracker();
    void initFusion();
    void initFusionWithCoregTransform(const glm::mat4 &coregistrationTransform);

    glm::mat4 computeWorldTransform(const glm::mat4 &dk2CameraToWorldTransform) const;
    void getTrackingCameraFrustum(const class DK2Context *dk2Context, TrackingCameraFrustum &outFrustum) const;

    const char *getLastErrorMessage() const
    { return m_lastErrorMessage; }
    PSMoveTracker_Status getTrackingStatus() const
    { return m_tracking_status; }
    PSMove_3AxisVector getTrackerPosition() const
    { return m_tracker_position; }
    unsigned int getButtonsDownBitmask() const
    { return m_buttons_down; }
    unsigned int getButtonsPressedBitmask() const
    { return m_buttons_pressed; }
    unsigned int getButtonsReleasedBitmask() const
    { return m_buttons_released; }

private:
    void setLastErrorMessage(const char *errorMessage);

private:
    bool m_psmoveapi_initialized;
    PSMove *m_move;
    PSMoveTracker *m_tracker;
    PSMoveFusion *m_fusion;
    glm::mat4 m_PS3EyeToDK2CameraXform;
    glm::mat4 m_DK2CameraToPS3EyeXform;
    enum PSMoveTracker_Status m_tracking_status;
    PSMove_3AxisVector m_tracker_position;
    unsigned int m_buttons_down;
    unsigned int m_buttons_pressed;
    unsigned int m_buttons_released;
    PSMoveTracker_Camera_API m_camera_api;
    bool m_use_custom_tracking_color;
    unsigned char m_custom_r, m_custom_g, m_custom_b;
    const char *m_lastErrorMessage;
};

class DK2Context 
{
public:
    DK2Context();
    ~DK2Context();

    bool init();
    void destroy();
    void update();

    void getTrackingCameraFrustum(TrackingCameraFrustum &frustum) const;
    OVR::Posef getHMDPose() const;
    OVR::Matrix4f getHMDTransform() const;
    OVR::Matrix4f getCameraTransform() const;
    OVR::Matrix4f getDK2CameraInv44() const;

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

    void renderText(float x, float y, const char *text);

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

class Camera
{
public:
    Camera(App *app) 
        : m_app(app)
        , m_cameraOrbitYawDegrees(0.f)
        , m_cameraOrbitPitchDegrees(0.f)
        , m_cameraOrbitRadius(100.f)
        , m_cameraPosition(0.f, 0.f, 100.f)
        , m_isPanningOrbitCamera(false)
        , m_isLocked(false)
    { }

    void onMouseMotion(int deltaX, int deltaY);
    void onMouseButtonDown(int buttonIndex);
    void onMouseButtonUp(int buttonIndex);
    void onMouseWheel(int scrollAmount);

    void setIsLocked(bool locked);
    void setCameraOrbitLocation(float yawDegrees, float pitchDegrees, float radius);
    void publishCameraViewMatrix();

private:
    App *m_app;
    float m_cameraOrbitYawDegrees;
    float m_cameraOrbitPitchDegrees;
    float m_cameraOrbitRadius;
    glm::vec3 m_cameraPosition;
    bool m_isPanningOrbitCamera;
    bool m_isLocked;
};

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

    virtual void enter() {}
    virtual void exit() {}
    virtual void update() {}
    virtual void render() = 0;
    virtual void renderUI() {}

    virtual void onKeyDown(SDL_Keycode keyCode) {}

protected:
    App *m_app;
};

class OrbitCameraSetupStage : public AppStage
{
public:
    OrbitCameraSetupStage(App *app) 
        : AppStage(app)
    { }

    virtual void enter();
    virtual void exit();
    virtual void update();
    virtual void render();
    virtual void renderUI();

    virtual void onKeyDown(SDL_Keycode keyCode);
};

class PSMoveSetupStage : public AppStage
{
private:
    enum ePSMoveSetup
    {
        _PSMoveSetupWaitingToStart,
        _PSMoveSetupConnectPSMove,
        _PSMoveSetupConnectPSMoveFailed,
        _PSMoveSetupConnectTracker,
        _PSMoveSetupConnectTrackerFailed,
        _PSMoveSetupCalibrateTracker,
        _PSMoveSetupCalibrateTrackerFailed,
        _PSMoveSetupSucceeded
    };

    ePSMoveSetup m_psmoveSetupState;
    bool m_hasRenderedStageAtLeastOnce;

    void setPSMoveSetupState(ePSMoveSetup newState)
    {
        if (newState != m_psmoveSetupState)
        {
            m_psmoveSetupState= newState;
            m_hasRenderedStageAtLeastOnce= false;
        }
    }

public:
    PSMoveSetupStage(App *app) 
        : AppStage(app)
        , m_psmoveSetupState(_PSMoveSetupWaitingToStart)
    { }

    virtual void enter();
    virtual void update();
    virtual void render();
    virtual void renderUI();

    virtual void onKeyDown(SDL_Keycode keyCode);
};

class ComputeCoregistrationStage : public AppStage
{
private:
    enum eCoregStage
    {
        _CoregStageAttachPSMoveToDK2,
        _CoregStageSampling,
        _CoregStageComplete
    };

    eCoregStage m_coregState;
    
    OVR::Matrix4f m_camera_invxform;
    OVR::Posef m_dk2poses[NPOSES];
    PSMove_3AxisVector m_psmoveposes[NPOSES];
    int m_poseCount;

	FrustumBounds m_sampleBounds;

	glm::mat4 m_coregTransform;

    int m_argc;
    char** m_argv;

public:
    ComputeCoregistrationStage(App *app)
        : AppStage(app)
        , m_coregState(_CoregStageAttachPSMoveToDK2)
        , m_poseCount(0)
        , m_argc(0)
        , m_argv(NULL)
    { }

    bool init(int argc, char** argv);

	bool getCoregistrationTransform(glm::mat4 &out_mat) const 
    { 
        out_mat= m_coregTransform;
        return m_poseCount > 0;
    }

    virtual void enter();
    virtual void update();
    virtual void render();
    virtual void renderUI();

    virtual void onKeyDown(SDL_Keycode keyCode);
};

class TestCoregistrationStage : public AppStage
{
public:
    TestCoregistrationStage(App *app)
        : AppStage(app)
    { }

    virtual void enter();
    virtual void render();
    virtual void renderUI();

    virtual void onKeyDown(SDL_Keycode keyCode);
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

    Camera *getOrbitCamera()
    { return &m_orbitCamera; }
    Camera *getFixedCamera()
    { return &m_fixedCamera; }

	OrbitCameraSetupStage *getOrbitCameraSetupStage() { return &m_orbitCameraSetupStage; }
	PSMoveSetupStage *getPSMoveSetupStage() { return &m_psmoveSetupStage; }
	ComputeCoregistrationStage *getComputeCoregistrationStage() { return &m_computeCoregistrationStage; }
	TestCoregistrationStage *getTestCoregistrationStage() { return &m_testCoregistrationStage; }

    int exec(int argc, char** argv);

    void setCameraType(eCameraType cameraType);
    void setAppStage(eAppStageType appStageType);

protected:
    bool init(int argc, char** argv);
    void destroy();
    
    void onSDLEvent(const SDL_Event &e);
    void update();
    void render();

private:
    // Contexts
    PSMoveContext m_psmoveContext;
    DK2Context m_dk2Context;
    Renderer m_renderer;

    // Assets (textures, fonts, sounds)
    AssetManager m_assetManager;

    // Cameras
    eCameraType m_cameraType;
    Camera *m_camera;
    Camera m_orbitCamera;
    Camera m_fixedCamera;

    // App Stages
    eAppStageType m_appStageType;
    AppStage *m_appStage;
    OrbitCameraSetupStage m_orbitCameraSetupStage;
    PSMoveSetupStage m_psmoveSetupStage;
    ComputeCoregistrationStage m_computeCoregistrationStage;
    TestCoregistrationStage m_testCoregistrationStage;
};

//-- prototypes -----
void computeAndSaveCoregistrationTransform(
    const OVR::Matrix4f &camera_invxform,
    const OVR::Posef *dk2poses, const PSMove_3AxisVector *psmoveposes, int poseCount,
	glm::mat4 &outCoregTransform);

void frustumBoundsInit(
    const glm::vec3 &origin, const glm::vec3 &forward, const glm::vec3 &left, const glm::vec3 &up, 
    FrustumBounds &bounds);
void frustumBoundsEnclosePoint(const glm::vec3 &point, FrustumBounds &bounds);

OVR::Matrix4f getDK2CameraInv44(ovrHmd HMD);
void ovrMatrix4ToEigenMatrix4(const OVR::Matrix4f& in_ovr, Eigen::Matrix4f& in_eig);
glm::mat4 ovrMatrix4fToGlmMat4(const OVR::Matrix4f& ovr_mat4);
glm::vec3 ovrVector3ToGlmVec3(const OVR::Vector3f &v);

void drawTransformedAxes(const glm::mat4 &transform, float scale);
void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &half_extents, const glm::vec3 &color);
void drawTransformedTexturedCube(const glm::mat4 &transform, int textureId, float scale);
void drawTrackingFrustum(const TrackingCameraFrustum &frustum, const glm::vec3 &color);
void drawFrustumBounds(const FrustumBounds &frustum, const glm::vec3 &color);
void drawDK2Samples(const OVR::Posef *dk2poses, const int poseCount);
void drawDK2Model(const glm::mat4 &transform);
void drawPSMoveSamples(const PSMove_3AxisVector *psmoveposes, const int poseCount);
void drawPSMoveModel(const glm::mat4 &transform, const glm::vec3 &color);

static bool loadDK2CameraInv44(const char *filename, OVR::Matrix4f *outCamMat);
static FILE *initRecordedPoseStream(const char *filename);
static bool readNextRecordedPoseStreamLine(FILE *stream, OVR::Posef *out_psmovepose, OVR::Posef *out_dk2pose);
static void closeRecordedPoseStream(FILE *stream);

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
    , m_cameraType(_cameraNone)
    , m_camera(NULL)
    , m_orbitCamera(this)
    , m_fixedCamera(this)
    , m_appStageType(_appStageOrbitCameraSetup)
    , m_appStage(NULL)
    , m_orbitCameraSetupStage(this)
    , m_psmoveSetupStage(this)
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

    if (success && !m_psmoveContext.init(argc, argv))
    {
        Log_ERROR("App::init", "Failed to initialize PSMove tracking context!");
        success= false;
    }

    if (success && !m_computeCoregistrationStage.init(argc, argv))
    {
        Log_ERROR("App::init", "Failed to initialize compute coreg stage!");
        success= false;
    }

    if (success)
    {
        m_orbitCamera.setIsLocked(false);
        m_fixedCamera.setIsLocked(true);

        setAppStage(_appStageOrbitCameraSetup);
    }

    return success;
}

void 
App::setCameraType(eCameraType cameraType)
{
    switch (cameraType)
    {
    case _cameraNone:
        m_camera= NULL;
        break;
    case _cameraOrbit:
        m_camera= &m_orbitCamera;
        break;
    case _cameraFixed:
        m_camera= &m_fixedCamera;
        break;
    }

    m_cameraType= cameraType;

    if (m_camera != NULL)
    {
        m_camera->publishCameraViewMatrix();
    }
    else
    {
        m_renderer.setCameraViewMatrix(glm::mat4(1.f));
    }
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
    case _appStageOrbitCameraSetup:
        m_appStage = &m_orbitCameraSetupStage;
        break;
    case _appStagePSMoveSetup:
        m_appStage = &m_psmoveSetupStage;
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
        case SDL_KEYDOWN:
            m_appStage->onKeyDown(e.key.keysym.sym);
            break;
        }
    }

    if (m_camera != NULL)
    {
        switch(e.type)
        {
        case SDL_MOUSEMOTION:
            m_camera->onMouseMotion((int)e.motion.xrel, (int)e.motion.yrel);
            break;
        case SDL_MOUSEBUTTONDOWN:
            m_camera->onMouseButtonDown((int)e.button.button);
            break;
        case SDL_MOUSEBUTTONUP:
            m_camera->onMouseButtonUp((int)e.button.button);
            break;
        case SDL_MOUSEWHEEL:
            m_camera->onMouseWheel((int)e.wheel.y);
            break;
        }
    }
}

void
App::update()
{
    m_psmoveContext.update();
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
                if (e.type == SDL_QUIT || 
                    (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)) 
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

//-- AppStage : OrbitCameraSetup -----
void OrbitCameraSetupStage::enter()
{
    m_app->getOrbitCamera()->setCameraOrbitLocation(-3.f, 0.f, 500.f); // yaw degrees, pitch degrees, radius cm
    m_app->setCameraType(_cameraOrbit);
}

void OrbitCameraSetupStage::exit()
{
}

void OrbitCameraSetupStage::update()
{
}

void OrbitCameraSetupStage::render()
{
    DK2Context *dk2Context= m_app->getDK2Context();

    drawTransformedAxes(glm::mat4(1.0f), 100.f);

    {
        TrackingCameraFrustum frustum;

        dk2Context->getTrackingCameraFrustum(frustum);
        drawTrackingFrustum(frustum, k_dk2_frustum_color);
    }
    
    {
        glm::mat4 transform= ovrMatrix4fToGlmMat4(dk2Context->getHMDTransform());

        drawDK2Model(transform);

        drawTransformedAxes(transform, 10.f);
    }
}

void OrbitCameraSetupStage::renderUI()
{
    m_app->getRenderer()->renderText(340, 20, "Viewpoint Setup");
    m_app->getRenderer()->renderText(200, 40, "Make sure DK2 is tracking position properly");
    m_app->getRenderer()->renderText(10, 550, "Left mouse button to pan camera. Mouse wheel to zoom.");
    m_app->getRenderer()->renderText(10, 570, "Press space when DK2 tracking is verified.");
}

void OrbitCameraSetupStage::onKeyDown(SDL_Keycode keyCode)
{
    if (keyCode == SDLK_SPACE)
    {
        m_app->setAppStage(_appStagePSMoveSetup);
    }
}

//-- AppStage : PSMoveSetupStage -----
void PSMoveSetupStage::enter()
{
    m_app->getFixedCamera()->setCameraOrbitLocation(0.f, 0.f, 150.f); // yaw degrees, pitch degrees, radius cm
    m_app->setCameraType(_cameraFixed);

    m_psmoveSetupState= _PSMoveSetupWaitingToStart;
    m_hasRenderedStageAtLeastOnce= false;
}

void PSMoveSetupStage::update()
{
    PSMoveContext *psmoveContext= m_app->getPSMoveContext();

    switch(m_psmoveSetupState)
    {
    case _PSMoveSetupConnectPSMove:
        setPSMoveSetupState(psmoveContext->connectController() ? _PSMoveSetupConnectTracker : _PSMoveSetupConnectPSMoveFailed);
        break;
    case _PSMoveSetupConnectTracker:
        if (m_hasRenderedStageAtLeastOnce)
        {
            // initTracker is a long blocking call. 
            // Make sure the state has had a change to render the ui once
            setPSMoveSetupState(psmoveContext->initTracker() ? _PSMoveSetupCalibrateTracker : _PSMoveSetupConnectTrackerFailed);
        }
        break;
    case _PSMoveSetupCalibrateTracker:
        if (m_hasRenderedStageAtLeastOnce)
        {
            // calibrateTracker is a long blocking call.
            // Make sure the state has had a change to render the ui once
            setPSMoveSetupState(psmoveContext->calibrateTracker() ? _PSMoveSetupSucceeded : _PSMoveSetupCalibrateTrackerFailed);
        }
        break;
    }
}

void PSMoveSetupStage::render()
{
    DK2Context *dk2Context= m_app->getDK2Context();
    glm::mat4 rotateX90= glm::rotate(glm::mat4(1.f), 90.f, glm::vec3(1.f, 0.f, 0.f));

    switch(m_psmoveSetupState)
    {
    case _PSMoveSetupWaitingToStart:
        drawPSMoveModel(rotateX90, glm::vec3(0.5f, 0.5f, 0.5f));
        break;
    case _PSMoveSetupConnectPSMove:
    case _PSMoveSetupConnectTracker:
    case _PSMoveSetupCalibrateTracker:
        drawPSMoveModel(rotateX90, glm::vec3(0.9f, 0.9f, 0.f));
        break;
    case _PSMoveSetupConnectPSMoveFailed:
    case _PSMoveSetupConnectTrackerFailed:
    case _PSMoveSetupCalibrateTrackerFailed:
        drawPSMoveModel(rotateX90, glm::vec3(0.9f, 0.0f, 0.0f));
        break;
    case _PSMoveSetupSucceeded:
        drawPSMoveModel(rotateX90, glm::vec3(0.f, 0.9f, 0.0f));
        break;
    }

    m_hasRenderedStageAtLeastOnce= true;
}

void PSMoveSetupStage::renderUI()
{
    PSMoveContext *psmoveContext= m_app->getPSMoveContext();
    Renderer *renderer= m_app->getRenderer();

    renderer->renderText(340, 20, "PSMove Setup");
    renderer->renderText(200, 40, "Hold the psmove in view of the camera");

    switch(m_psmoveSetupState)
    {
    case _PSMoveSetupWaitingToStart:
        renderer->renderText(10, 570, "Press space when ready.");
        break;
    case _PSMoveSetupConnectPSMove:
        renderer->renderText(10, 570, "Connecting to PSMove controller...");
        break;
    case _PSMoveSetupConnectTracker:
        renderer->renderText(10, 570, "Calibrating PS3Eye camera exposure...");
        break;
    case _PSMoveSetupCalibrateTracker:
        renderer->renderText(10, 570, "Calibrating tracking color");
        break;
    case _PSMoveSetupConnectPSMoveFailed:
    case _PSMoveSetupConnectTrackerFailed:
        renderer->renderText(10, 550, psmoveContext->getLastErrorMessage());
        renderer->renderText(10, 570, "Press space to retry.");
        break;
    case _PSMoveSetupCalibrateTrackerFailed:
        renderer->renderText(10, 550, psmoveContext->getLastErrorMessage());
        renderer->renderText(10, 570, "Press space to retry blink calibration. Press backspace to reset the camera.");
        break;
    case _PSMoveSetupSucceeded:
        renderer->renderText(10, 550, "Calibration succeeded! Press space to continue to coregistration.");
        renderer->renderText(10, 570, "Press enter to skip to testing.");
        break;
    }
}

void PSMoveSetupStage::onKeyDown(SDL_Keycode keyCode)
{
    switch(m_psmoveSetupState)
    {
    case _PSMoveSetupWaitingToStart:
        if (keyCode == SDLK_SPACE)
        {
            setPSMoveSetupState(_PSMoveSetupConnectPSMove);
        }
        break;
    case _PSMoveSetupConnectPSMove:
    case _PSMoveSetupConnectTracker:
    case _PSMoveSetupCalibrateTracker:
        // Ignore the key press 
        break;
    case _PSMoveSetupConnectPSMoveFailed:
        if (keyCode == SDLK_SPACE)
        {
            setPSMoveSetupState(_PSMoveSetupConnectPSMove);
        }
        break;
    case _PSMoveSetupConnectTrackerFailed:
        if (keyCode == SDLK_SPACE)
        {
            setPSMoveSetupState(_PSMoveSetupConnectTracker);
        }
        break;
    case _PSMoveSetupCalibrateTrackerFailed:
        if (keyCode == SDLK_SPACE)
        {
            setPSMoveSetupState(_PSMoveSetupCalibrateTracker);
        }
        else if (keyCode == SDLK_BACKSPACE)
        {
            setPSMoveSetupState(_PSMoveSetupConnectTracker);
        }
        break;
    case _PSMoveSetupSucceeded:
        if (keyCode == SDLK_SPACE)
        {
            m_app->setAppStage(_appStageComputeCoreg);
        }
        else if (keyCode == SDLK_RETURN)
        {
            m_app->setAppStage(_appStateTestCoreg);
        }
        break;
    }
}

//-- AppStage : ComputeCoregistrationStage -----
bool ComputeCoregistrationStage::init(int argc, char** argv)
{
    m_argc= argc;
    m_argv= argv;

    return true;
}

void ComputeCoregistrationStage::enter()
{
    m_app->getFixedCamera()->setCameraOrbitLocation(0.f, 0.f, 150.f); // yaw degrees, pitch degrees, radius cm
    m_app->setCameraType(_cameraFixed);

    m_coregState= _CoregStageAttachPSMoveToDK2;
    m_poseCount= 0;

	m_coregTransform = glm::mat4(1.f);
}

void ComputeCoregistrationStage::update()
{
    switch(m_coregState)
    {
    case _CoregStageAttachPSMoveToDK2:
    case _CoregStageComplete:
        // Do nothing
        break;
    case _CoregStageSampling:
        {
            DK2Context *dk2Context= m_app->getDK2Context();
            PSMoveContext *psMoveContext= m_app->getPSMoveContext();
            bool loadedDK2CameraInv44= false;

            if (m_poseCount == 0)
            {
                if (m_argc >= 2 && m_argc <= 3)
                {
                    loadedDK2CameraInv44= loadDK2CameraInv44(m_argv[1], &m_camera_invxform);
                }

                if (m_argc == 3 && loadedDK2CameraInv44)
                {
                    TrackingCameraFrustum frustum; 
                    OVR::Posef dk2pose;               // The DK2 pose
                    OVR::Posef psmovepose;            // The psmove pose

                    char *input_fpath = psmove_util_get_file_path(m_argv[2]);
                    FILE *recorded_pose_stream= recorded_pose_stream= initRecordedPoseStream(input_fpath);
                    free(input_fpath);

                    // Reset the sample bounds
                    dk2Context->getTrackingCameraFrustum(frustum);
                    frustumBoundsInit(
                        frustum.origin, frustum.forward, frustum.left, frustum.up, 
                        m_sampleBounds);

                    while (m_poseCount < NPOSES && 
                            readNextRecordedPoseStreamLine(recorded_pose_stream, &psmovepose, &dk2pose))
                    {
                        glm::vec3 dk2position= ovrVector3ToGlmVec3(dk2pose.Translation);
                        PSMove_3AxisVector psmoveposition;

                        psmoveposition.x= psmovepose.Translation.x;
                        psmoveposition.y= psmovepose.Translation.y;
                        psmoveposition.z= psmovepose.Translation.z;

                        m_dk2poses[m_poseCount]= dk2pose;
                        m_psmoveposes[m_poseCount]= psmoveposition;
                        m_poseCount++;

                        frustumBoundsEnclosePoint(dk2position, m_sampleBounds);
                    }

                    if (recorded_pose_stream != NULL)
                    {
                        closeRecordedPoseStream(recorded_pose_stream);
                    }
                }
            }

            if (m_poseCount == 0)
            {
                TrackingCameraFrustum frustum;

                // Reset the sample bounds
                dk2Context->getTrackingCameraFrustum(frustum);
                frustumBoundsInit(
                    frustum.origin, frustum.forward, frustum.left, frustum.up, 
                    m_sampleBounds);

                if (!loadedDK2CameraInv44)
                {
                    // Compute current camera pose inverse if this is the first sample
                    m_camera_invxform = dk2Context->getDK2CameraInv44();
                }
            }

            if (psMoveContext->getTrackingStatus() == Tracker_TRACKING && m_poseCount < NPOSES)
            {
                OVR::Posef dk2pose= dk2Context->getHMDPose();
                glm::vec3 dk2position= ovrVector3ToGlmVec3(dk2pose.Translation);
                PSMove_3AxisVector psmovepose= psMoveContext->getTrackerPosition();

                m_dk2poses[m_poseCount]= dk2pose;
                m_psmoveposes[m_poseCount]= psmovepose;
                m_poseCount++;

                frustumBoundsEnclosePoint(dk2position, m_sampleBounds);
            }

            if (m_poseCount >= NPOSES)
            {
				computeAndSaveCoregistrationTransform(m_camera_invxform, m_dk2poses, m_psmoveposes, NPOSES, m_coregTransform);
                m_coregState= _CoregStageComplete;
            }
        }
        break;
    }
}

void ComputeCoregistrationStage::render()
{
    switch(m_coregState)
    {
    case _CoregStageAttachPSMoveToDK2:
        {
            glm::mat4 rotateX90= glm::rotate(glm::mat4(1.f), 90.f, glm::vec3(1.f, 0.f, 0.f));

            // Offset the models (in cm) just enough so that they look like they are attached
            drawDK2Model(glm::translate(glm::mat4(1.f), glm::vec3(-9.f, 0.f, 0.f)));
            drawPSMoveModel(glm::translate(glm::mat4(1.f), glm::vec3(2.3f, 9.f, 0.f)) * rotateX90, glm::vec3(1.f, 1.f, 1.f));
        }
        break;
    case _CoregStageSampling:
    case _CoregStageComplete:
        {
            DK2Context *dk2Context= m_app->getDK2Context();
            PSMoveContext *psMoveContext= m_app->getPSMoveContext();

            // Draw the origin axes
            drawTransformedAxes(glm::mat4(1.0f), 100.f);

            // Draw a line strip connecting all of the dk2 positions collected so far
            if (m_poseCount > 0)
            {
                drawDK2Samples(m_dk2poses, m_poseCount);
                //drawPSMoveSamples(m_psmoveposes, m_poseCount);
            }

            // Draw a frustum bounding box of the samples
            drawFrustumBounds(m_sampleBounds, glm::vec3(0.f, 1.f, 0.f));

            // Draw the frustum for the DK2 camera
            {
                TrackingCameraFrustum frustum;

                dk2Context->getTrackingCameraFrustum(frustum);
                drawTrackingFrustum(frustum, k_dk2_frustum_color);
            }

            // Draw the DK2 model
            {
                glm::mat4 transform= ovrMatrix4fToGlmMat4(dk2Context->getHMDTransform());

                drawDK2Model(transform);
                drawTransformedAxes(transform, 10.f);
            }
        }
        break;
    }
}

void ComputeCoregistrationStage::renderUI()
{
    Renderer *renderer= m_app->getRenderer();

    renderer->renderText(340, 20, "Coregistration");

    switch(m_coregState)
    {
    case _CoregStageAttachPSMoveToDK2:
        renderer->renderText(200, 40, "Attach the PSMove to the side of the DK2");
        renderer->renderText(10, 570, "Press space when ready to sample poses.");
        break;
    case _CoregStageSampling:
        renderer->renderText(200, 40, "Sweep DK2+PSMove around the frustum.");
        renderer->renderText(200, 60, "Try to cover as much area as possible.");
        break;
    case _CoregStageComplete:
        renderer->renderText(200, 40, "Coregistration complete.");
        renderer->renderText(200, 60, "Detach psmove from the DK2.");
        renderer->renderText(10, 570, "Press space to continue or backspace to redo coregistration.");
        break;
    }
}

void ComputeCoregistrationStage::onKeyDown(SDL_Keycode keyCode)
{
    switch(m_coregState)
    {
    case _CoregStageAttachPSMoveToDK2:
        if (keyCode == SDLK_SPACE)
        {
            m_app->setCameraType(_cameraOrbit);
            m_coregState= _CoregStageSampling;
        }
        break;
    case _CoregStageSampling:
        break;
    case _CoregStageComplete:
        if (keyCode == SDLK_SPACE)
        {
            m_app->setAppStage(_appStateTestCoreg);
        }
        else if (keyCode == SDLK_BACKSPACE)
        {
            m_poseCount= 0;
            m_coregState= _CoregStageSampling;
			m_coregTransform = glm::mat4(1.f);
        }
        break;
    }
}

//-- AppStage : TestCoregistrationStage -----
void TestCoregistrationStage::enter()
{
    glm::mat4 coreg_transform;

    m_app->setCameraType(_cameraOrbit);

    if (m_app->getComputeCoregistrationStage()->getCoregistrationTransform(coreg_transform))
    {
        m_app->getPSMoveContext()->initFusionWithCoregTransform(coreg_transform);
    }
    else
    {
        m_app->getPSMoveContext()->initFusion();
    }
}

void TestCoregistrationStage::render()
{
    DK2Context *dk2Context= m_app->getDK2Context();
    PSMoveContext *psMoveContext= m_app->getPSMoveContext();

    // Draw the origin axes
    drawTransformedAxes(glm::mat4(1.0f), 100.f);

    // Draw the frustum for the DK2 camera
    {
        TrackingCameraFrustum frustum;

        dk2Context->getTrackingCameraFrustum(frustum);
        drawTrackingFrustum(frustum, k_dk2_frustum_color);
    }

    // Draw the frustum for the PSMove camera
    {
        TrackingCameraFrustum frustum;

        psMoveContext->getTrackingCameraFrustum(dk2Context, frustum);
        drawTrackingFrustum(frustum, k_psmove_frustum_color);
    }

    // Draw the DK2 model
    {
        glm::mat4 transform= ovrMatrix4fToGlmMat4(dk2Context->getHMDTransform());

        drawDK2Model(transform);
        drawTransformedAxes(transform, 10.f);
    }

    // Draw the psmove model
    {
        glm::mat4 dk2CameraToWorldTransform= ovrMatrix4fToGlmMat4(dk2Context->getCameraTransform());
        glm::mat4 worldTransform= psMoveContext->computeWorldTransform(dk2CameraToWorldTransform);

        drawPSMoveModel(worldTransform, glm::vec3(1.f, 1.f, 1.f));
        drawTransformedAxes(worldTransform, 10.f);
    }
}

void TestCoregistrationStage::renderUI()
{
    Renderer *renderer= m_app->getRenderer();

    renderer->renderText(340, 20, "Testing");
    renderer->renderText(10, 570, "Press escape to exit or backspace to redo coregistration.");
}

void TestCoregistrationStage::onKeyDown(SDL_Keycode keyCode)
{
    if (keyCode == SDLK_BACKSPACE)
    {
        m_app->setAppStage(_appStageComputeCoreg);
    }
}

//-- PSMoveContext -----
PSMoveContext::PSMoveContext() 
    : m_psmoveapi_initialized(false)
    , m_move(NULL)
    , m_tracker(NULL)
    , m_fusion(NULL)
    , m_PS3EyeToDK2CameraXform(1.f)
    , m_tracking_status(Tracker_NOT_CALIBRATED)
    , m_buttons_down(0)
    , m_buttons_pressed(0)
    , m_buttons_released(0)
    , m_camera_api(PSMove_Camera_API_PS3EYE_LIBUSB)
    , m_use_custom_tracking_color(false)
    , m_custom_r(0)
    , m_custom_g(0)
    , m_custom_b(0)
    , m_lastErrorMessage(NULL)
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

    if (success) 
    {
        m_camera_api= PSMove_Camera_API_PS3EYE_LIBUSB;

        if (argc >= 2)
        {
            if (strcmp(argv[1], "libusb") == 0)
            {
                m_camera_api= PSMove_Camera_API_PS3EYE_LIBUSB;
            }
            else if (strcmp(argv[1], "cleye") == 0)
            {
                m_camera_api= PSMove_Camera_API_PS3EYE_CLEYE;
            }
            else if (strcmp(argv[1], "opencv") == 0)
            {
                m_camera_api= PSMove_Camera_API_OPENCV;
            }
        }

        if (argc >= 5)
        {
            m_custom_r= (unsigned char)atoi(argv[1]);
            m_custom_g= (unsigned char)atoi(argv[2]);
            m_custom_b= (unsigned char)atoi(argv[3]);
            m_use_custom_tracking_color= true;

            Log_INFO("PSMoveContext::init()", "Setting LEDS for controller 1 from command-line r: %i, g: %i, b: %i", 
                m_custom_r, m_custom_g, m_custom_b);
        }
    }

    return success;
}

bool PSMoveContext::initTracker()
{
    bool success = true;

    m_lastErrorMessage= NULL;

    if (m_tracker != NULL)
    {
        Log_INFO("PSMoveContext::init()", "Tracker already initialized. Turning off first.");
        psmove_tracker_free(m_tracker);
        m_tracker= NULL;
    }

    Log_INFO("PSMoveContext::init()", "Turning on PSMove Tracking Camera");
    if (success)
    {
        PSMoveTrackerSettings settings;
        psmove_tracker_settings_set_default(&settings);
        settings.color_mapping_max_age = 0;
        settings.exposure_mode = Exposure_MANUAL;
        settings.camera_mirror = PSMove_True;
        settings.camera_type= PSMove_Focal_Length_PS3EYE_BLUEDOT; // Wider FOV
        settings.camera_api= m_camera_api;
        settings.use_fitEllipse = 1;

        m_tracker = psmove_tracker_new_with_settings(&settings);
        if (m_tracker != NULL) 
        {
            Log_INFO("PSMoveContext::init()", "Tracking camera initialized");

            // Make sure NOT to use the positional Kalman filter since it adds a 
            // time latency to the psmove positional sampling that skews
            // the coregistration transform in wierd ways.
            PSMoveTrackerSmoothingSettings smoothing_settings;
            psmove_tracker_get_smoothing_settings(m_tracker, &smoothing_settings);
            smoothing_settings.filter_do_2d_r = 0;
            smoothing_settings.filter_do_2d_xy = 0;
            smoothing_settings.filter_3d_type = Smoothing_LowPass;
            psmove_tracker_set_smoothing_settings(m_tracker, &smoothing_settings);
        }
        else
        {
            setLastErrorMessage("No tracker available! (Missing PS3Eye camera?)");
            success= false;
        }
    }

    return success;
}

bool PSMoveContext::connectController()
{
    bool success= true;

    m_lastErrorMessage= NULL;

    if (m_move == NULL)
    {
        Log_INFO("PSMoveContext::init()", "Connecting ");
        if (success && psmove_count_connected() < 1)
        {
            setLastErrorMessage("No PSMove controllers connected!");
            success = false;
        }

        if (success)
        {
            m_move = psmove_connect();

            if (m_move == NULL)
            {
                setLastErrorMessage("Failed to connect psmove controller. Is it turned on?");
                success= false;
            }
        }

        if (success && !psmove_has_calibration(m_move))
        {
            setLastErrorMessage("Controller had invalid USB calibration blob. Re-Pair with the PC?");
            success= false;
        }

        if (success)
        {
            psmove_enable_orientation(m_move, PSMove_True);  // Though we don't actually use it.

            if (psmove_has_orientation(m_move))
            {
                // The corresponds to default pose of the psmove model (which is aligned down the +z axis)
                psmove_set_calibration_transform(m_move, k_psmove_identity_pose_laying_flat);

                // Put the orientation data in the OpenGL coordinate system
                psmove_set_sensor_data_transform(m_move, k_psmove_sensor_transform_opengl);
            }
            else
            {
                setLastErrorMessage("Failed to initialize PSMove orientation update.");
                success= false;
            }
        }
    }
    else
    {
        Log_INFO("PSMoveContext::init()", "PSMove controller already connected.");
    }

    if (!success && m_move != NULL)
    {
        psmove_disconnect(m_move);
        m_move= NULL;
    }

    return success;
}

bool PSMoveContext::calibrateTracker()
{
    bool success = true;
    enum PSMoveTracker_Status tracking_status = Tracker_NOT_CALIBRATED;

    m_lastErrorMessage= NULL;
    Log_INFO("PSMoveContext::init()", "Calibrating PSMove tracking color...");

    if (m_use_custom_tracking_color) 
    {
        Log_INFO("PSMoveContext::init()", "Setting LEDS for controller 1 from command-line r: %i, g: %i, b: %i", 
            m_custom_r, m_custom_g, m_custom_b);
        tracking_status = psmove_tracker_enable_with_color(m_tracker, m_move, m_custom_r, m_custom_g, m_custom_b);
    }
    else 
    {
        tracking_status = psmove_tracker_enable(m_tracker, m_move);
    }
            
    if (tracking_status == Tracker_CALIBRATED) 
    {
        Log_INFO("PSMoveContext::init()", "Successfully calibrated tracking color");
    } 
    else 
    {
        setLastErrorMessage("Failed to calibrate color. Is PSMove in frame?");
        success= false;
    }

    return success;
}

void PSMoveContext::initFusion()
{
    assert(m_tracker);

    if (m_fusion != NULL)
    {
        psmove_fusion_free(m_fusion);
    }

    // Setup the fusion tracker system. This also loads the physical transform from file if present.
    m_fusion = psmove_fusion_new(m_tracker, 1., 1000.);

    // Extract the co registration transform
	m_PS3EyeToDK2CameraXform= glm::make_mat4(psmove_fusion_get_coregistration_matrix(m_fusion));
}

void PSMoveContext::initFusionWithCoregTransform(
	const glm::mat4 &coregistrationTransform)
{
    initFusion();
    m_PS3EyeToDK2CameraXform = coregistrationTransform;
}

glm::mat4 PSMoveContext::computeWorldTransform(const glm::mat4 &dk2CameraToWorldTransform) const
{
    assert(m_fusion);
    assert(m_move);

    // Get the orientation of the controller in world space (OpenGL Coordinate System)
    glm::quat q;
    psmove_get_orientation(m_move, &q.w, &q.x, &q.y, &q.z);
    glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);

    // Convert the position provided by the fusion api in dk2 camera space to world space
    glm::vec4 dk2CameraSpaceTranslation(1.f);
    psmove_fusion_get_transformed_location(m_fusion, m_move, 
        &dk2CameraSpaceTranslation.x, &dk2CameraSpaceTranslation.y, &dk2CameraSpaceTranslation.z);
    glm::mat4 worldSpaceTranslation= 
        glm::translate(glm::mat4(1.f), glm::vec3(dk2CameraToWorldTransform * dk2CameraSpaceTranslation));

    // Return a transform that merges together the world space orientation and translation
    return worldSpaceTranslation * worldSpaceOrientation;
}

void PSMoveContext::getTrackingCameraFrustum(
    const class DK2Context *dk2Context, 
    TrackingCameraFrustum &frustum) const
{
    glm::mat4 dk2CameraToWorld= ovrMatrix4fToGlmMat4(dk2Context->getCameraTransform());
    glm::mat4 psmoveCameraToWorld= dk2CameraToWorld * m_PS3EyeToDK2CameraXform;

    frustum.origin= glm::vec3(psmoveCameraToWorld[3]);

    frustum.forward= glm::vec3(psmoveCameraToWorld[2]); // z-axis
    frustum.left= glm::vec3(psmoveCameraToWorld[0]); // x-axis
    frustum.up= glm::vec3(psmoveCameraToWorld[1]); // y-axis

    {
        PSMoveTrackerSettings settings;

        psmove_tracker_get_settings(m_tracker, &settings);

        switch(settings.camera_type)
        {
        case PSMove_Focal_Length_PS3EYE_BLUEDOT:
        default:
            frustum.HFOV= glm::radians(60.f);
            frustum.VFOV= glm::radians(45.f);
            break;
        case PSMove_Focal_Length_PS3EYE_REDDOT:
            frustum.HFOV= glm::radians(56.f);
            frustum.VFOV= glm::radians(56.f);
            break;
        }

        // Outside of these distances psmove tracking isn't very good
        frustum.zNear= 10.f; // cm
        frustum.zFar= 200.f; // cm
    }
}

void PSMoveContext::setLastErrorMessage(const char *errorMessage)
{
    m_lastErrorMessage= errorMessage;
    Log_ERROR("PSMoveContext", "%s", errorMessage);
}

void PSMoveContext::destroy()
{
    if (m_move != NULL)
    {
        psmove_disconnect(m_move);
        m_move= NULL;
    }

    if (m_fusion != NULL)
    {
        psmove_fusion_free(m_fusion);
        m_fusion= NULL;
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
    if (m_tracker != NULL && m_move != NULL)
    {
        psmove_tracker_update_image(m_tracker);
        psmove_tracker_update(m_tracker, m_move);
        m_tracking_status = psmove_tracker_get_status(m_tracker, m_move);

        while (psmove_poll(m_move));
    
        psmove_tracker_get_location(m_tracker, m_move, 
            &m_tracker_position.x, &m_tracker_position.y, &m_tracker_position.z);

        m_buttons_down = psmove_get_buttons(m_move);
        psmove_get_button_events(m_move, &m_buttons_pressed, &m_buttons_released);
    }
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

#if defined(OVR_OS_WIN32)
    if (success)
    {
        m_HMDDesc= ovr_GetHmdDesc(m_HMD);
    }
#endif

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
#ifdef _WIN32
	m_dk2state = ovr_GetTrackingState(m_HMD, 0.0, ovrFalse);
#else
    m_dk2state = ovr_GetTrackingState(m_HMD, 0.0);
#endif
    m_dk2pose = m_dk2state.HeadPose.ThePose;
    m_dk2pose.Rotation.Normalize();
    m_dk2pose.Translation *= METERS_TO_CENTIMETERS;
    m_dk2mat= OVR::Matrix4f(m_dk2pose);
}

void DK2Context::getTrackingCameraFrustum(
    TrackingCameraFrustum &frustum) const
{
    const ovrPosef &cameraPose= m_dk2state.CameraPose;
    const ovrQuatf &q= m_dk2state.CameraPose.Orientation;
    OVR::Matrix3f cameraMatrix(OVR::Quatf(q.x, q.y, q.z, q.w));

    frustum.origin= ovrVector3ToGlmVec3(cameraPose.Position);
    frustum.origin*= METERS_TO_CENTIMETERS;

    frustum.forward= ovrVector3ToGlmVec3(cameraMatrix.Col(OVR::Axis_Z));
    frustum.left= ovrVector3ToGlmVec3(cameraMatrix.Col(OVR::Axis_X));
    frustum.up= ovrVector3ToGlmVec3(cameraMatrix.Col(OVR::Axis_Y));

#if defined(OVR_OS_WIN32)
    frustum.HFOV= m_HMDDesc.CameraFrustumHFovInRadians;
    frustum.VFOV= m_HMDDesc.CameraFrustumVFovInRadians;
    frustum.zNear= m_HMDDesc.CameraFrustumNearZInMeters*METERS_TO_CENTIMETERS;
    frustum.zFar= m_HMDDesc.CameraFrustumFarZInMeters*METERS_TO_CENTIMETERS;
#else
    frustum.HFOV= m_HMD->CameraFrustumHFovInRadians;
    frustum.VFOV= m_HMD->CameraFrustumVFovInRadians;
    frustum.zNear= m_HMD->CameraFrustumNearZInMeters*METERS_TO_CENTIMETERS;
    frustum.zFar= m_HMD->CameraFrustumFarZInMeters*METERS_TO_CENTIMETERS;
#endif
}

OVR::Posef DK2Context::getHMDPose() const
{
    return m_dk2pose;
}

OVR::Matrix4f DK2Context::getHMDTransform() const
{
    return m_dk2mat;
}

OVR::Matrix4f DK2Context::getCameraTransform() const
{
    const ovrQuatf &q= m_dk2state.CameraPose.Orientation;
    const ovrVector3f &v= m_dk2state.CameraPose.Position;
    OVR::Posef cameraPose(OVR::Quatf(q.x, q.y, q.z, q.w), OVR::Vector3f(v.x, v.y, v.z) * METERS_TO_CENTIMETERS);

    return OVR::Matrix4f(cameraPose);
}

OVR::Matrix4f DK2Context::getDK2CameraInv44() const
{
    OVR::Posef campose(m_dk2state.CameraPose);
    campose.Rotation.Normalize();  // Probably does nothing as the SDK returns normalized quats anyway.
    campose.Translation *= METERS_TO_CENTIMETERS;
    
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

        setProjectionMatrix(
            glm::perspective(k_camera_vfov, getWindowAspectRatio(), k_camera_z_near, k_camera_z_far));

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

void Renderer::renderText(float x, float y, const char *text)
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

//-- Orbit Camera -----
void Camera::onMouseMotion(int deltaX, int deltaY)
{
    if (!m_isLocked && m_isPanningOrbitCamera)
    {
        const float deltaYaw= -(float)deltaX * k_camera_mouse_pan_scalar;
        const float deltaPitch= (float)deltaY * k_camera_mouse_pan_scalar;

        setCameraOrbitLocation(
            m_cameraOrbitYawDegrees+deltaYaw, 
            m_cameraOrbitPitchDegrees+deltaPitch, 
            m_cameraOrbitRadius);
    }
}

void Camera::onMouseButtonDown(int buttonIndex)
{
    if (!m_isLocked && buttonIndex == SDL_BUTTON_LEFT)
    {
        m_isPanningOrbitCamera= true;
    }
}

void Camera::onMouseButtonUp(int buttonIndex)
{
    if (!m_isLocked && buttonIndex == SDL_BUTTON_LEFT)
    {
        m_isPanningOrbitCamera= false;
    }
}

void Camera::onMouseWheel(int scrollAmount)
{
    if (!m_isLocked)
    {
        const float deltaRadius= (float)scrollAmount * k_camera_mouse_zoom_scalar;

        setCameraOrbitLocation(
            m_cameraOrbitYawDegrees, 
            m_cameraOrbitPitchDegrees, 
            m_cameraOrbitRadius+deltaRadius);
    }
}

void Camera::setIsLocked(bool locked)
{
    if (locked)
    {
        m_isLocked= true;
        m_isPanningOrbitCamera= false;
    }
    else
    {
        m_isLocked= false;
    }
}

void Camera::setCameraOrbitLocation(float yawDegrees, float pitchDegrees, float radius)
{
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

    publishCameraViewMatrix();
}

void Camera::publishCameraViewMatrix()
{
    Renderer *renderer= m_app->getRenderer();

    renderer->setCameraViewMatrix(
        glm::lookAt(
            m_cameraPosition,
            glm::vec3(0, 0, 0), // Look at tracking origin
            glm::vec3(0, 1, 0)));    // Up is up.
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

//-- math helper functions -----
void computeAndSaveCoregistrationTransform(
    const OVR::Matrix4f &camera_invxform,
    const OVR::Posef *dk2poses,
    const PSMove_3AxisVector *psmoveposes,
    int poseCount,
	glm::mat4 &outCoregTransform)
{
    Eigen::MatrixXf A(poseCount * 3, 15);  // X = A/b
    Eigen::VectorXf b(poseCount * 3);

    // Build the A matrix and the b colomn vector from the given poses
    {
        Eigen::Matrix4f dk2eig;             // DK2 pose in Eigen 4x4 mat
        Eigen::Matrix3f RMi;                // Transpose of inner 3x3 of DK2 pose

        // Print the column headers
        char *output_fpath = psmove_util_get_file_path("output.txt");
        FILE *output_fp = fopen(output_fpath, "w");
        free(output_fpath);
        fprintf(output_fp, "psm_px,psm_py,psm_pz,psm_ow,psm_ox,psm_oy,psm_oz,dk2_px,dk2_py,dk2_pz,dk2_ow,dk2_ox,dk2_oy,dk2_oz\n");

        for (int poseIndex= 0; poseIndex < poseCount; ++poseIndex)
        {
            PSMove_3AxisVector psmove_pos= psmoveposes[poseIndex];
            OVR::Posef psmovepose(OVR::Quatf::Identity(), OVR::Vector3f(psmove_pos.x, psmove_pos.y, psmove_pos.z));
            OVR::Posef dk2pose= dk2poses[poseIndex];

            fprintf(output_fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                psmovepose.Translation.x, psmovepose.Translation.y, psmovepose.Translation.z,
                psmovepose.Rotation.w, psmovepose.Rotation.x, psmovepose.Rotation.y, psmovepose.Rotation.z,
                dk2pose.Translation.x, dk2pose.Translation.y, dk2pose.Translation.z,
                dk2pose.Rotation.w, dk2pose.Rotation.x, dk2pose.Rotation.y, dk2pose.Rotation.z);

            OVR::Matrix4f dk2mat = OVR::Matrix4f(dk2pose);
            dk2mat = camera_invxform * dk2mat;  // Make the camera pose the new origin, so dk2 is returned relative to that.
            OVR::Matrix4f psmovemat = OVR::Matrix4f(psmovepose);

            ovrMatrix4ToEigenMatrix4(dk2mat, dk2eig);
            RMi = dk2eig.topLeftCorner(3, 3).transpose();           // inner 33 transposed

            A.block<3, 3>(poseIndex * 3, 0) = RMi * psmovemat.M[0][3];
            A.block<3, 3>(poseIndex * 3, 3) = RMi * psmovemat.M[1][3];
            A.block<3, 3>(poseIndex * 3, 6) = RMi * psmovemat.M[2][3];
            A.block<3, 3>(poseIndex * 3, 9) = RMi;
            A.block<3, 3>(poseIndex * 3, 12) = -Eigen::Matrix3f::Identity();
            b.segment(poseIndex * 3, 3) = RMi * dk2eig.block<3, 1>(0, 3);
        }

        fclose(output_fp);
    }

    // Compute the coregistration transform and save to disk
    {
        Eigen::VectorXf x(15);
        x = A.colPivHouseholderQr().solve(b);
        Log_INFO("Coreg", "\nglobalxfm:\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
            x(0), x(3), x(6), x(9),
            x(1), x(4), x(7), x(10),
            x(2), x(5), x(8), x(11));
        Log_INFO("Coreg", "\nlocalxfm:\n%f,%f,%f\n", x(12), x(13), x(14));

        char *fpath = psmove_util_get_file_path("transform.csv");
        FILE *fp = fopen(fpath, "w");
        free(fpath);

        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
            x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11));

		outCoregTransform = glm::mat4(1.f);
		for (int i = 0; i < 12; i++)
		{
			int row_ix = i % 3;
			int col_ix = (float(i) / 3.0);
			outCoregTransform[col_ix][row_ix] = x(i);
		}

        fclose(fp);
    }
}

void frustumBoundsInit(
    const glm::vec3 &origin, const glm::vec3 &forward, const glm::vec3 &left, const glm::vec3 &up, 
    FrustumBounds &bounds)
{
    bounds.origin= origin;
    bounds.forward= forward;
    bounds.left= left;
    bounds.up= up;
    bounds.hAngleMin= k_real_pi;
    bounds.hAngleMax= -k_real_pi;
    bounds.vAngleMin= k_real_pi;
    bounds.vAngleMax= -k_real_pi;
    bounds.zNear= k_real_max;
    bounds.zFar= -k_real_max;
}

void frustumBoundsEnclosePoint(const glm::vec3 &point, FrustumBounds &bounds)
{
    glm::vec3 v= point - bounds.origin;
    float vAngle= atan2f(glm::dot(v, bounds.up), glm::dot(v, bounds.forward));
    float hAngle= atan2f(glm::dot(v, bounds.left), glm::dot(v, bounds.forward));
    float z= glm::dot(v, bounds.forward);

    if (z > k_real_epsilon)
    {
        bounds.hAngleMin= fmin(hAngle, bounds.hAngleMin);
        bounds.hAngleMax= fmax(hAngle, bounds.hAngleMax);
        bounds.vAngleMin= fmin(vAngle, bounds.vAngleMin);
        bounds.vAngleMax= fmax(vAngle, bounds.vAngleMax);
        bounds.zNear= fmin(z, bounds.zNear);
        bounds.zFar= fmax(z, bounds.zFar);
    }
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

//-- debug render helper functions -----
void drawTrackingFrustum(const TrackingCameraFrustum &frustum, const glm::vec3 &color)
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

    glColor3fv(glm::value_ptr(color));

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

void drawFrustumBounds(const FrustumBounds &frustum, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    if (frustum.hAngleMax > frustum.hAngleMin &&
        frustum.vAngleMax > frustum.vAngleMin &&
        frustum.zFar > frustum.zNear)
    {
        const float HMinRatio= tanf(frustum.hAngleMin);
        const float HMaxRatio= tanf(frustum.hAngleMax);
        const float VMinRatio= tanf(frustum.vAngleMin);
        const float VMaxRatio= tanf(frustum.vAngleMax);

        glm::vec3 nearXMin= frustum.left*frustum.zNear*HMinRatio;
        glm::vec3 farXMin= frustum.left*frustum.zFar*HMinRatio;
        glm::vec3 nearXMax= frustum.left*frustum.zNear*HMaxRatio;
        glm::vec3 farXMax= frustum.left*frustum.zFar*HMaxRatio;

        glm::vec3 nearYMin= frustum.up*frustum.zNear*VMinRatio;
        glm::vec3 farYMin= frustum.up*frustum.zFar*VMinRatio;
        glm::vec3 nearYMax= frustum.up*frustum.zNear*VMaxRatio;
        glm::vec3 farYMax= frustum.up*frustum.zFar*VMaxRatio;

        glm::vec3 nearZ= frustum.forward*frustum.zNear;
        glm::vec3 farZ= frustum.forward*frustum.zFar;

        glm::vec3 nearCenter= frustum.origin + nearZ;
        glm::vec3 near0= frustum.origin + nearXMax + nearYMax + nearZ;
        glm::vec3 near1= frustum.origin + nearXMin + nearYMax + nearZ;
        glm::vec3 near2= frustum.origin + nearXMin + nearYMin + nearZ;
        glm::vec3 near3= frustum.origin + nearXMax + nearYMin + nearZ;

        glm::vec3 far0= frustum.origin + farXMax + farYMax + farZ;
        glm::vec3 far1= frustum.origin + farXMin + farYMax + farZ;
        glm::vec3 far2= frustum.origin + farXMin + farYMin + farZ;
        glm::vec3 far3= frustum.origin + farXMax + farYMin + farZ;
    
        glBegin(GL_LINES);

        glColor3fv(glm::value_ptr(color));

        glVertex3fv(glm::value_ptr(near0)); glVertex3fv(glm::value_ptr(near1));
        glVertex3fv(glm::value_ptr(near1)); glVertex3fv(glm::value_ptr(near2));
        glVertex3fv(glm::value_ptr(near2)); glVertex3fv(glm::value_ptr(near3));
        glVertex3fv(glm::value_ptr(near3)); glVertex3fv(glm::value_ptr(near0));

        glVertex3fv(glm::value_ptr(far0)); glVertex3fv(glm::value_ptr(far1));
        glVertex3fv(glm::value_ptr(far1)); glVertex3fv(glm::value_ptr(far2));
        glVertex3fv(glm::value_ptr(far2)); glVertex3fv(glm::value_ptr(far3));
        glVertex3fv(glm::value_ptr(far3)); glVertex3fv(glm::value_ptr(far0));

        glVertex3fv(glm::value_ptr(near0)); glVertex3fv(glm::value_ptr(far0));
        glVertex3fv(glm::value_ptr(near1)); glVertex3fv(glm::value_ptr(far1));
        glVertex3fv(glm::value_ptr(near2)); glVertex3fv(glm::value_ptr(far2));
        glVertex3fv(glm::value_ptr(near3)); glVertex3fv(glm::value_ptr(far3));

        glEnd();
    }
}

void drawDK2Samples(const OVR::Posef *dk2poses, const int poseCount)
{
    glColor3ub(255, 255, 0);
    glBegin(GL_LINE_STRIP);

    for (int sampleIndex= 0; sampleIndex < poseCount; ++sampleIndex)
    {
        const OVR::Posef &pose= dk2poses[sampleIndex];        

        glVertex3f(pose.Translation.x, pose.Translation.y, pose.Translation.z);
    }

    glEnd();
}

void drawPSMoveSamples(const PSMove_3AxisVector *psmoveposes, const int poseCount)
{
    glColor3ub(0, 255, 0);
    glBegin(GL_LINE_STRIP);

    for (int sampleIndex= 0; sampleIndex < poseCount; ++sampleIndex)
    {
        const PSMove_3AxisVector &pose= psmoveposes[sampleIndex];        

        glVertex3f(pose.x, pose.y, pose.z);
    }

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

void drawPSMoveModel(const glm::mat4 &transform, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    //int textureID= AssetManager::getInstance()->getPSMoveTextureId();

    //glBindTexture(GL_TEXTURE_2D, textureID);
    glColor3fv(glm::value_ptr(color));

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glEnableClientState(GL_VERTEX_ARRAY);
        //glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, psmoveVerts);
        //glTexCoordPointer(2, GL_FLOAT, 0, DK2TexCoords);
        glDrawArrays(GL_TRIANGLES, 0, psmoveNumVerts);
        glDisableClientState(GL_VERTEX_ARRAY);
        //glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glPopMatrix();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

//-- debug logging -----
static bool loadDK2CameraInv44(const char *filename, OVR::Matrix4f *outCamMat) 
{
    OVR::Posef campose;
    bool success= false;

    char *fpath = psmove_util_get_file_path(filename);
    FILE *fp = fopen(fpath, "r");
    free(fpath);

    if (fp != NULL)
    {
        char line[1024];

        if (fgets(line, 1024, fp))
        {
            const int k_colomn_count= 7;
            float *target_value[k_colomn_count]= {
                &campose.Translation.x, &campose.Translation.y, &campose.Translation.z,
                &campose.Rotation.w, &campose.Rotation.x, &campose.Rotation.y, &campose.Rotation.z};
            int match_token_count= 0;

            for (const char *token= strtok(line, ","); token && *token; token = strtok(NULL, ",\n"))
            {
                *target_value[match_token_count]= atof(token);
                ++match_token_count;
            }

            success= match_token_count == k_colomn_count;
        }

        fclose(fp);
    }

    if (success)
    {
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

        *outCamMat= camMat;
    }

    return success;
}

static FILE *initRecordedPoseStream(const char *filename)
{
    FILE *stream= fopen(filename, "r");
    bool success= false;

    if (stream != NULL)
    {
        char line[1024];
        if (fgets(line, 1024, stream))
        {
            const int k_colomn_count= 14;
            const char headers[k_colomn_count][8]= {"psm_px","psm_py","psm_pz","psm_ow","psm_ox","psm_oy","psm_oz","dk2_px","dk2_py","dk2_pz","dk2_ow","dk2_ox","dk2_oy","dk2_oz"};
            int match_token_count= 0;

            for (const char *token= strtok(line, ","); token && *token; token = strtok(NULL, ",\n"))
            {
                if (strcmp(token, headers[match_token_count]) == 0)
                {
                    ++match_token_count;
                }
                else
                {
                    break;
                }
            }

            success= match_token_count == k_colomn_count;
        }
    }

    if (!success)
    {
        if (stream != NULL)
        {
            fclose(stream);
        }

        stream= NULL;
    }

    return stream;
}

static bool readNextRecordedPoseStreamLine(FILE *stream, OVR::Posef *out_psmovepose, OVR::Posef *out_dk2pose)
{
    bool success= false;

    if (stream != NULL)
    {
        char line[1024];

        if (fgets(line, 1024, stream))
        {
            const int k_colomn_count= 14;
            float *target_value[k_colomn_count]= {
                &out_psmovepose->Translation.x, &out_psmovepose->Translation.y, &out_psmovepose->Translation.z,
                &out_psmovepose->Rotation.w, &out_psmovepose->Rotation.x, &out_psmovepose->Rotation.y, &out_psmovepose->Rotation.z,
                &out_dk2pose->Translation.x, &out_dk2pose->Translation.y, &out_dk2pose->Translation.z,
                &out_dk2pose->Rotation.w, &out_dk2pose->Rotation.x, &out_dk2pose->Rotation.y, &out_dk2pose->Rotation.z};
            int match_token_count= 0;

            for (const char *token= strtok(line, ","); token && *token; token = strtok(NULL, ",\n"))
            {
                *target_value[match_token_count]= atof(token);
                ++match_token_count;
            }

            success= match_token_count == k_colomn_count;
        }
    }

    return success;
}

static void closeRecordedPoseStream(FILE *stream)
{
    if (stream)
    {
        fclose(stream);
    }
}
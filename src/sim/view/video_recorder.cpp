/**
 * @file
 *
 * @brief Offscreen EGL video recorder piping raw frames to ffmpeg.
 */

#include "micras/sim/view/video_recorder.hpp"

#ifdef MICRAS_VIDEO

    #include <algorithm>
    #include <array>
    #include <format>
    #include <span>
    #include <string>
    #include <vector>

    #include <EGL/egl.h>
    #include <EGL/eglext.h>

namespace micras::sim {
namespace {
/**
 * @brief Framebuffer configuration MuJoCo needs from EGL.
 */
const std::array<EGLint, 17> config_attributes{
    EGL_RED_SIZE,
    8,
    EGL_GREEN_SIZE,
    8,
    EGL_BLUE_SIZE,
    8,
    EGL_ALPHA_SIZE,
    8,
    EGL_DEPTH_SIZE,
    24,
    EGL_STENCIL_SIZE,
    8,
    EGL_COLOR_BUFFER_TYPE,
    EGL_RGB_BUFFER,
    EGL_SURFACE_TYPE,
    EGL_PBUFFER_BIT,
    EGL_NONE
};

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast, misc-misplaced-const): the EGL C API hands out its
// extension entry points as void pointers and typedefs its handles as pointers.

/**
 * @brief Try to bring up a current OpenGL context on one EGL display.
 *
 * @param display Display to initialise.
 * @return True when the context is current and usable.
 */
bool make_context_current(EGLDisplay display) {
    if (display == EGL_NO_DISPLAY) {
        return false;
    }

    EGLint major = 0;
    EGLint minor = 0;

    if (eglInitialize(display, &major, &minor) != EGL_TRUE) {
        return false;
    }

    EGLint    count = 0;
    EGLConfig config{};

    if (eglChooseConfig(display, config_attributes.data(), &config, 1, &count) != EGL_TRUE or count < 1) {
        eglTerminate(display);
        return false;
    }

    if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) {
        eglTerminate(display);
        return false;
    }

    const EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);

    if (context == EGL_NO_CONTEXT) {
        eglTerminate(display);
        return false;
    }

    if (eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context) != EGL_TRUE) {
        eglDestroyContext(display, context);
        eglTerminate(display);
        return false;
    }

    return true;
}

/**
 * @brief Make a headless OpenGL context current.
 *
 * @note The default display works on most drivers; when it does not (a common
 *       failure with several EGL vendors installed) every enumerated EGL device
 *       is tried in turn, which is what picks the NVIDIA one on this machine.
 *
 * @return True on success.
 */
bool init_opengl() {
    if (make_context_current(eglGetDisplay(EGL_DEFAULT_DISPLAY))) {
        return true;
    }

    auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
    auto get_platform_display =
        reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));

    if (query_devices == nullptr or get_platform_display == nullptr) {
        return false;
    }

    std::array<EGLDeviceEXT, 16> devices{};
    EGLint                       device_count = 0;

    if (query_devices(static_cast<EGLint>(devices.size()), devices.data(), &device_count) != EGL_TRUE) {
        return false;
    }

    for (EGLint i = 0; i < device_count; i++) {
        if (make_context_current(get_platform_display(EGL_PLATFORM_DEVICE_EXT, devices.at(i), nullptr))) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Release the current EGL context and its display.
 */
void close_opengl() {
    const EGLDisplay display = eglGetCurrentDisplay();

    if (display == EGL_NO_DISPLAY) {
        return;
    }

    const EGLContext context = eglGetCurrentContext();
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    if (context != EGL_NO_CONTEXT) {
        eglDestroyContext(display, context);
    }

    eglTerminate(display);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast, misc-misplaced-const)

/**
 * @brief Name of a Micras::State value, for the HUD.
 *
 * @param state Numeric state as reported by the firmware pool.
 * @return State name, or "?" when out of range.
 */
const char* state_name(double state) {
    const std::array<const char*, 7> names{"INIT",      "IDLE", "WAIT_FOR_RUN", "RUN", "WAIT_FOR_CALIBRATE",
                                           "CALIBRATE", "ERROR"};
    const int                        index = static_cast<int>(state);

    if (index < 0 or index >= static_cast<int>(names.size())) {
        return "?";
    }

    return names.at(static_cast<std::size_t>(index));
}
}  // namespace

std::unique_ptr<VideoRecorder> VideoRecorder::create(mjModel* model, const VideoConfig& config, std::string& error) {
    if (not init_opengl()) {
        error = "could not create a headless EGL/OpenGL context";
        return nullptr;
    }

    std::unique_ptr<VideoRecorder> recorder(new VideoRecorder());
    recorder->model = model;
    recorder->config = config;

    mjv_defaultCamera(&recorder->camera);
    mjv_defaultOption(&recorder->option);
    mjv_defaultScene(&recorder->scene);
    mjr_defaultContext(&recorder->context);

    recorder->option.flags[mjVIS_RANGEFINDER] = 1;

    recorder->display = eglGetCurrentDisplay();
    recorder->gl_context = eglGetCurrentContext();

    mjv_makeScene(model, &recorder->scene, 2000);
    mjr_makeContext(model, &recorder->context, mjFONTSCALE_150);
    recorder->context_ready = true;

    if (recorder->context.offWidth < config.width or recorder->context.offHeight < config.height) {
        error = "offscreen framebuffer is " + std::to_string(recorder->context.offWidth) + "x" +
                std::to_string(recorder->context.offHeight) + ", smaller than the requested " +
                std::to_string(config.width) + "x" + std::to_string(config.height);
        return nullptr;
    }

    if (config.camera == "free") {
        mjv_defaultFreeCamera(model, &recorder->camera);
        recorder->camera.type = mjCAMERA_FREE;
        recorder->camera.lookat[0] = model->stat.center[0];
        recorder->camera.lookat[1] = model->stat.center[1];
        recorder->camera.lookat[2] = model->stat.center[2];
        recorder->camera.distance = 1.6 * model->stat.extent;
        recorder->camera.azimuth = 90.0;
        recorder->camera.elevation = -65.0;
    } else {
        const int id = mj_name2id(model, mjOBJ_CAMERA, config.camera.c_str());

        if (id < 0) {
            error = "model has no camera named '" + config.camera + "'; it has";

            for (int i = 0; i < model->ncam; i++) {
                const char* name = mj_id2name(model, mjOBJ_CAMERA, i);
                error += std::string(" '") + (name == nullptr ? "?" : name) + "'";
            }

            error += " and 'free'";
            return nullptr;
        }

        recorder->camera.type = mjCAMERA_FIXED;
        recorder->camera.fixedcamid = id;
    }

    const std::string command = "ffmpeg -y -loglevel error -f rawvideo -pixel_format rgb24 -video_size " +
                                std::to_string(config.width) + "x" + std::to_string(config.height) + " -framerate " +
                                std::to_string(config.fps) + " -i - -c:v libx264 -pix_fmt yuv420p -crf 20 '" +
                                config.path + "'";

    recorder->encoder = popen(command.c_str(), "w");

    if (recorder->encoder == nullptr) {
        error = "could not start ffmpeg";
        return nullptr;
    }

    const std::size_t size = static_cast<std::size_t>(config.width) * static_cast<std::size_t>(config.height) * 3;
    recorder->pixels.resize(size);
    recorder->flipped.resize(size);

    recorder->release_context();
    return recorder;
}

VideoRecorder::~VideoRecorder() {
    if (this->encoder != nullptr) {
        pclose(this->encoder);
    }

    if (this->context_ready) {
        this->make_context_current();
        mjr_freeContext(&this->context);
        mjv_freeScene(&this->scene);
    }

    close_opengl();
}

void VideoRecorder::attach(MujocoWorld& world, const Telemetry& telemetry, uint64_t ticks_per_frame) {
    this->world = &world;
    this->telemetry = &telemetry;
    this->ticks_per_frame = std::max<uint64_t>(1, ticks_per_frame);
}

void VideoRecorder::on_after_tick(const Simulation& simulation) {
    if (this->world == nullptr or simulation.tick() % this->ticks_per_frame != 0) {
        return;
    }

    const VideoHud hud{
        .sim_time = this->world->time(),
        .fsm_state = this->telemetry->value_of("FSM State"),
        .desired_linear = this->telemetry->value_of("Desired Linear Speed"),
        .odometry_linear = this->telemetry->value_of("Odometry Linear Velocity"),
    };

    this->capture(this->world->data(), hud);
}

void VideoRecorder::make_context_current() const {
    if (this->display != nullptr and this->gl_context != nullptr) {
        eglMakeCurrent(this->display, EGL_NO_SURFACE, EGL_NO_SURFACE, this->gl_context);
    }
}

void VideoRecorder::release_context() const {
    if (this->display != nullptr) {
        eglMakeCurrent(this->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
}

void VideoRecorder::capture(mjData* data, const VideoHud& hud) {
    if (this->encoder == nullptr) {
        return;
    }

    this->make_context_current();

    const mjrRect viewport{0, 0, this->config.width, this->config.height};

    mjv_updateScene(this->model, data, &this->option, nullptr, &this->camera, mjCAT_ALL, &this->scene);
    mjr_setBuffer(mjFB_OFFSCREEN, &this->context);
    mjr_render(viewport, &this->scene, &this->context);

    const std::string labels = "time\nstate\nv desired\nv odometry";
    const std::string values = std::format(
        "{:.3f} s\n{}\n{:.3f} m/s\n{:.3f} m/s", hud.sim_time, state_name(hud.fsm_state), hud.desired_linear,
        hud.odometry_linear
    );
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, labels.c_str(), values.c_str(), &this->context);

    mjr_readPixels(this->pixels.data(), nullptr, viewport, &this->context);

    const auto                     stride = static_cast<std::size_t>(this->config.width) * 3;
    const auto                     rows = static_cast<std::size_t>(this->config.height);
    const std::span<unsigned char> read_back(this->pixels);
    const std::span<unsigned char> written(this->flipped);

    for (std::size_t row = 0; row < rows; row++) {
        std::ranges::copy(read_back.subspan((rows - 1 - row) * stride, stride), written.subspan(row * stride).begin());
    }

    std::fwrite(this->flipped.data(), 1, this->flipped.size(), this->encoder);
    this->frames++;
    this->release_context();
}
}  // namespace micras::sim

#else  // MICRAS_VIDEO

namespace micras::sim {
std::unique_ptr<VideoRecorder>
    VideoRecorder::create(mjModel* /*model*/, const VideoConfig& /*config*/, std::string& error) {
    error = "this binary was built with -DMICRAS_VIDEO=OFF";
    return nullptr;
}

VideoRecorder::~VideoRecorder() = default;

void VideoRecorder::attach(MujocoWorld& /*world*/, const Telemetry& /*telemetry*/, uint64_t /*ticks_per_frame*/) { }

void VideoRecorder::on_after_tick(const Simulation& /*simulation*/) { }

void VideoRecorder::capture(mjData* /*data*/, const VideoHud& /*hud*/) { }
}  // namespace micras::sim

#endif  // MICRAS_VIDEO

/**
 * @file
 *
 * @brief Offscreen EGL video recorder piping raw frames to ffmpeg.
 */

#include "micras/sim/view/video_recorder.hpp"

#ifdef MICRAS_VIDEO

    #include <algorithm>
    #include <array>
    #include <cmath>
    #include <format>
    #include <limits>
    #include <span>
    #include <string>
    #include <utility>
    #include <vector>

    #include <EGL/egl.h>
    #include <EGL/eglext.h>

    #include "camera.hpp"

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
 *       is tried in turn.
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

}  // namespace

std::unique_ptr<VideoRecorder> VideoRecorder::create(
    MujocoWorld& world, const VideoConfig& config, OverlaySpec overlay, const VariableSource* variables,
    std::string& error
) {
    if (not init_opengl()) {
        error = "could not create a headless EGL/OpenGL context";
        return nullptr;
    }

    mjModel* model = world.model();
    model->vis.global.offwidth = std::max(model->vis.global.offwidth, config.width);
    model->vis.global.offheight = std::max(model->vis.global.offheight, config.height);

    std::unique_ptr<VideoRecorder> recorder(new VideoRecorder());
    recorder->world = &world;
    recorder->config = config;
    recorder->config.ticks_per_frame = std::max<uint64_t>(1, config.ticks_per_frame);
    recorder->overlay = std::move(overlay);
    recorder->variables = variables;
    recorder->trail_body = config.trail ? world.require_id(mjOBJ_BODY, config.body) : -1;

    mjv_defaultCamera(&recorder->camera);
    mjv_defaultOption(&recorder->option);
    mjv_defaultScene(&recorder->scene);
    mjr_defaultContext(&recorder->context);
    show_everything(recorder->option);

    recorder->display = eglGetCurrentDisplay();
    recorder->gl_context = eglGetCurrentContext();

    mjv_makeScene(model, &recorder->scene, 2000 + static_cast<int>(max_trail_points));
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
        recorder->camera.lookat[0] = model->stat.center[0];
        recorder->camera.lookat[1] = model->stat.center[1];
        recorder->camera.lookat[2] = model->stat.center[2];
        recorder->camera.distance = 1.6 * model->stat.extent;
        recorder->camera.azimuth = 90.0;
        recorder->camera.elevation = -65.0;
    } else if (not select_model_camera(model, config.camera, recorder->camera, error)) {
        return nullptr;
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

void VideoRecorder::on_after_tick(const Simulation& simulation) {
    if (this->trail_body >= 0) {
        this->extend_trail();
    }

    if (simulation.tick() % this->config.ticks_per_frame != 0) {
        return;
    }

    const auto value_of = [this](const std::string& variable) {
        return this->variables == nullptr ? std::numeric_limits<double>::quiet_NaN() :
                                            this->variables->value_of(variable);
    };

    std::string labels = "time";
    std::string values = std::format("{:.3f} s", this->world->time());

    if (this->overlay.state.has_value()) {
        labels += "\nstate";
        values += "\n" + this->overlay.state->name_of(value_of(this->overlay.state->variable));
    }

    for (const OverlayLine& line : this->overlay.lines) {
        labels += "\n" + line.label;
        values += std::format("\n{:.3f} {}", value_of(line.variable), line.unit);
    }

    this->capture(labels, values);
}

void VideoRecorder::extend_trail() {
    const std::span<const mjtNum> positions{
        this->world->data()->xpos, 3 * static_cast<std::size_t>(this->world->model()->nbody)
    };
    const std::span<const mjtNum> position = positions.subspan(3 * static_cast<std::size_t>(this->trail_body), 3);

    if (not this->trail.empty() and
        std::hypot(position[0] - this->trail.back()[0], position[1] - this->trail.back()[1]) < trail_spacing) {
        return;
    }

    if (this->trail.size() >= max_trail_points) {
        std::size_t kept = 0;

        for (std::size_t i = 0; i < this->trail.size(); i += 2) {
            this->trail[kept++] = this->trail[i];
        }

        this->trail.resize(kept);
    }

    this->trail.push_back({position[0], position[1], 0.002});
}

void VideoRecorder::draw_trail() {
    const std::span<mjvGeom> geoms{this->scene.geoms, static_cast<std::size_t>(this->scene.maxgeom)};

    for (std::size_t i = 1; i < this->trail.size() and this->scene.ngeom < this->scene.maxgeom; i++) {
        mjvGeom* geom = &geoms[static_cast<std::size_t>(this->scene.ngeom)];

        mjv_initGeom(geom, mjGEOM_NONE, nullptr, nullptr, nullptr, trail_color.data());
        mjv_connector(geom, mjGEOM_LINE, trail_width, this->trail[i - 1].data(), this->trail[i].data());
        geom->category = mjCAT_DECOR;
        this->scene.ngeom++;
    }
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

void VideoRecorder::capture(const std::string& labels, const std::string& values) {
    if (this->encoder == nullptr) {
        return;
    }

    this->make_context_current();

    const mjrRect viewport{0, 0, this->config.width, this->config.height};

    mjv_updateScene(
        this->world->model(), this->world->data(), &this->option, nullptr, &this->camera, mjCAT_ALL, &this->scene
    );
    this->draw_trail();
    mjr_setBuffer(mjFB_OFFSCREEN, &this->context);
    mjr_render(viewport, &this->scene, &this->context);

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
std::unique_ptr<VideoRecorder> VideoRecorder::create(
    MujocoWorld& /*world*/, const VideoConfig& /*config*/, OverlaySpec /*overlay*/, const VariableSource* /*variables*/,
    std::string& error
) {
    error = "this binary was built with -DMICRAS_VIDEO=OFF";
    return nullptr;
}

VideoRecorder::~VideoRecorder() = default;

void VideoRecorder::on_after_tick(const Simulation& /*simulation*/) { }
}  // namespace micras::sim

#endif  // MICRAS_VIDEO

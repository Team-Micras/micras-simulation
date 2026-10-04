/**
 * @file
 */

#include <memory>
#include <string>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/view/mujoco_viewer.hpp"
#include "micras/sim/view/panel_spec.hpp"
#include "micras/sim/view/view_options.hpp"

#ifdef MICRAS_SIM_VIEWER

    #include <algorithm>
    #include <format>
    #include <utility>

    #include <GLFW/glfw3.h>
    #include <imgui.h>
    #include <mujoco/mjdata.h>
    #include <mujoco/mjmodel.h>
    #include <mujoco/mjrender.h>
    #include <mujoco/mjtype.h>
    #include <mujoco/mjvisualize.h>
    #include <mujoco/mujoco.h>

    #include "camera.hpp"
    #include "micras/sim/view/control_panel.hpp"

namespace micras::sim {
namespace {
/**
 * @brief How much one scroll step zooms.
 */
constexpr double zoom_per_scroll{-0.05};
}  // namespace

/**
 * @brief Get the viewer a GLFW window belongs to.
 *
 * @param window Window the event came from.
 * @return The viewer.
 */
static MujocoViewer* viewer_of(GLFWwindow* window) {
    return static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
}

namespace {
/**
 * @brief Count how many GLFW users are still up.
 *
 * @note Unsynchronized because viewers are only ever built and destroyed on the
 *       simulation thread, which is also the only thread GLFW may be called
 *       from.
 */
// NOLINTNEXTLINE(*-avoid-non-const-global-variables): GLFW itself is process wide.
int glfw_users = 0;

/**
 * @brief Last message GLFW reported, so a failure says what actually happened.
 */
// NOLINTNEXTLINE(*-avoid-non-const-global-variables): the GLFW error callback takes no user pointer.
std::string glfw_error;
}  // namespace

std::unique_ptr<MujocoViewer> MujocoViewer::create(
    MujocoWorld& world, PanelSpec panel, const VariableSource* variables, const ViewerConfig& config, std::string& error
) {
    glfwSetErrorCallback([](int code, const char* description) {
        glfw_error = std::format("GLFW error {}: {}", code, description == nullptr ? "?" : description);
    });

    if (glfw_users == 0 and glfwInit() == GLFW_FALSE) {
        error = glfw_error.empty() ? "could not initialize GLFW; is a display available?" : glfw_error;
        return nullptr;
    }

    glfw_users++;

    std::unique_ptr<MujocoViewer> viewer(new MujocoViewer());
    viewer->world = &world;
    viewer->config = config;

    viewer->window = glfwCreateWindow(config.width, config.height, config.title.c_str(), nullptr, nullptr);

    if (viewer->window == nullptr) {
        error = glfw_error.empty() ? "could not open a window; is a display available?" : glfw_error;
        return nullptr;
    }

    glfwMakeContextCurrent(viewer->window);
    glfwSwapInterval(0);
    glfwSetWindowUserPointer(viewer->window, viewer.get());

    glfwSetKeyCallback(viewer->window, [](GLFWwindow* window, int key, int, int action, int) {
        if (action == GLFW_PRESS or action == GLFW_REPEAT) {
            viewer_of(window)->on_key(key);
        }
    });

    glfwSetMouseButtonCallback(viewer->window, [](GLFWwindow* window, int button, int action, int) {
        viewer_of(window)->on_mouse_button(button, action == GLFW_PRESS);
    });

    glfwSetCursorPosCallback(viewer->window, [](GLFWwindow* window, double x, double y) {
        viewer_of(window)->on_mouse_move(x, y);
    });

    glfwSetScrollCallback(viewer->window, [](GLFWwindow* window, double, double offset) {
        viewer_of(window)->on_scroll(offset);
    });

    mjv_defaultCamera(&viewer->camera);
    mjv_defaultOption(&viewer->option);
    mjv_defaultScene(&viewer->scene);
    mjv_defaultPerturb(&viewer->perturb);
    mjr_defaultContext(&viewer->context);

    show_everything(viewer->option);

    mjv_makeScene(world.model(), &viewer->scene, 2000);
    mjr_makeContext(world.model(), &viewer->context, mjFONTSCALE_150);
    viewer->context_ready = true;

    if (not viewer->select_camera(config.camera, error)) {
        return nullptr;
    }

    viewer->panel = std::make_unique<ControlPanel>(viewer->window, config.title, std::move(panel), variables);
    return viewer;
}

MujocoViewer::~MujocoViewer() {
    if (this->window != nullptr) {
        glfwMakeContextCurrent(this->window);
    }

    this->panel.reset();

    if (this->context_ready) {
        mjr_freeContext(&this->context);
        mjv_freeScene(&this->scene);
    }

    if (this->window != nullptr) {
        glfwDestroyWindow(this->window);
    }

    glfw_users--;

    if (glfw_users == 0) {
        glfwTerminate();
    }
}

bool MujocoViewer::select_camera(const std::string& name, std::string& error) {
    if (name == "free") {
        mjv_defaultFreeCamera(this->world->model(), &this->camera);
        return true;
    }

    return select_model_camera(this->world->model(), name, this->camera, error);
}

void MujocoViewer::on_start(const Simulation& simulation) {
    this->draw(simulation);
}

RunControl MujocoViewer::on_before_tick(const Simulation& simulation) {
    glfwPollEvents();

    while ((this->paused and this->pending_steps == 0) or this->wait_for_speed_limit()) {
        if (this->should_quit()) {
            return RunControl::QUIT;
        }

        this->draw(simulation);
        glfwWaitEventsTimeout(0.005);
    }

    if (this->should_quit()) {
        return RunControl::QUIT;
    }

    if (this->paused and this->pending_steps > 0) {
        this->pending_steps--;
    }

    this->last_tick_at = glfwGetTime();
    this->apply_perturbation();
    return RunControl::RUN;
}

bool MujocoViewer::should_quit() const {
    return glfwWindowShouldClose(this->window) != 0 or this->quit_requested;
}

bool MujocoViewer::wait_for_speed_limit() const {
    if (this->speed <= 0.0) {
        return false;
    }

    return glfwGetTime() - this->last_tick_at < this->config.us_per_tick * 1e-6 / this->speed;
}

bool MujocoViewer::was_interactive() const {
    return this->interactive or this->panel->was_touched();
}

void MujocoViewer::on_after_tick(const Simulation& simulation) {
    if (simulation.tick() % this->config.ticks_per_frame != 0) {
        return;
    }

    this->panel->sample(this->world->time());
    this->draw(simulation);
}

void MujocoViewer::on_finish(const Simulation& simulation) {
    this->draw(simulation);
}

void MujocoViewer::apply_perturbation() {
    const mjModel* model = this->world->model();
    mjData*        data = this->world->data();

    mju_zero(data->xfrc_applied, static_cast<int>(6 * model->nbody));

    if (this->perturb.active == 0) {
        return;
    }

    this->interactive = true;
    mjv_applyPerturbForce(model, data, &this->perturb);
}

void MujocoViewer::draw(const Simulation& simulation) {
    glfwMakeContextCurrent(this->window);

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(this->window, &width, &height);

    const mjrRect viewport{.left = 0, .bottom = 0, .width = width, .height = height};

    mjv_updateScene(
        this->world->model(), this->world->data(), &this->option, &this->perturb, &this->camera, mjCAT_ALL, &this->scene
    );
    mjr_render(viewport, &this->scene, &this->context);

    const std::string labels = "tick\ntime\nstate\nkeys";
    const std::string values = std::format(
        "{}\n{:.3f} s\n{}\nspace pause  right step  tab camera  esc quit", simulation.tick(), this->world->time(),
        this->paused ? "paused" : "running"
    );
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, labels.c_str(), values.c_str(), &this->context);

    const PanelRequest request = this->panel->draw(simulation, this->paused);

    if (request.paused_changed) {
        this->paused = request.paused;
    }

    this->speed = request.speed;
    this->quit_requested = request.quit;

    if (request.step and this->paused) {
        this->pending_steps++;
    }

    glfwSwapBuffers(this->window);
}

void MujocoViewer::on_key(int key) {
    if (ImGui::GetIO().WantCaptureKeyboard) {
        return;
    }

    switch (key) {
        case GLFW_KEY_SPACE:
            this->paused = not this->paused;
            break;

        case GLFW_KEY_RIGHT:
            if (this->paused) {
                this->pending_steps++;
            }

            break;

        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(this->window, GLFW_TRUE);
            break;

        case GLFW_KEY_TAB:
            this->next_camera();
            break;

        case GLFW_KEY_C:
            this->option.flags[mjVIS_CONTACTPOINT] = 1 - this->option.flags[mjVIS_CONTACTPOINT];
            break;

        case GLFW_KEY_F:
            this->option.flags[mjVIS_CONTACTFORCE] = 1 - this->option.flags[mjVIS_CONTACTFORCE];
            break;

        case GLFW_KEY_T:
            this->option.flags[mjVIS_TRANSPARENT] = 1 - this->option.flags[mjVIS_TRANSPARENT];
            break;

        case GLFW_KEY_R:
            this->option.flags[mjVIS_RANGEFINDER] = 1 - this->option.flags[mjVIS_RANGEFINDER];
            break;

        default:
            break;
    }
}

void MujocoViewer::next_camera() {
    const auto cameras = static_cast<int>(this->world->model()->ncam);

    if (this->camera.type == mjCAMERA_FIXED and this->camera.fixedcamid + 1 < cameras) {
        this->camera.fixedcamid++;
    } else if (this->camera.type == mjCAMERA_FIXED or cameras == 0) {
        this->camera.type = mjCAMERA_FREE;
    } else {
        this->camera.type = mjCAMERA_FIXED;
        this->camera.fixedcamid = 0;
    }
}

void MujocoViewer::on_mouse_button(int button, bool pressed) {
    if (ImGui::GetIO().WantCaptureMouse) {
        this->left_held = false;
        this->right_held = false;
        this->middle_held = false;
        this->perturb.active = 0;
        return;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        this->left_held = pressed;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        this->right_held = pressed;
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        this->middle_held = pressed;
    }

    glfwGetCursorPos(this->window, &this->last_x, &this->last_y);

    const bool dragging = this->left_held and (glfwGetKey(this->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);

    if (not dragging) {
        this->perturb.active = 0;
        return;
    }

    const int body = mj_name2id(this->world->model(), mjOBJ_BODY, this->config.body.c_str());

    if (body < 0) {
        return;
    }

    this->perturb.select = body;
    this->perturb.active = mjPERT_TRANSLATE;
    mjv_initPerturb(this->world->model(), this->world->data(), &this->scene, &this->perturb);
}

void MujocoViewer::on_mouse_move(double x, double y) {
    if (ImGui::GetIO().WantCaptureMouse) {
        this->last_x = x;
        this->last_y = y;
        return;
    }

    const double dx = x - this->last_x;
    const double dy = y - this->last_y;
    this->last_x = x;
    this->last_y = y;

    if (not this->left_held and not this->right_held and not this->middle_held) {
        return;
    }

    int width = 0;
    int height = 0;
    glfwGetWindowSize(this->window, &width, &height);

    const double relative_x = dx / std::max(1, width);
    const double relative_y = dy / std::max(1, height);

    if (this->perturb.active != 0) {
        mjv_movePerturb(
            this->world->model(), this->world->data(), mjMOUSE_MOVE_H, relative_x, relative_y, &this->scene,
            &this->perturb
        );
        return;
    }

    mjtMouse action = mjMOUSE_ROTATE_V;

    if (this->right_held) {
        action = mjMOUSE_MOVE_H;
    } else if (this->middle_held) {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(this->world->model(), action, relative_x, relative_y, &this->camera);
}

void MujocoViewer::on_scroll(double offset) {
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    mjv_moveCamera(this->world->model(), mjMOUSE_ZOOM, 0.0, zoom_per_scroll * offset, &this->camera);
}
}  // namespace micras::sim

#else  // MICRAS_SIM_VIEWER

namespace micras::sim {
std::unique_ptr<MujocoViewer> MujocoViewer::create(
    MujocoWorld& /*world*/, PanelSpec /*panel*/, const VariableSource* /*variables*/, const ViewerConfig& /*config*/,
    std::string& error
) {
    error = "this binary was built with -DMICRAS_SIM_VIEWER=OFF";
    return nullptr;
}

bool MujocoViewer::was_interactive() const {
    return this->interactive;
}

MujocoViewer::~MujocoViewer() = default;

void MujocoViewer::on_start(const Simulation& /*simulation*/) { }

RunControl MujocoViewer::on_before_tick(const Simulation& /*simulation*/) {
    return RunControl::RUN;
}

void MujocoViewer::on_after_tick(const Simulation& /*simulation*/) { }

void MujocoViewer::on_finish(const Simulation& /*simulation*/) { }
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEWER

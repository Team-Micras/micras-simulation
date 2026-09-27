/**
 * @file
 *
 * @brief Live window onto a running simulation.
 */

#ifndef MICRAS_SIM_VIEW_MUJOCO_VIEWER_HPP
#define MICRAS_SIM_VIEW_MUJOCO_VIEWER_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/view/control_panel.hpp"
#include "micras/sim/view/panel_spec.hpp"
#include "micras/sim/view/view_options.hpp"

struct GLFWwindow;

namespace micras::sim {
/**
 * @brief Draws the simulation and lets a human steer the view and the pace.
 *
 * @note Runs on the simulation thread, between ticks: there is no render thread
 *       and therefore no way to read a half written state. Everything it does
 *       by itself only reads mjModel and mjData, so a run watched through an
 *       untouched window produces the same data.csv as a headless one. Dragging
 *       the robot applies a force, which is a real change to the run, so the
 *       first drag marks the run interactive.
 */
class MujocoViewer : public IRunListener {
public:
    /**
     * @brief Open a window and build the render context.
     *
     * @note Rangefinder rays are drawn from the start: range sensors are
     *       usually the interesting part of a run.
     *
     * @param world World to draw.
     * @param panel What the robot target shows on the panel.
     * @param variables Source of the panel's plots and state label, or null.
     * @param config Window configuration.
     * @param error Filled with a human readable reason when creation fails.
     * @return The viewer, or nullptr if no window could be opened.
     */
    static std::unique_ptr<MujocoViewer> create(
        MujocoWorld& world, PanelSpec panel, const VariableSource* variables, const ViewerConfig& config,
        std::string& error
    );

    MujocoViewer(const MujocoViewer&) = delete;
    MujocoViewer(MujocoViewer&&) = delete;
    MujocoViewer& operator=(const MujocoViewer&) = delete;
    MujocoViewer& operator=(MujocoViewer&&) = delete;

    /**
     * @brief Release the render context and close the window.
     */
    ~MujocoViewer() override;

    /**
     * @brief Draw the initial state before the first tick.
     *
     * @param simulation Run about to start.
     */
    void on_start(const Simulation& simulation) override;

    /**
     * @brief Handle input, and block here while the run is paused.
     *
     * @param simulation Run about to advance.
     * @return QUIT once the window is closed.
     */
    RunControl on_before_tick(const Simulation& simulation) override;

    /**
     * @brief Redraw the frame this tick falls on, if any.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

    /**
     * @brief Draw the final state and let the run end.
     *
     * @note The window closes with the run rather than waiting to be dismissed,
     *       so a scripted run is not held up by it. Pause before the end to
     *       look at the final state.
     *
     * @param simulation Run that finished.
     */
    void on_finish(const Simulation& simulation) override;

    /**
     * @brief Check whether a human changed anything the run can see.
     *
     * @return True once the robot has been dragged or a control was used.
     */
    bool was_interactive() const;

private:
    MujocoViewer() = default;

    /**
     * @brief Draw one frame and present it.
     *
     * @param simulation Run being drawn, for the overlay.
     */
    void draw(const Simulation& simulation);

    /**
     * @brief Apply the drag force, if the robot is being dragged.
     */
    void apply_perturbation();

    /**
     * @brief Check whether the human asked the run to end.
     *
     * @return True once the window was closed or quit was pressed.
     */
    bool should_quit() const;

    /**
     * @brief Check whether the next tick is still held back by the speed limit.
     *
     * @note The only wall clock in a run, and it only decides when a tick
     *       starts, never what it computes.
     *
     * @return True while the tick must wait.
     */
    bool wait_for_speed_limit() const;

    /**
     * @brief Point the camera at what the configuration asked for.
     *
     * @param name Model camera name, or "free".
     * @param error Filled when the model has no such camera.
     * @return False when the name was rejected.
     */
    bool select_camera(const std::string& name, std::string& error);

    /**
     * @brief Handle a key press.
     *
     * @param key GLFW key code.
     */
    void on_key(int key);

    /**
     * @brief Look through the next camera: the free one, then each camera of the model in turn.
     */
    void next_camera();

    /**
     * @brief Handle a mouse button change.
     *
     * @note Holding control and the left button drags the robot. Selecting its
     *       body is enough, because the drag is applied as a force and never as
     *       a pose, so the physics stays in charge of the result.
     *
     * @param button GLFW button code.
     * @param pressed Whether it went down.
     */
    void on_mouse_button(int button, bool pressed);

    /**
     * @brief Handle a mouse move.
     *
     * @param x Cursor position in pixels.
     * @param y Cursor position in pixels.
     */
    void on_mouse_move(double x, double y);

    /**
     * @brief Handle a scroll wheel step.
     *
     * @param offset Scroll amount.
     */
    void on_scroll(double offset);

    /**
     * @brief World being drawn.
     */
    MujocoWorld* world{nullptr};

    /**
     * @brief Window configuration.
     */
    ViewerConfig config;

    /**
     * @brief Window the frames go to.
     */
    GLFWwindow* window{nullptr};

    /**
     * @brief MuJoCo visualization state.
     */
    ///@{
    mjvScene   scene{};
    mjvCamera  camera{};
    mjvOption  option{};
    mjvPerturb perturb{};
    mjrContext context{};
    ///@}

    /**
     * @brief Whether the render context and scene were created.
     */
    bool context_ready{false};

    /**
     * @brief Whether the run is waiting for the human to let it continue.
     */
    bool paused{false};

    /**
     * @brief Number of ticks the human asked to run while paused.
     */
    uint64_t pending_steps{0};

    /**
     * @brief Whether the robot has been dragged at least once.
     */
    bool interactive{false};

    /**
     * @brief Whether the human asked the run to end.
     */
    bool quit_requested{false};

    /**
     * @brief Simulated seconds allowed per wall second, zero for no limit.
     */
    double speed{0.0};

    /**
     * @brief Mouse state, in window coordinates.
     */
    ///@{
    double last_x{0.0};
    double last_y{0.0};
    bool   left_held{false};
    bool   right_held{false};
    bool   middle_held{false};
    ///@}

    /**
     * @brief The board and the plots, drawn over the scene.
     */
    std::unique_ptr<ControlPanel> panel;

    /**
     * @brief Wall time the last tick was allowed to start at.
     */
    double last_tick_at{0.0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_MUJOCO_VIEWER_HPP

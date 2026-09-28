/**
 * @file
 *
 * @brief The panel a human drives the robot from.
 */

#ifndef MICRAS_SIM_VIEW_CONTROL_PANEL_HPP
#define MICRAS_SIM_VIEW_CONTROL_PANEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/view/panel_spec.hpp"

struct GLFWwindow;

namespace micras::sim {
/**
 * @brief What the human asked of the run through the panel.
 */
struct PanelRequest {
    /**
     * @brief Whether the run should stay paused, and whether the panel changed it.
     */
    ///@{
    bool paused{false};
    bool paused_changed{false};
    ///@}

    /**
     * @brief Whether the human asked for one more tick, in this frame.
     */
    bool step{false};

    /**
     * @brief Simulated seconds allowed per wall second, zero for no limit.
     */
    double speed{0.0};

    /**
     * @brief Whether the human asked the run to end.
     */
    bool quit{false};
};

/**
 * @brief A fixed length history of one plotted value.
 */
class PlotTrace {
public:
    /**
     * @brief Number of samples kept.
     */
    static constexpr std::size_t capacity{2048};

    /**
     * @brief Name the plot and the firmware variable it follows.
     *
     * @param variable Firmware name of the variable.
     */
    explicit PlotTrace(std::string variable);

    /**
     * @brief Append the current value of the variable.
     *
     * @param time Simulated time of the sample.
     * @param variables Where the value is read, or null when the robot reports none.
     */
    void sample(double time, const VariableSource* variables);

    /**
     * @brief Get the firmware name of the variable.
     *
     * @return The name.
     */
    const std::string& name() const { return this->variable; }

    /**
     * @brief Get the sample times, in storage order.
     *
     * @note Storage order is oldest first starting at oldest(), wrapping around.
     *
     * @return The first size() entries are valid.
     */
    const std::array<double, capacity>& times() const { return this->sample_times; }

    /**
     * @brief Get the sample values, in the same storage order as times().
     *
     * @return The first size() entries are valid.
     */
    const std::array<double, capacity>& values() const { return this->sample_values; }

    /**
     * @brief Get how many samples are kept.
     *
     * @return Number of valid entries, at most capacity.
     */
    std::size_t size() const { return this->count; }

    /**
     * @brief Get the storage index of the oldest sample.
     *
     * @return Index into times() and values().
     */
    std::size_t oldest() const { return this->count < capacity ? 0 : this->next; }

private:
    /**
     * @brief Firmware name of the variable.
     */
    std::string variable;

    /**
     * @brief Ring buffers of the samples.
     */
    ///@{
    std::array<double, capacity> sample_times{};
    std::array<double, capacity> sample_values{};
    ///@}

    /**
     * @brief Storage index the next sample is written to.
     */
    std::size_t next{0};

    /**
     * @brief Number of valid samples.
     */
    std::size_t count{0};
};

/**
 * @brief Draws what the robot target declared, and turns clicks into its input.
 *
 * @note Runs on the simulation thread between ticks, like the viewer it shares
 *       a window with. Buttons react to the press and release edges, never to
 *       the held state: writing the held state every frame would overwrite a
 *       scripted press. Touching a button or a switch marks the run
 *       interactive, because nothing about it is reproducible from the command
 *       line alone, and hands the board over from the script to the human.
 */
class ControlPanel {
public:
    /**
     * @brief Attach ImGui and ImPlot to an existing window.
     *
     * @param window Window the panel is drawn in.
     * @param title Title of the panel window.
     * @param spec What the robot target shows and lets a human drive.
     * @param variables Source of the plots and the state label, or null.
     */
    ControlPanel(GLFWwindow* window, std::string title, PanelSpec spec, const VariableSource* variables);

    ControlPanel(const ControlPanel&) = delete;
    ControlPanel(ControlPanel&&) = delete;
    ControlPanel& operator=(const ControlPanel&) = delete;
    ControlPanel& operator=(ControlPanel&&) = delete;

    /**
     * @brief Detach ImGui and ImPlot from the window.
     */
    ~ControlPanel();

    /**
     * @brief Append a sample to every plot.
     *
     * @param time Simulated time of the sample.
     */
    void sample(double time);

    /**
     * @brief Draw the panel over the current frame.
     *
     * @param simulation Run being driven.
     * @param paused Whether the run is currently paused, which the viewer owns.
     * @return What the human asked of the run.
     */
    PanelRequest draw(const Simulation& simulation, bool paused);

    /**
     * @brief Check whether a human touched anything the firmware can see.
     *
     * @return True once a control was used.
     */
    bool was_touched() const { return this->touched; }

private:
    /**
     * @brief Draw the board controls and readouts.
     *
     * @param paused Whether the run is currently paused.
     */
    void draw_board(bool paused);

    /**
     * @brief Draw the plots of firmware variables.
     */
    void draw_plots();

    /**
     * @brief Record that a human has taken the board over from the script.
     */
    void take_over();

    /**
     * @brief Title of the panel window.
     */
    std::string title;

    /**
     * @brief What the robot target shows and lets a human drive.
     */
    PanelSpec spec;

    /**
     * @brief Source of the plots and the state label, or null.
     *
     * @note Owned by the robot target, which outlives the window.
     */
    const VariableSource* variables;

    /**
     * @brief Plots shown under the controls.
     */
    std::vector<PlotTrace> traces;

    /**
     * @brief What the human last asked of the run.
     */
    PanelRequest request;

    /**
     * @brief Whether a control has been used.
     */
    bool touched{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_CONTROL_PANEL_HPP

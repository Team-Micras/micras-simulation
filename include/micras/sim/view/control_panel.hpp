/**
 * @file
 *
 * @brief The panel a human drives the robot from.
 */

#ifndef MICRAS_SIM_VIEW_CONTROL_PANEL_HPP
#define MICRAS_SIM_VIEW_CONTROL_PANEL_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "micras/sim/core/proxy_state.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/telemetry/telemetry.hpp"

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
     * @brief Ticks the human asked to run while paused.
     */
    uint64_t steps{0};

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
     * @brief Name the plot and the pool variable it follows.
     *
     * @param variable Firmware name of the variable.
     */
    explicit PlotTrace(std::string variable);

    /**
     * @brief Append the current value of the variable.
     *
     * @param time Simulated time of the sample.
     * @param telemetry Decoder holding the pool values.
     */
    void sample(double time, const Telemetry& telemetry);

    /**
     * @brief Get the firmware name of the variable.
     *
     * @return The name.
     */
    const std::string& name() const { return this->variable; }

    /**
     * @brief Get the sample times, oldest first.
     *
     * @return The times.
     */
    const std::vector<double>& times() const { return this->sample_times; }

    /**
     * @brief Get the sample values, oldest first.
     *
     * @return The values.
     */
    const std::vector<double>& values() const { return this->sample_values; }

private:
    /**
     * @brief Firmware name of the variable.
     */
    std::string variable;

    /**
     * @brief Samples kept, oldest first.
     */
    ///@{
    std::vector<double> sample_times;
    std::vector<double> sample_values;
    ///@}
};

/**
 * @brief Draws the board and the plots, and turns clicks into interface input.
 *
 * @note Runs on the simulation thread between ticks, like the viewer it shares
 *       a window with. It writes only the interface input, which is what a hand
 *       on the real board would write, so a run driven from here is a run the
 *       firmware cannot tell from a real one. Touching anything here marks the
 *       run interactive, because nothing about it is reproducible from the
 *       command line alone.
 */
class ControlPanel {
public:
    /**
     * @brief Attach ImGui and ImPlot to an existing window.
     *
     * @param window Window the panel is drawn in.
     * @param state Boundary record the panel reads and writes.
     * @param telemetry Decoder feeding the plots and the state readout.
     */
    ControlPanel(GLFWwindow* window, ProxyState& state, const Telemetry& telemetry);

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
     * @brief Draw the pool plots.
     */
    void draw_plots();

    /**
     * @brief Record that a human has taken the board over from the script.
     */
    void take_over();

    /**
     * @brief Boundary record the panel reads and writes.
     *
     * @note Bound for the life of the panel; the context outlives the window.
     */
    ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Decoder feeding the plots and the state readout.
     *
     * @note Bound for the life of the panel; the context outlives the window.
     */
    const Telemetry& telemetry;  // NOLINT(*-avoid-const-or-ref-data-members)

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

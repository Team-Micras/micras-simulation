/**
 * @file
 *
 * @brief Scripted input for a run: what the robot is told to do, and when.
 */

#ifndef MICRAS_SIM_CORE_SCENARIO_HPP
#define MICRAS_SIM_CORE_SCENARIO_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "micras/sim/core/proxy_state.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/telemetry/telemetry.hpp"

namespace micras::sim {
/**
 * @brief Command to hand the firmware during a run.
 *
 * @note Delivered as a single SERIAL_VARIABLE packet writing true to the
 *       matching bidirectional pool variable, exactly like the monitor does.
 */
enum class Command : uint8_t {
    NONE,
    EXPLORE,
    SOLVE,
    CALIBRATE,
};

/**
 * @brief Button press to emit during a run.
 */
enum class ButtonPress : uint8_t {
    NONE,
    SHORT,
    LONG,
    EXTRA_LONG,
};

/**
 * @brief Press durations the board classifies against.
 *
 * @note Taken from the Button config rather than read from target.hpp, so the
 *       scenario stays independent of the firmware shadow.
 */
struct ButtonDelays {
    uint32_t long_press;
    uint32_t extra_long_press;
};

/**
 * @brief Everything the scenario can be told from the command line.
 */
struct ScenarioScript {
    /**
     * @brief Command to inject, and when.
     */
    ///@{
    Command command{Command::NONE};
    double  command_at{0.5};
    ///@}

    /**
     * @brief Button press to emit, and when it goes down.
     */
    ///@{
    ButtonPress button{ButtonPress::NONE};
    double      button_at{0.5};
    ///@}

    /**
     * @brief DIP switch states, indexed as in Interface::DipSwitchPins.
     */
    std::array<bool, InterfaceInput::dip_switch_count> dip{};

    /**
     * @brief Whether the Fan proxy is allowed to drive its actuator.
     */
    bool fan_enabled{true};
};

/**
 * @brief Parse a command name as the command line spells it.
 *
 * @param name One of none, explore, solve or calibrate.
 * @return The command, or nullopt for an unknown name.
 */
std::optional<Command> parse_command(std::string_view name);

/**
 * @brief Parse a button press name as the command line spells it.
 *
 * @param name One of none, short, long or extra_long.
 * @return The press, or nullopt for an unknown name.
 */
std::optional<ButtonPress> parse_button_press(std::string_view name);

/**
 * @brief Get the pool variable a command is delivered through.
 *
 * @param command Command to deliver.
 * @return Firmware variable name, empty for Command::NONE.
 */
std::string_view command_variable(Command command);

/**
 * @brief Plays the script into the simulation, one tick at a time.
 *
 * @note Writes only interface input and the serial queue, never mjData or the
 *       clock, so a run driven by a scenario is the same run a human could
 *       drive by hand.
 *
 * @note Stops writing the board entirely once a human has touched it, so the
 *       two never fight over the same field.
 */
class Scenario : public IRunListener {
public:
    /**
     * @brief Take the script and everything it needs to play it.
     *
     * @param script What to do and when.
     * @param state Interface input the button is written into.
     * @param telemetry Decoder used to address the command variable.
     * @param clock Clock the instants are converted against.
     * @param delays Press durations the board classifies against.
     */
    Scenario(
        const ScenarioScript& script, ProxyState& state, Telemetry& telemetry, const Clock& clock,
        const ButtonDelays& delays
    );

    /**
     * @brief Apply the DIP switches and the fan override before the first tick.
     *
     * @param simulation Run about to start.
     */
    void on_start(const Simulation& simulation) override;

    /**
     * @brief Hold the button and inject the command at their instants.
     *
     * @param simulation Run about to advance.
     * @return Always RunControl::RUN; a script never stops a run.
     */
    RunControl on_before_tick(const Simulation& simulation) override;

    /**
     * @brief Check whether the command was delivered.
     *
     * @return True once the command packet was queued, or if there was none.
     */
    bool command_delivered() const { return this->command_sent; }

    /**
     * @brief Get how many simulated milliseconds a press must be held.
     *
     * @note Button::update classifies on elapsed_time_ms strictly greater than
     *       each delay, so long and extra long presses are one millisecond past
     *       their thresholds.
     *
     * @param press Press to emit.
     * @param delays Press durations the board classifies against.
     * @return Hold time in simulated milliseconds, zero for no press.
     */
    static uint32_t hold_time_ms(ButtonPress press, const ButtonDelays& delays);

private:
    /**
     * @brief Script being played.
     */
    ScenarioScript script;

    /**
     * @brief Interface input written by the script.
     *
     * @note Bound for the life of the run; the context outlives every listener.
     */
    ProxyState& state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Decoder used to address and queue the command.
     *
     * @note Bound for the life of the run; the context outlives every listener.
     */
    Telemetry& telemetry;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief First tick the command may be injected on.
     *
     * @note Never tick 0: that tick carries the variable map request and the
     *       firmware drops the rest of its receive buffer after framing one
     *       packet.
     */
    uint64_t command_tick;

    /**
     * @brief Ticks the button is held between.
     */
    ///@{
    uint64_t button_press_tick;
    uint64_t button_release_tick;
    ///@}

    /**
     * @brief Whether the command still has to be injected.
     */
    bool command_sent;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_SCENARIO_HPP

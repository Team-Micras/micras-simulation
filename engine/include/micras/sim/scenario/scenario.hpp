/**
 * @file
 *
 * @brief A scenario file: the arena, the duration, and what happens to the robot when.
 */

#ifndef MICRAS_SIM_SCENARIO_SCENARIO_HPP
#define MICRAS_SIM_SCENARIO_SCENARIO_HPP

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/devices/device.hpp"
#include "micras/sim/devices/digital_input.hpp"

namespace micras::sim {
/**
 * @brief One thing that happens to the robot at an instant.
 *
 * @note With a condition (when and equals) the event waits, from its time on, for the firmware
 * variable to take that value, and happens on the first tick it does. Events happen in order, so a
 * waiting event holds back the ones after it.
 */
struct ScenarioEvent {
    /**
     * @brief What kind of thing.
     */
    enum class Kind : uint8_t {
        PRESS,
        SET,
        SEND,
    };

    Kind        kind{Kind::PRESS};
    double      at{0.0};
    double      duration{0.0};
    std::string target;
    bool        value{false};
    std::string when;
    std::string equals;
};

/**
 * @brief When a run ends before its duration: a firmware variable reaching one of some values.
 *
 * @note Only what happens from `after` on counts, and the run ends the `count`-th time the
 *       variable reaches one of the values. A value it already holds at `after` counts as reached.
 *       Any of the abort values ends the run the first time, whatever the count.
 */
struct StopCondition {
    std::string              variable;
    std::vector<std::string> equals;
    double                   after{0.0};
    int                      count{1};
    std::vector<std::string> abort;
};

/**
 * @brief A scenario, as its TOML file states it.
 *
 * @note The file names the arena, the duration, the seed and optionally the
 *       robot and its start pose; then a timeline of events, and optionally a
 *       stop condition. Events act on what the robot target exposes: its named
 *       inputs (press, set) and its named link messages (send). Command line
 *       options override the file's arena, duration and seed.
 */
struct Scenario {
    std::string                           robot;
    std::string                           arena;
    double                                seconds{0.0};
    std::optional<uint64_t>               seed;
    std::optional<MujocoWorld::Placement> start;
    std::vector<ScenarioEvent>            events;
    std::optional<StopCondition>          stop;

    /**
     * @brief Read a scenario file.
     *
     * @param path Path of the file.
     * @return The scenario.
     */
    static Scenario load(const std::filesystem::path& path);

    /**
     * @brief Read a scenario.
     *
     * @param text Contents of a scenario file.
     * @param origin Name of the text, for messages.
     * @return The scenario.
     */
    static Scenario parse(std::string_view text, const std::string& origin = "scenario");
};

/**
 * @brief What a robot target exposes to scenarios.
 */
struct ScenarioHooks {
    /**
     * @brief Inputs a scenario presses or sets, by name.
     */
    std::map<std::string, DigitalInput*, std::less<>> inputs;

    /**
     * @brief Messages a scenario sends on the link, by name.
     */
    std::map<std::string, std::vector<uint8_t>, std::less<>> messages;

    /**
     * @brief Names of the values of state variables, for conditions and the event log.
     */
    StateNames state_names;
};

/**
 * @brief Plays a scenario's timeline into the run and stops it when its condition holds.
 *
 * @note Writes inputs and the link only, never the physics state or the clock,
 *       so a scripted run is a run a human could have driven by hand. Once a
 *       human takes the board over from the panel it stops touching inputs.
 */
class ScenarioPlayer : public IRunListener {
public:
    /**
     * @brief Take the scenario and everything it acts on.
     *
     * @param scenario The scenario.
     * @param hooks What the robot exposes.
     * @param serial The link's world side.
     * @param variables Where stop conditions read, or null.
     */
    ScenarioPlayer(Scenario scenario, const ScenarioHooks& hooks, SerialBus& serial, const VariableSource* variables);

    /**
     * @brief Apply the events of this tick and check the stop condition.
     *
     * @param simulation Run about to advance.
     * @return QUIT once the stop condition holds.
     */
    RunControl on_before_tick(const Simulation& simulation) override;

    /**
     * @brief Stop touching inputs: a human has the board.
     */
    void hand_over() { this->human = true; }

    /**
     * @brief Get the instant the stop condition held.
     *
     * @return Seconds, or nothing when it never did.
     */
    std::optional<double> stopped_at() const { return this->stop_time; }

private:
    /**
     * @brief An input to release at a tick.
     */
    struct Release {
        DigitalInput* input;
        uint64_t      tick;
    };

    /**
     * @brief Check an event against what the robot exposes.
     *
     * @param event The event.
     */
    void check(const ScenarioEvent& event) const;

    /**
     * @brief Check whether an event's condition holds.
     *
     * @param index Index of the event.
     * @return True for an event without a condition, or when its variable has the value.
     */
    bool is_due(std::size_t index) const;

    /**
     * @brief Check the stop condition.
     *
     * @param now Simulated time in seconds.
     * @return QUIT once the condition is reached for the count-th time or takes an abort value.
     */
    RunControl check_stop(double now);

    /**
     * @brief Turn one of the stop condition's values into the number the variable takes.
     *
     * @param variable The variable the condition watches.
     * @param value A number, or the name of one of the variable's states.
     * @return The number.
     */
    double resolve(const std::string& variable, const std::string& value) const;

    Scenario              scenario;
    const ScenarioHooks&  hooks;   // NOLINT(*-avoid-const-or-ref-data-members): the target outlives the run.
    SerialBus&            serial;  // NOLINT(*-avoid-const-or-ref-data-members): the run's bus.
    const VariableSource* variables;
    std::vector<Release>  releases;
    std::size_t           next_event{0};
    std::vector<double>   stop_values;
    std::vector<double>   event_values;
    std::vector<double>   abort_values;
    bool                  stop_matching{false};
    int                   stop_reached{0};
    std::optional<double> stop_time;
    bool                  human{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_SCENARIO_SCENARIO_HPP

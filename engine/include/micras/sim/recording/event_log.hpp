/**
 * @file
 *
 * @brief What a run notices at full rate, whatever the recording's decimation.
 */

#ifndef MICRAS_SIM_RECORDING_EVENT_LOG_HPP
#define MICRAS_SIM_RECORDING_EVENT_LOG_HPP

#include <cstddef>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/recording/run_metadata.hpp"

namespace micras::sim {
/**
 * @brief Logs collisions and state changes every tick, for meta.json.
 *
 * @note A collision is a watched geom starting to touch anything but an ignored
 *       geom, such as the floor, after having been clear of everything for at
 *       least min_separation: a chassis pressed against a wall makes and breaks
 *       contact every few steps, and that is one collision, not hundreds. A
 *       state change is a watched variable taking a
 *       new value, named from its table when it has one. The log keeps the first
 *       entries only, so a robot stuck against a wall cannot grow meta.json
 *       without bound; the counts keep counting.
 */
class EventLog : public IRunListener {
public:
    /**
     * @brief A variable whose changes are logged.
     */
    struct WatchedState {
        std::string              variable;
        std::vector<std::string> names;
    };

    /**
     * @brief Most entries kept.
     */
    static constexpr std::size_t max_events{500};

    /**
     * @brief Time a geom has to stay clear for its next contact to be a new collision, in seconds.
     */
    static constexpr double min_separation{0.05};

    /**
     * @brief Watch geoms and states.
     *
     * @param geoms Ids of the geoms whose collisions are logged.
     * @param ignored Ids of the geoms touching which is not a collision.
     * @param states Variables whose changes are logged.
     * @param variables Where the variables are read, or null.
     */
    EventLog(
        std::vector<int> geoms, std::set<int> ignored, std::vector<WatchedState> states, const VariableSource* variables
    );

    /**
     * @brief Log a variable's changes too.
     *
     * @param state The variable and the names of its values.
     */
    void watch(WatchedState state);

    /**
     * @brief Look for new contacts and new states.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

    /**
     * @brief Get the entries, in time order.
     *
     * @return The entries kept.
     */
    const std::vector<RunEvent>& entries() const { return this->logged; }

    /**
     * @brief Get how many collisions started.
     *
     * @return Number of collisions, counted past the entries kept.
     */
    std::size_t collisions() const { return this->collision_count; }

private:
    /**
     * @brief Keep an entry, while there is room.
     *
     * @param event The entry.
     */
    void add(RunEvent event);

    /**
     * @brief Log the collisions that started on this tick.
     *
     * @param model The model, naming the geoms.
     * @param contacts The tick's contacts.
     * @param now Simulated time, in seconds.
     */
    void log_collisions(const mjModel* model, std::span<const mjContact> contacts, double now);

    /**
     * @brief Log the watched variables that changed on this tick.
     *
     * @param now Simulated time, in seconds.
     */
    void log_states(double now);

    std::vector<int>                   geoms;
    std::set<int>                      ignored;
    std::vector<WatchedState>          states;
    const VariableSource*              variables;
    std::vector<double>                last_touch;
    std::vector<std::optional<double>> last_values;
    std::vector<RunEvent>              logged;
    std::size_t                        collision_count{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_EVENT_LOG_HPP

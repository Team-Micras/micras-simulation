/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <span>
#include <utility>

#include "micras/sim/recording/event_log.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Find the geom a contact pairs with a given one.
 *
 * @param contact The contact.
 * @param geom The geom on one side.
 * @return The geom on the other side, or -1 when the contact does not involve the given one.
 */
int other_geom(const mjContact& contact, int geom) {
    if (contact.geom[0] == geom) {
        return contact.geom[1];
    }

    if (contact.geom[1] == geom) {
        return contact.geom[0];
    }

    return -1;
}
}  // namespace

EventLog::EventLog(
    std::vector<int> geoms, std::set<int> ignored, std::vector<WatchedState> states, const VariableSource* variables
) :
    geoms{std::move(geoms)},
    ignored{std::move(ignored)},
    states{std::move(states)},
    variables{variables},
    last_touch(this->geoms.size(), -std::numeric_limits<double>::infinity()),
    last_values(this->states.size()) { }

void EventLog::watch(WatchedState state) {
    this->states.push_back(std::move(state));
    this->last_values.emplace_back();
}

void EventLog::add(RunEvent event) {
    if (this->logged.size() < max_events) {
        this->logged.push_back(std::move(event));
    }
}

void EventLog::on_after_tick(const Simulation& simulation) {
    const MujocoWorld& world = simulation.context().world;
    const mjData*      data = world.data();

    this->log_collisions(
        world.model(), std::span<const mjContact>(data->contact, static_cast<std::size_t>(data->ncon)), data->time
    );

    if (this->variables != nullptr) {
        this->log_states(data->time);
    }
}

void EventLog::log_collisions(const mjModel* model, std::span<const mjContact> contacts, double now) {
    for (std::size_t index = 0; index < this->geoms.size(); index++) {
        const int  geom = this->geoms[index];
        const auto touches = std::ranges::any_of(contacts, [this, geom](const mjContact& contact) {
            const int other = other_geom(contact, geom);
            return other >= 0 and not this->ignored.contains(other);
        });

        if (not touches) {
            continue;
        }

        if (now - this->last_touch[index] > min_separation) {
            this->collision_count++;
            const char* name = mj_id2name(model, mjOBJ_GEOM, geom);
            this->add({.time = now, .kind = "collision", .detail = name == nullptr ? std::to_string(geom) : name});
        }

        this->last_touch[index] = now;
    }
}

void EventLog::log_states(double now) {
    for (std::size_t index = 0; index < this->states.size(); index++) {
        const WatchedState& state = this->states[index];
        const double        value = this->variables->value_of(state.variable);

        if (std::isnan(value) or this->last_values[index] == value) {
            continue;
        }

        this->last_values[index] = value;
        const bool  named = value >= 0.0 and value < static_cast<double>(state.names.size());
        std::string name = named ? state.names[static_cast<std::size_t>(value)] : std::format("{}", value);
        this->add({.time = now, .kind = state.variable, .detail = std::move(name)});
    }
}
}  // namespace micras::sim

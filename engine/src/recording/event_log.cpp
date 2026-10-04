/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include <mujoco/mujoco.h>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/recording/event_log.hpp"
#include "micras/sim/recording/run_metadata.hpp"

namespace micras::sim {
/**
 * @brief Find the geom a contact pairs with a given one.
 *
 * @param contact The contact.
 * @param geom The geom on one side.
 * @return The geom on the other side, or -1 when the contact does not involve the given one.
 */
static int other_geom(const mjContact& contact, int geom) {
    if (contact.geom[0] == geom) {
        return contact.geom[1];
    }

    if (contact.geom[1] == geom) {
        return contact.geom[0];
    }

    return -1;
}

EventLog::EventLog(std::vector<int> geoms, std::set<int> ignored, StateNames states, const VariableSource* variables) :
    geoms{std::move(geoms)},
    ignored{std::move(ignored)},
    states{std::move(states)},
    variables{variables},
    last_touch(this->geoms.size(), -std::numeric_limits<double>::infinity()),
    last_values(this->states.size()) { }

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
        const int  geom = this->geoms.at(index);
        const auto touches = std::ranges::any_of(contacts, [this, geom](const mjContact& contact) {
            const int other = other_geom(contact, geom);
            return other >= 0 and not this->ignored.contains(other);
        });

        if (not touches) {
            continue;
        }

        if (now - this->last_touch.at(index) > min_separation) {
            this->collision_count++;
            const char* name = mj_id2name(model, mjOBJ_GEOM, geom);
            this->add({.time = now, .kind = "collision", .detail = name == nullptr ? std::to_string(geom) : name});
        }

        this->last_touch.at(index) = now;
    }
}

void EventLog::log_states(double now) {
    std::size_t index = 0;

    for (const auto& [variable, names] : this->states) {
        const double           value = this->variables->value_of(variable);
        std::optional<double>& last = this->last_values.at(index++);

        if (std::isnan(value) or last == value) {
            continue;
        }

        last = value;
        const bool  named = value >= 0.0 and value < static_cast<double>(names.size());
        std::string name = named ? names.at(static_cast<std::size_t>(value)) : std::format("{}", value);
        this->add({.time = now, .kind = variable, .detail = std::move(name)});
    }
}
}  // namespace micras::sim

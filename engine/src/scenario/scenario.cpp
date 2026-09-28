/**
 * @file
 */

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <toml++/toml.hpp>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/text_file.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/scenario/scenario.hpp"

namespace micras::sim {
/**
 * @brief Refuse keys of a table outside a known set.
 *
 * @param table Table to check.
 * @param known Keys it may have.
 * @param where Name of the table, for messages.
 */
static void
    refuse_unknown(const toml::table& table, const std::set<std::string_view>& known, const std::string& where) {
    for (const auto& [key, node] : table) {
        if (not known.contains(key.str())) {
            throw std::runtime_error(std::format("{}: unknown key {}", where, key.str()));
        }
    }
}

/**
 * @brief Read a required number.
 *
 * @param table Table to read.
 * @param key Key.
 * @param where Name of the table, for messages.
 * @return The number.
 */
static double require_number(const toml::table& table, std::string_view key, const std::string& where) {
    const std::optional<double> value = table[key].value<double>();

    if (not value.has_value()) {
        throw std::runtime_error(std::format("{}: {} must be a number", where, key));
    }

    return *value;
}

/**
 * @brief Format a stop condition's value as text, a number or a name.
 *
 * @param node The value's node.
 * @param where Name of the table, for messages.
 * @return The value as text.
 */
static std::string condition_value(const toml::node_view<const toml::node>& node, const std::string& where) {
    if (const auto text = node.value<std::string>(); text.has_value()) {
        return *text;
    }

    if (const auto number = node.value<double>(); number.has_value()) {
        return std::format("{}", *number);
    }

    throw std::runtime_error(where + ": equals must be a number or a name");
}

/**
 * @brief Read a condition's values, given as one value or as an array of them.
 *
 * @param node The values' node; a missing one gives no value.
 * @param where Name of the table, for messages.
 * @return The values as text.
 */
static std::vector<std::string>
    condition_values(const toml::node_view<const toml::node>& node, const std::string& where) {
    std::vector<std::string> values;

    if (const toml::array* array = node.as_array(); array != nullptr) {
        for (const toml::node& value : *array) {
            values.push_back(condition_value(toml::node_view<const toml::node>{value}, where));
        }
    } else if (node) {
        values.push_back(condition_value(node, where));
    }

    return values;
}

/**
 * @brief Read the stop condition.
 *
 * @param table The [stop] table.
 * @param where Name of the table, for messages.
 * @return The condition.
 */
static StopCondition read_stop(const toml::table& table, const std::string& where) {
    refuse_unknown(table, {"when", "equals", "after", "count", "abort"}, where);
    const std::optional<std::string> variable = table["when"].value<std::string>();

    if (not variable.has_value()) {
        throw std::runtime_error(where + ": when must name a firmware variable");
    }

    StopCondition stop{
        .variable = *variable,
        .equals = condition_values(table["equals"], where),
        .after = table["after"].value_or(0.0),
        .count = static_cast<int>(table["count"].value_or(int64_t{1})),
        .abort = condition_values(table["abort"], where),
    };

    if (stop.equals.empty()) {
        throw std::runtime_error(where + ": equals names no value");
    }

    if (stop.count < 1) {
        throw std::runtime_error(where + ": count must be at least 1");
    }

    return stop;
}

/**
 * @brief Read one event of the timeline.
 *
 * @param table The event's table.
 * @param where Name of the event, for messages.
 * @return The event.
 */
static ScenarioEvent read_event(const toml::table& table, const std::string& where) {
    refuse_unknown(table, {"at", "press", "for", "set", "value", "send", "when", "equals"}, where);

    ScenarioEvent event;
    event.at = require_number(table, "at", where);
    const int kinds = static_cast<int>(table.contains("press")) + static_cast<int>(table.contains("set")) +
                      static_cast<int>(table.contains("send"));

    if (kinds != 1) {
        throw std::runtime_error(where + ": an event is exactly one of press, set or send");
    }

    if (const auto press = table["press"].value<std::string>(); press.has_value()) {
        event.kind = ScenarioEvent::Kind::PRESS;
        event.target = *press;
        event.duration = require_number(table, "for", where);
    } else if (const auto set = table["set"].value<std::string>(); set.has_value()) {
        event.kind = ScenarioEvent::Kind::SET;
        event.target = *set;
        const std::optional<bool> value = table["value"].value<bool>();

        if (not value.has_value()) {
            throw std::runtime_error(where + ": set needs a true or false value");
        }

        event.value = *value;
    } else {
        const std::optional<std::string> send = table["send"].value<std::string>();

        if (not send.has_value()) {
            throw std::runtime_error(where + ": send must name a message");
        }

        event.kind = ScenarioEvent::Kind::SEND;
        event.target = *send;
    }

    if (event.at < 0.0 or event.duration < 0.0) {
        throw std::runtime_error(where + ": times cannot be negative");
    }

    if (table.contains("when") != table.contains("equals")) {
        throw std::runtime_error(where + ": a condition needs both when and equals");
    }

    if (table.contains("when")) {
        const std::optional<std::string> variable = table["when"].value<std::string>();

        if (not variable.has_value()) {
            throw std::runtime_error(where + ": when must name a firmware variable");
        }

        event.when = *variable;
        event.equals = condition_value(table["equals"], where);
    }

    return event;
}

Scenario Scenario::load(const std::filesystem::path& path) {
    return parse(read_text_file(path, "scenario"), path.string());
}

Scenario Scenario::parse(std::string_view text, const std::string& origin) {
    toml::table table;

    try {
        table = toml::parse(text, origin);
    } catch (const toml::parse_error& error) {
        throw std::runtime_error(std::format("{}: {}", origin, error.description()));
    }

    refuse_unknown(table, {"robot", "arena", "seconds", "seed", "start", "events", "stop"}, origin);

    Scenario scenario;
    scenario.robot = table["robot"].value_or(std::string{});
    scenario.arena = table["arena"].value_or(std::string{});
    scenario.seconds = table["seconds"].value_or(0.0);

    if (const auto seed = table["seed"].value<int64_t>(); seed.has_value()) {
        scenario.seed = static_cast<uint64_t>(*seed);
    }

    if (const toml::table* start = table["start"].as_table(); start != nullptr) {
        const std::string where = origin + ": [start]";
        refuse_unknown(*start, {"x", "y", "yaw_deg"}, where);
        scenario.start = MujocoWorld::Placement{
            .x = require_number(*start, "x", where),
            .y = require_number(*start, "y", where),
            .yaw = require_number(*start, "yaw_deg", where) * std::numbers::pi / 180.0,
        };
    }

    if (const toml::array* events = table["events"].as_array(); events != nullptr) {
        for (std::size_t index = 0; index < events->size(); index++) {
            const toml::table* event = events->at(index).as_table();
            const std::string  where = std::format("{}: events[{}]", origin, index);

            if (event == nullptr) {
                throw std::runtime_error(where + " must be a table");
            }

            scenario.events.push_back(read_event(*event, where));
        }
    }

    std::ranges::stable_sort(scenario.events, {}, &ScenarioEvent::at);

    if (const toml::table* stop = table["stop"].as_table(); stop != nullptr) {
        scenario.stop = read_stop(*stop, origin + ": [stop]");
    }

    return scenario;
}

ScenarioPlayer::ScenarioPlayer(
    Scenario scenario, const ScenarioHooks& hooks, SerialBus& serial, const VariableSource* variables
) :
    scenario{std::move(scenario)}, hooks{hooks}, serial{serial}, variables{variables} {
    for (const ScenarioEvent& event : this->scenario.events) {
        this->check(event);
        this->event_values.push_back(
            event.when.empty() ? std::numeric_limits<double>::quiet_NaN() : this->resolve(event.when, event.equals)
        );
    }

    if (not this->scenario.stop.has_value()) {
        return;
    }

    const StopCondition& stop = *this->scenario.stop;

    for (const std::string& value : stop.equals) {
        this->stop_values.push_back(this->resolve(stop.variable, value));
    }

    for (const std::string& value : stop.abort) {
        this->abort_values.push_back(this->resolve(stop.variable, value));
    }
}

double ScenarioPlayer::resolve(const std::string& variable, const std::string& value) const {
    double      number{};
    const char* last = std::to_address(value.cend());

    if (std::from_chars(value.data(), last, number).ptr == last) {
        return number;
    }

    const auto names = this->hooks.state_names.find(variable);

    if (names == this->hooks.state_names.end()) {
        throw std::runtime_error("the scenario names state " + value + ", but " + variable + " has no state names");
    }

    const auto found = std::ranges::find(names->second, value);

    if (found == names->second.end()) {
        throw std::runtime_error(variable + " has no state named " + value);
    }

    return static_cast<double>(std::distance(names->second.begin(), found));
}

bool ScenarioPlayer::is_due(std::size_t index) const {
    const ScenarioEvent& event = this->scenario.events.at(index);

    if (event.when.empty()) {
        return true;
    }

    return this->variables != nullptr and this->variables->value_of(event.when) == this->event_values.at(index);
}

void ScenarioPlayer::check(const ScenarioEvent& event) const {
    switch (event.kind) {
        case ScenarioEvent::Kind::PRESS:
        case ScenarioEvent::Kind::SET:
            if (not this->hooks.inputs.contains(event.target)) {
                throw std::runtime_error("the robot has no input named " + event.target);
            }

            break;

        case ScenarioEvent::Kind::SEND:
            if (not this->hooks.messages.contains(event.target)) {
                throw std::runtime_error("the robot has no message named " + event.target);
            }

            break;
    }
}

RunControl ScenarioPlayer::on_before_tick(const Simulation& simulation) {
    const Clock&   clock = simulation.context().clock;
    const uint64_t tick = simulation.tick();
    const double   now = static_cast<double>(tick) * clock.us_per_tick() * 1e-6;

    for (auto release = this->releases.begin(); release != this->releases.end();) {
        if (tick >= release->tick) {
            if (not this->human) {
                release->input->set(false);
            }

            release = this->releases.erase(release);
        } else {
            ++release;
        }
    }

    while (this->next_event < this->scenario.events.size() and
           clock.tick_at(this->scenario.events.at(this->next_event).at) <= tick and this->is_due(this->next_event)) {
        const ScenarioEvent& event = this->scenario.events.at(this->next_event++);
        const uint64_t       end = tick + std::max<uint64_t>(1, clock.tick_at(event.duration));

        switch (event.kind) {
            case ScenarioEvent::Kind::PRESS:
                if (not this->human) {
                    DigitalInput* input = this->hooks.inputs.find(event.target)->second;
                    input->set(true);
                    this->releases.push_back({.input = input, .tick = end});
                }

                break;

            case ScenarioEvent::Kind::SET:
                if (not this->human) {
                    this->hooks.inputs.find(event.target)->second->set(event.value);
                }

                break;

            case ScenarioEvent::Kind::SEND:
                this->serial.queue_for_firmware(this->hooks.messages.find(event.target)->second);
                break;
        }
    }

    return this->check_stop(now);
}

RunControl ScenarioPlayer::check_stop(double now) {
    if (not this->scenario.stop.has_value() or this->variables == nullptr or this->stop_time.has_value()) {
        return this->stop_time.has_value() ? RunControl::QUIT : RunControl::RUN;
    }

    const StopCondition& stop = *this->scenario.stop;

    if (now < stop.after) {
        return RunControl::RUN;
    }

    const double current = this->variables->value_of(stop.variable);
    const auto   found = std::ranges::find(this->stop_values, current);
    const bool   matching = found != this->stop_values.end();

    if (matching and not this->stop_matching) {
        this->stop_reached++;
    }

    this->stop_matching = matching;

    if (matching and this->stop_reached >= stop.count) {
        this->stop_time = now;
        const auto index = static_cast<std::size_t>(std::distance(this->stop_values.begin(), found));
        std::cout << "stop condition " << stop.variable << " == " << stop.equals.at(index) << " held at " << now
                  << " s\n";
        return RunControl::QUIT;
    }

    const auto aborted = std::ranges::find(this->abort_values, current);

    if (aborted != this->abort_values.end()) {
        this->stop_time = now;
        const auto index = static_cast<std::size_t>(std::distance(this->abort_values.begin(), aborted));
        std::cout << "stop condition " << stop.variable << " == " << stop.abort.at(index) << " (abort) held at " << now
                  << " s\n";
        return RunControl::QUIT;
    }

    return RunControl::RUN;
}
}  // namespace micras::sim

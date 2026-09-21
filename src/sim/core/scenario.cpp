/**
 * @file
 */

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "micras/sim/core/scenario.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Name each command answers to on the command line.
 */
constexpr std::array<std::pair<std::string_view, Command>, 4> command_names{
    {{"none", Command::NONE},
     {"explore", Command::EXPLORE},
     {"solve", Command::SOLVE},
     {"calibrate", Command::CALIBRATE}}
};

/**
 * @brief Name each button press answers to on the command line.
 */
constexpr std::array<std::pair<std::string_view, ButtonPress>, 4> press_names{
    {{"none", ButtonPress::NONE},
     {"short", ButtonPress::SHORT},
     {"long", ButtonPress::LONG},
     {"extra_long", ButtonPress::EXTRA_LONG}}
};
}  // namespace

std::optional<Command> parse_command(std::string_view name) {
    const auto* const entry = std::ranges::find(command_names, name, &std::pair<std::string_view, Command>::first);
    return entry == command_names.end() ? std::nullopt : std::optional<Command>{entry->second};
}

std::optional<ButtonPress> parse_button_press(std::string_view name) {
    const auto* const entry = std::ranges::find(press_names, name, &std::pair<std::string_view, ButtonPress>::first);
    return entry == press_names.end() ? std::nullopt : std::optional<ButtonPress>{entry->second};
}

std::string_view command_variable(Command command) {
    switch (command) {
        case Command::EXPLORE:
            return "Explore";

        case Command::SOLVE:
            return "Solve";

        case Command::CALIBRATE:
            return "Calibrate";

        case Command::NONE:
            break;
    }

    return {};
}

uint32_t Scenario::hold_time_ms(ButtonPress press, const ButtonDelays& delays) {
    switch (press) {
        case ButtonPress::SHORT:
            return delays.long_press / 2;

        case ButtonPress::LONG:
            return delays.long_press + 1;

        case ButtonPress::EXTRA_LONG:
            return delays.extra_long_press + 1;

        case ButtonPress::NONE:
            break;
    }

    return 0;
}

Scenario::Scenario(
    const ScenarioScript& script, ProxyState& state, Telemetry& telemetry, const Clock& clock,
    const ButtonDelays& delays
) :
    script{script},
    state{state},
    telemetry{telemetry},
    command_tick{std::max<uint64_t>(1, clock.tick_at(script.command_at))},
    button_press_tick{clock.tick_at(script.button_at)},
    button_release_tick{this->button_press_tick + clock.ticks_for_elapsed_ms(hold_time_ms(script.button, delays))},
    command_sent{script.command == Command::NONE} { }

void Scenario::on_start(const Simulation& /*simulation*/) {
    this->state.overrides.fan_enabled = this->script.fan_enabled;
    this->state.interface_input.dip_switches = this->script.dip;
}

RunControl Scenario::on_before_tick(const Simulation& simulation) {
    const uint64_t tick = simulation.tick();

    if (this->state.interface_input.driven_by_human) {
        return RunControl::RUN;
    }

    if (this->script.button != ButtonPress::NONE) {
        this->state.interface_input.button_pressed =
            tick >= this->button_press_tick and tick < this->button_release_tick;
    }

    if (this->command_sent or tick < this->command_tick or not this->telemetry.has_variable_map()) {
        return RunControl::RUN;
    }

    const std::string             name{command_variable(this->script.command)};
    const std::optional<uint16_t> id = this->telemetry.find_variable_id(name);

    if (not id.has_value()) {
        throw std::runtime_error("firmware pool has no variable named " + name);
    }

    this->telemetry.send_variable(*id, {0x01});
    this->command_sent = true;
    std::cout << "injected " << name << "=true at tick " << tick << " (id " << *id << ")\n";

    return RunControl::RUN;
}
}  // namespace micras::sim

/**
 * @file
 */

#include <algorithm>
#include <array>
#include <charconv>
#include <functional>
#include <stdexcept>

#include "micras/sim/app/cli.hpp"
#include "micras/sim/core/clock.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Read the value that follows an option.
 *
 * @param arguments Whole command line.
 * @param index Index of the option, advanced to the value.
 * @return The value.
 */
std::string take_value(std::span<char*> arguments, std::size_t& index) {
    if (index + 1 >= arguments.size()) {
        throw std::runtime_error("missing value for option " + std::string(arguments[index]));
    }

    return arguments[++index];
}

/**
 * @brief Parse the --dip option into the scenario switch states.
 *
 * @param value Comma separated list of name=0|1 pairs.
 * @param script Script to fill, indexed as in Interface::DipSwitchPins.
 */
void parse_dip(const std::string& value, ScenarioScript& script) {
    const std::array<std::string, InterfaceInput::dip_switch_count> names{"fan", "diagonal", "boost", "risky"};
    std::size_t                                                     start = 0;

    while (start <= value.size()) {
        const std::size_t comma = value.find(',', start);
        const std::string item = value.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        start = comma == std::string::npos ? value.size() + 1 : comma + 1;

        if (item.empty()) {
            continue;
        }

        const std::size_t equals = item.find('=');

        if (equals == std::string::npos) {
            throw std::runtime_error("malformed --dip entry '" + item + "', expected name=0|1");
        }

        const std::string name = item.substr(0, equals);
        const std::string state = item.substr(equals + 1);
        const auto* const slot = std::ranges::find(names, name);

        if (slot == names.end()) {
            throw std::runtime_error("unknown dip switch '" + name + "'");
        }

        if (state != "0" and state != "1") {
            throw std::runtime_error("dip switch '" + name + "' must be 0 or 1");
        }

        script.dip.at(static_cast<std::size_t>(slot - names.begin())) = state == "1";
    }
}

/**
 * @brief Parse a number, naming the option and rejecting trailing rubbish.
 *
 * @tparam T Type to parse.
 * @param value Number as spelled on the command line.
 * @param option Name of the option, for the error message.
 * @return The number.
 */
template <typename T>
T require_number(const std::string& value, const std::string& option) {
    T                           number{};
    const std::span<const char> digits(value);
    const auto                  result = std::from_chars(digits.data(), digits.data() + digits.size(), number);

    if (result.ec != std::errc{} or result.ptr != digits.data() + digits.size()) {
        throw std::runtime_error(option + " expects a number, got '" + value + "'");
    }

    return number;
}

/**
 * @brief Parse a size spelled as <width>x<height>.
 *
 * @param value Size as spelled on the command line.
 * @param option Name of the option, for the error message.
 * @param width Width to fill.
 * @param height Height to fill.
 */
void parse_size(const std::string& value, const std::string& option, int& width, int& height) {
    const std::size_t cross = value.find('x');

    if (cross == std::string::npos) {
        throw std::runtime_error(option + " must look like 1280x720, got " + value);
    }

    width = require_number<int>(value.substr(0, cross), option);
    height = require_number<int>(value.substr(cross + 1), option);
}

/**
 * @brief Options being built, plus what the parser has to remember about them.
 */
struct ParsedOptions {
    CliOptions parsed;
    bool       ticks_given{false};
};

/**
 * @brief Reads the value that follows the option being applied.
 */
using ValueReader = std::function<std::string()>;

/**
 * @brief Parse a command name, naming the offender when it is unknown.
 *
 * @param value Name as spelled on the command line.
 * @return The command.
 */
Command require_command(const std::string& value) {
    const auto command = parse_command(value);

    if (not command.has_value()) {
        throw std::runtime_error("unknown --command " + value);
    }

    return *command;
}

/**
 * @brief Parse a button press name, naming the offender when it is unknown.
 *
 * @param value Name as spelled on the command line.
 * @return The press.
 */
ButtonPress require_button_press(const std::string& value) {
    const auto press = parse_button_press(value);

    if (not press.has_value()) {
        throw std::runtime_error("unknown --button " + value);
    }

    return *press;
}

/**
 * @brief Apply one of the window options.
 *
 * @param argument Name of the option.
 * @param value Reader of the value that follows it.
 * @param options Options to fill.
 * @return False when the name is not a window option.
 */
bool apply_viewer_option(const std::string& argument, const ValueReader& value, CliOptions& options) {
    if (argument == "--viewer") {
        options.viewer_enabled = true;
    } else if (argument == "--viewer-camera") {
        options.viewer.camera = value();
    } else if (argument == "--viewer-fps") {
        options.viewer_fps = require_number<int>(value(), argument);
    } else if (argument == "--viewer-size") {
        parse_size(value(), "--viewer-size", options.viewer.width, options.viewer.height);
    } else if (argument == "--monitor") {
        options.monitor_enabled = true;
    } else if (argument == "--monitor-port") {
        options.monitor_port = require_number<int>(value(), argument);
    } else {
        return false;
    }

    return true;
}

/**
 * @brief Apply one of the recording options.
 *
 * @param argument Name of the option.
 * @param value Reader of the value that follows it.
 * @param options Options to fill.
 * @return False when the name is not a recording option.
 */
bool apply_video_option(const std::string& argument, const ValueReader& value, CliOptions& options) {
    if (argument == "--video") {
        options.video.path = value();
        options.video_enabled = true;
    } else if (argument == "--video-fps") {
        options.video.fps = require_number<int>(value(), argument);
    } else if (argument == "--video-camera") {
        options.video.camera = value();
    } else if (argument == "--video-size") {
        parse_size(value(), "--video-size", options.video.width, options.video.height);
    } else {
        return false;
    }

    return true;
}

/**
 * @brief Reject an option combination the run could not honour.
 *
 * @param options Options as parsed.
 * @param ticks_given Whether --ticks was given.
 */
void validate(const CliOptions& options, bool ticks_given) {
    if (options.model.empty()) {
        throw std::runtime_error("--model is required");
    }

    if (options.out.empty()) {
        throw std::runtime_error("--out is required");
    }

    if (ticks_given) {
        if (options.ticks == 0) {
            throw std::runtime_error("--ticks must be at least 1");
        }
    } else if (not(options.seconds > 0.0)) {
        throw std::runtime_error("--seconds must be strictly positive, got " + std::to_string(options.seconds));
    }

    if (options.viewer_enabled) {
        if (options.viewer.width < 1 or options.viewer.height < 1) {
            throw std::runtime_error("--viewer-size must be strictly positive");
        }

        if (options.viewer_fps < 1) {
            throw std::runtime_error("--viewer-fps must be at least 1");
        }
    }

    if (options.monitor_enabled and (options.monitor_port < 1 or options.monitor_port > 65535)) {
        throw std::runtime_error("--monitor-port must be a TCP port");
    }

    if (not options.video_enabled) {
        return;
    }

    if (options.video.fps < 1) {
        throw std::runtime_error("--video-fps must be at least 1");
    }

    if (options.video.width < 1 or options.video.height < 1) {
        throw std::runtime_error("--video-size must be strictly positive");
    }
}

/**
 * @brief Apply one option to the options being built.
 *
 * @param argument Name of the option.
 * @param value Reader of the value that follows it.
 * @param options Options to fill.
 * @return False when the name is not an option at all.
 */
bool apply_option(const std::string& argument, const ValueReader& value, ParsedOptions& options) {
    if (argument == "--model") {
        options.parsed.model = value();
    } else if (argument == "--out") {
        options.parsed.out = value();
    } else if (argument == "--maze") {
        options.parsed.maze = value();
    } else if (argument == "--seconds") {
        options.parsed.seconds = require_number<double>(value(), argument);
    } else if (argument == "--ticks") {
        options.parsed.ticks = require_number<uint64_t>(value(), argument);
        options.ticks_given = true;
    } else if (argument == "--command") {
        options.parsed.scenario.command = require_command(value());
    } else if (argument == "--command-at") {
        options.parsed.scenario.command_at = require_number<double>(value(), argument);
    } else if (argument == "--dip") {
        parse_dip(value(), options.parsed.scenario);
    } else if (argument == "--button") {
        options.parsed.scenario.button = require_button_press(value());
    } else if (argument == "--button-at") {
        options.parsed.scenario.button_at = require_number<double>(value(), argument);
    } else if (argument == "--no-fan") {
        options.parsed.scenario.fan_enabled = false;
    } else {
        return apply_video_option(argument, value, options.parsed) or
               apply_viewer_option(argument, value, options.parsed);
    }

    return true;
}
}  // namespace

std::string Cli::usage() {
    return "usage: micras_simulation --model <xml> --out <dir> [--seconds <float> | --ticks <int>]\n"
           "                     [--maze <xml>] [--command none|explore|solve|calibrate] [--command-at <seconds>]\n"
           "                     [--dip fan=0,diagonal=0,boost=0,risky=0]\n"
           "                     [--button none|short|long|extra_long] [--button-at <seconds>] [--no-fan]\n"
           "                     [--video <out.mp4>] [--video-fps <int>] [--video-camera <name>|free]\n"
           "                     [--video-size <width>x<height>]\n"
           "                     [--viewer] [--viewer-camera <name>|free] [--viewer-fps <int>]\n"
           "                     [--viewer-size <width>x<height>]\n"
           "                     [--monitor] [--monitor-port <int>]\n";
}

CliOptions Cli::parse(std::span<char*> arguments, uint32_t loop_time_us) {
    ParsedOptions options;

    for (std::size_t i = 1; i < arguments.size(); i++) {
        const std::string argument = arguments[i];
        const ValueReader value = [&arguments, &i] { return take_value(arguments, i); };

        if (not apply_option(argument, value, options)) {
            throw std::runtime_error("unknown option " + argument);
        }
    }

    validate(options.parsed, options.ticks_given);

    if (not options.ticks_given) {
        options.parsed.ticks = Clock::total_ticks(options.parsed.seconds, loop_time_us);

        if (options.parsed.ticks == 0) {
            throw std::runtime_error(
                "--seconds " + std::to_string(options.parsed.seconds) + " is shorter than one firmware loop period"
            );
        }
    }

    return options.parsed;
}
}  // namespace micras::sim

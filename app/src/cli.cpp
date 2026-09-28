/**
 * @file
 */

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include "micras/sim/app/cli.hpp"
#include "micras/sim/app/wiring.hpp"
#include "micras/sim/core/span_at.hpp"

namespace micras::sim {
/**
 * @brief Read the value that follows an option.
 *
 * @param arguments Whole command line.
 * @param index Index of the option, advanced to the value.
 * @return The value.
 */
static std::string take_value(std::span<char*> arguments, std::size_t& index) {
    if (index + 1 >= arguments.size()) {
        throw std::runtime_error("missing value for option " + std::string(at(arguments, index)));
    }

    return at(arguments, ++index);
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
static T require_number(const std::string& value, const std::string& option) {
    T                           number{};
    const std::span<const char> digits(value);
    const char* const           end = std::to_address(digits.end());
    const auto                  result = std::from_chars(digits.data(), end, number);

    if (result.ec != std::errc{} or result.ptr != end) {
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
static void parse_size(const std::string& value, const std::string& option, int& width, int& height) {
    const std::size_t cross = value.find('x');

    if (cross == std::string::npos) {
        throw std::runtime_error(option + " must look like 1280x720, got " + value);
    }

    width = require_number<int>(value.substr(0, cross), option);
    height = require_number<int>(value.substr(cross + 1), option);
}

namespace {
/**
 * @brief Reads the value that follows the option being applied.
 */
using ValueReader = std::function<std::string()>;
}  // namespace

/**
 * @brief Apply one of the window options.
 *
 * @param argument Name of the option.
 * @param value Reader of the value that follows it.
 * @param options Options to fill.
 * @return False when the name is not a window option.
 */
static bool apply_viewer_option(const std::string& argument, const ValueReader& value, CliOptions& options) {
    if (argument == "--viewer") {
        options.viewer_enabled = true;
    } else if (argument == "--viewer-camera") {
        options.viewer.camera = value();
    } else if (argument == "--viewer-fps") {
        options.viewer_fps = require_number<int>(value(), argument);
    } else if (argument == "--viewer-size") {
        parse_size(value(), "--viewer-size", options.viewer.width, options.viewer.height);
    } else {
        return false;
    }

    return true;
}

/**
 * @brief Apply one of the monitor bridge options.
 *
 * @param argument Name of the option.
 * @param value Reader of the value that follows it.
 * @param options Options to fill.
 * @return False when the name is not a monitor option.
 */
static bool apply_monitor_option(const std::string& argument, const ValueReader& value, CliOptions& options) {
    if (argument == "--monitor") {
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
static bool apply_video_option(const std::string& argument, const ValueReader& value, CliOptions& options) {
    if (argument == "--video") {
        options.video.path = value();
    } else if (argument == "--video-fps") {
        options.video.fps = require_number<int>(value(), argument);
    } else if (argument == "--video-camera") {
        options.video.camera = value();
    } else if (argument == "--video-size") {
        parse_size(value(), "--video-size", options.video.width, options.video.height);
    } else if (argument == "--video-trail") {
        options.video.trail = true;
    } else {
        return false;
    }

    return true;
}

/**
 * @brief Reject an option combination the run could not honor.
 *
 * @param options Options as parsed.
 */
static void validate(const CliOptions& options) {
    if (options.out.empty()) {
        throw std::runtime_error("--out is required");
    }

    if (options.ticks.has_value() and *options.ticks == 0) {
        throw std::runtime_error("--ticks must be at least 1");
    }

    if (options.seconds.has_value() and not(*options.seconds > 0.0)) {
        throw std::runtime_error("--seconds must be strictly positive, got " + std::to_string(*options.seconds));
    }

    if (options.record_every == 0) {
        throw std::runtime_error("--record-every must be at least 1");
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

    if (options.video.path.empty()) {
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
static bool apply_option(const std::string& argument, const ValueReader& value, CliOptions& options) {
    if (argument == "--out") {
        options.out = value();
    } else if (argument == "--scenario") {
        options.scenario = value();
    } else if (argument == "--maze") {
        options.maze = value();
    } else if (argument == "--seconds") {
        options.seconds = require_number<double>(value(), argument);
    } else if (argument == "--ticks") {
        options.ticks = require_number<uint64_t>(value(), argument);
    } else if (argument == "--seed") {
        options.seed = require_number<uint64_t>(value(), argument);
    } else if (argument == "--ideal") {
        options.ideal = true;
    } else if (argument == "--record-every") {
        options.record_every = require_number<uint32_t>(value(), argument);
    } else {
        return apply_video_option(argument, value, options) or apply_viewer_option(argument, value, options) or
               apply_monitor_option(argument, value, options);
    }

    return true;
}

std::string Cli::usage(std::string_view program, std::span<const CliOption> target_options) {
    const std::string indent(7 + program.size(), ' ');
    std::string text = "usage: " + std::string(program) + " --out <dir> [--scenario <toml>] [--maze <name>|<txt>]\n" +
                       indent +
                       "[--seconds <float> | --ticks <int>] [--seed <int>] [--ideal] [--record-every <ticks>]\n" +
                       indent + "[--video <out.mp4>] [--video-fps <int>] [--video-camera <name>|free]\n" + indent +
                       "[--video-size <width>x<height>] [--video-trail]\n" + indent +
                       "[--viewer] [--viewer-camera <name>|free] [--viewer-fps <int>]\n" + indent +
                       "[--viewer-size <width>x<height>]\n" + indent + "[--monitor] [--monitor-port <int>]\n";

    for (const CliOption& option : target_options) {
        text += indent + "[" + option.name + (option.argument.empty() ? "" : " " + option.argument) + "]\n";
    }

    return text;
}

CliOptions Cli::parse(std::span<char*> arguments, std::span<const CliOption> target_options) {
    CliOptions options;

    for (std::size_t i = 1; i < arguments.size(); i++) {
        const std::string argument = at(arguments, i);
        const ValueReader value = [&arguments, &i] { return take_value(arguments, i); };

        if (apply_option(argument, value, options)) {
            continue;
        }

        const auto option = std::ranges::find(target_options, argument, &CliOption::name);

        if (option == target_options.end()) {
            throw std::runtime_error("unknown option " + argument);
        }

        option->apply(option->argument.empty() ? std::string{} : value());
    }

    validate(options);
    return options;
}
}  // namespace micras::sim

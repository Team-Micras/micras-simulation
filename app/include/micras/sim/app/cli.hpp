/**
 * @file
 *
 * @brief Command line of every simulation executable.
 */

#ifndef MICRAS_SIM_APP_CLI_HPP
#define MICRAS_SIM_APP_CLI_HPP

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "micras/sim/app/target.hpp"
#include "micras/sim/view/view_options.hpp"

namespace micras::sim {
/**
 * @brief Everything a run can be told from the command line.
 */
struct CliOptions {
    /**
     * @brief Directory the run is written to.
     */
    std::filesystem::path out;

    /**
     * @brief Scenario file to play; empty for the robot's default.
     */
    std::filesystem::path scenario;

    /**
     * @brief Maze to run in, a name from the arena's collection or a path; overrides the scenario's.
     */
    std::string maze;

    /**
     * @brief Length of the run, in simulated seconds or in exact ticks; overrides the scenario's.
     */
    ///@{
    std::optional<double>   seconds;
    std::optional<uint64_t> ticks;
    ///@}

    /**
     * @brief Seed of every noise stream; overrides the scenario's.
     */
    std::optional<uint64_t> seed;

    /**
     * @brief Whether the world is ideal: no noise, no bias, no scale error.
     */
    bool ideal{false};

    /**
     * @brief Ticks between two recorded rows.
     */
    uint32_t record_every{1};

    /**
     * @brief Offscreen recording, enabled by --video.
     */
    ///@{
    VideoConfig video;
    bool        video_enabled{false};
    ///@}

    /**
     * @brief Whether --video-camera was given, so the robot's preferred camera must not replace it.
     */
    bool video_camera_given{false};

    /**
     * @brief Live window, enabled by --viewer.
     */
    ///@{
    ViewerConfig viewer;
    int          viewer_fps{30};
    bool         viewer_enabled{false};
    ///@}

    /**
     * @brief Bridge to micras-monitor, enabled by --monitor.
     */
    ///@{
    int  monitor_port{8080};
    bool monitor_enabled{false};
    ///@}
};

/**
 * @brief Turns an argument vector into options, and nothing else.
 *
 * @note Pure: it never touches the filesystem, the model or the clock, so the
 *       whole command line surface is testable without a simulation. The
 *       robot target's own options are handed to their handlers as they come.
 */
class Cli {
public:
    /**
     * @brief Parse a command line.
     *
     * @param arguments Whole command line, program name included.
     * @param target_options Options the robot target adds.
     * @return Parsed options.
     */
    static CliOptions parse(std::span<char*> arguments, std::span<const CliOption> target_options = {});

    /**
     * @brief Get the usage text.
     *
     * @param program Name of the executable, as argv[0] gives it.
     * @param target_options Options the robot target adds.
     * @return Usage text, newline terminated.
     */
    static std::string usage(std::string_view program, std::span<const CliOption> target_options = {});
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_APP_CLI_HPP

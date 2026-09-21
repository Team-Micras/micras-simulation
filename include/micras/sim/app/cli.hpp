/**
 * @file
 *
 * @brief Command line of every simulation executable.
 */

#ifndef MICRAS_SIM_APP_CLI_HPP
#define MICRAS_SIM_APP_CLI_HPP

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>

#include "micras/sim/core/scenario.hpp"
#include "micras/sim/view/view_options.hpp"

namespace micras::sim {
/**
 * @brief Everything a run can be told from the command line.
 */
struct CliOptions {
    /**
     * @brief Model to load and directory the run is written to.
     */
    ///@{
    std::filesystem::path model;
    std::filesystem::path out;
    ///@}

    /**
     * @brief Maze recorded in the metadata; the model already attaches its own.
     */
    std::string maze;

    /**
     * @brief Scripted input for the run.
     */
    ScenarioScript scenario;

    /**
     * @brief Length of the run, in simulated seconds or in exact ticks.
     */
    ///@{
    double   seconds{10.0};
    uint64_t ticks{0};
    ///@}

    /**
     * @brief Offscreen recording, enabled by --video.
     */
    ///@{
    VideoConfig video;
    bool        video_enabled{false};
    ///@}

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
 *       whole command line surface is testable without a simulation.
 */
class Cli {
public:
    /**
     * @brief Parse a command line.
     *
     * @param arguments Whole command line, program name included.
     * @param loop_time_us Firmware loop period, used to turn seconds into ticks.
     * @return Parsed options.
     */
    static CliOptions parse(std::span<char*> arguments, uint32_t loop_time_us);

    /**
     * @brief Get the usage text.
     *
     * @return Usage text, newline terminated.
     */
    static std::string usage();
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_APP_CLI_HPP

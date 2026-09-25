/**
 * @file
 *
 * @brief What the command line can ask of the window and the recorder.
 *
 * @note A leaf header on purpose: the command line has to name these, and
 *       nothing that only parses options should have to pull in MuJoCo, GLFW or
 *       the run loop to do it.
 */

#ifndef MICRAS_SIM_VIEW_VIEW_OPTIONS_HPP
#define MICRAS_SIM_VIEW_VIEW_OPTIONS_HPP

#include <array>
#include <cstdint>
#include <string>

namespace micras::sim {
/**
 * @brief Everything the command line can configure about a recording.
 */
struct VideoConfig {
    /**
     * @brief Path of the mp4 file ffmpeg writes.
     */
    std::string path;

    /**
     * @brief Model camera name, or "free" for the fixed whole-world view.
     *
     * @note A robot target may prefer one of its own cameras; the command line
     *       overrides either.
     */
    std::string camera{"free"};

    /**
     * @brief Frame rate handed to ffmpeg.
     */
    int fps{30};

    /**
     * @brief Whether every frame draws the path the robot has traveled, as a line behind it.
     */
    bool trail{false};

    /**
     * @brief Color of the trail, red, green and blue in [0, 1].
     */
    std::array<float, 3> trail_color{1.0F, 0.35F, 0.05F};

    /**
     * @brief Whether the robot is drawn in the color of its trail, to tell apart two recordings of one maze.
     */
    bool tint{false};

    /**
     * @brief Frame size in pixels.
     */
    ///@{
    int width{1280};
    int height{720};
    ///@}
};

/**
 * @brief Everything the command line can configure about the window.
 */
struct ViewerConfig {
    /**
     * @brief Window title, the robot target's name.
     */
    std::string title;

    /**
     * @brief Body a ctrl-drag pushes, the robot's own.
     */
    std::string body;

    /**
     * @brief Window size in pixels.
     */
    ///@{
    int width{1200};
    int height{900};
    ///@}

    /**
     * @brief Model camera name, or "free" for the orbiting camera.
     */
    std::string camera{"free"};

    /**
     * @brief Number of firmware ticks between redraws.
     */
    uint64_t ticks_per_frame{16};

    /**
     * @brief Duration of one firmware tick, which the speed limiter paces against.
     *
     * @note No default: it comes from the clock of the loaded run.
     */
    uint32_t us_per_tick{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_VIEW_OPTIONS_HPP

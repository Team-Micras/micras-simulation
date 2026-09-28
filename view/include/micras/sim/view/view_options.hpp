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

#include <cstdint>
#include <string>

namespace micras::sim {
/**
 * @brief How a recording is made: the command line's options, and what the application fills in.
 */
struct VideoConfig {
    /**
     * @brief Path of the mp4 file ffmpeg writes, empty for no recording.
     */
    std::string path;

    /**
     * @brief Model camera name, "free" for the fixed whole-world view, or empty for the robot
     *        target's preferred camera.
     */
    std::string camera;

    /**
     * @brief Frame rate handed to ffmpeg.
     */
    int fps{30};

    /**
     * @brief Whether every frame draws the path the robot has traveled, as a line behind it.
     */
    bool trail{false};

    /**
     * @brief Frame size in pixels.
     */
    ///@{
    int width{1280};
    int height{720};
    ///@}

    /**
     * @brief Body the trail follows, the robot's own.
     */
    std::string body;

    /**
     * @brief Number of firmware ticks between frames.
     */
    uint64_t ticks_per_frame{1};
};

/**
 * @brief How the window is opened: the command line's options, and what the application fills in.
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
     */
    uint32_t us_per_tick{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_VIEW_OPTIONS_HPP

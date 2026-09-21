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
 * @brief Everything the command line can configure about a recording.
 */
struct VideoConfig {
    /**
     * @brief Path of the mp4 file ffmpeg writes.
     */
    std::string path;

    /**
     * @brief Model camera name, or "free" for the fixed whole-maze view.
     */
    std::string camera{"side tracking"};

    /**
     * @brief Frame rate handed to ffmpeg.
     */
    int fps{30};

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
    uint32_t us_per_tick{1042};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_VIEW_OPTIONS_HPP

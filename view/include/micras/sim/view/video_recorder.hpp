/**
 * @file
 *
 * @brief Offscreen EGL video recorder piping raw frames to ffmpeg.
 */

#ifndef MICRAS_SIM_VIEW_VIDEO_RECORDER_HPP
#define MICRAS_SIM_VIEW_VIDEO_RECORDER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/view/panel_spec.hpp"
#include "micras/sim/view/view_options.hpp"

namespace micras::sim {
/**
 * @brief Renders the simulation offscreen and streams it to ffmpeg.
 *
 * @note Rendering only reads mjModel/mjData, so a recorded run produces the
 *       exact same data.csv as the same run without --video.
 */
class VideoRecorder : public IRunListener {
public:
    /**
     * @brief Bring up EGL, the MuJoCo render context and the ffmpeg pipe.
     *
     * @note Grows the model's offscreen framebuffer to the frame size, since
     *       mjr_makeContext sizes it from the model.
     *
     * @param world World whose state is rendered.
     * @param config Recording configuration.
     * @param overlay What the robot target prints under the simulated time.
     * @param variables Source of the overlay's values, or null.
     * @param error Filled with a human readable reason when creation fails.
     * @return The recorder, or nullptr if anything failed.
     */
    static std::unique_ptr<VideoRecorder> create(
        MujocoWorld& world, const VideoConfig& config, OverlaySpec overlay, const VariableSource* variables,
        std::string& error
    );

    /**
     * @brief Extend the trail, then render the frame this tick falls on, if any.
     *
     * @note The trail is sampled every tick, a point every few millimeters, so that it follows
     * the curves however far apart the frames are.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

    VideoRecorder(const VideoRecorder&) = delete;
    VideoRecorder(VideoRecorder&&) = delete;
    VideoRecorder& operator=(const VideoRecorder&) = delete;
    VideoRecorder& operator=(VideoRecorder&&) = delete;

    /**
     * @brief Release the ffmpeg pipe, the render context and the EGL context.
     */
    ~VideoRecorder() override;

    /**
     * @brief Get how many frames were written so far.
     *
     * @return Frame count.
     */
    uint64_t frame_count() const { return this->frames; }

private:
    /**
     * @brief Distance between two points of the trail, in meters.
     */
    static constexpr double trail_spacing{0.003};

    /**
     * @brief Largest number of points of the trail, beyond which every other point is dropped.
     */
    static constexpr std::size_t max_trail_points{12000};

    /**
     * @brief Width of the trail, in pixels.
     */
    static constexpr float trail_width{3.0F};

    /**
     * @brief Color of the trail, red, green, blue and alpha.
     */
    static constexpr std::array<float, 4> trail_color{1.0F, 0.35F, 0.05F, 1.0F};

    VideoRecorder() = default;

    /**
     * @brief Extend the trail with where the body is now, if it has moved far enough.
     */
    void extend_trail();

    /**
     * @brief Add the trail to the scene, as one line per pair of points.
     */
    void draw_trail();

    /**
     * @brief Render one frame of the current state and write it to ffmpeg.
     *
     * @param labels Overlay labels, one per line.
     * @param values Overlay values, one per line.
     */
    void capture(const std::string& labels, const std::string& values);

    /**
     * @brief Make this recorder's offscreen context current.
     *
     * @note A window and a recorder can be open at the same time, and a thread
     *       cannot hold an EGL and a GLX context at once, so the recorder takes
     *       its context only while it draws and lets go afterwards.
     */
    void make_context_current() const;

    /**
     * @brief Let go of this recorder's offscreen context.
     */
    void release_context() const;

    /**
     * @brief Headless context the frames are rendered through.
     */
    ///@{
    void* display{nullptr};
    void* gl_context{nullptr};
    ///@}

    /**
     * @brief Recording configuration.
     */
    VideoConfig config;

    /**
     * @brief MuJoCo visualization state.
     */
    ///@{
    mjvScene   scene{};
    mjvCamera  camera{};
    mjvOption  option{};
    mjrContext context{};
    ///@}

    /**
     * @brief Pipe to the ffmpeg encoder.
     */
    std::FILE* encoder{nullptr};

    /**
     * @brief Frame buffers, bottom-up as read back and top-down as written out.
     */
    ///@{
    std::vector<unsigned char> pixels;
    std::vector<unsigned char> flipped;
    ///@}

    /**
     * @brief Number of frames written.
     */
    uint64_t frames{0};

    /**
     * @brief World being rendered.
     */
    MujocoWorld* world{nullptr};

    /**
     * @brief Source of the overlay's values.
     */
    const VariableSource* variables{nullptr};

    /**
     * @brief What the overlay prints under the simulated time.
     */
    OverlaySpec overlay;

    /**
     * @brief Body the trail follows, or -1 for no trail, and the points it has left.
     */
    ///@{
    int                                trail_body{-1};
    std::vector<std::array<double, 3>> trail;
    ///@}

    /**
     * @brief Whether the render context and scene were created.
     */
    bool context_ready{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_VIDEO_RECORDER_HPP

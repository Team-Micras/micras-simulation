/**
 * @file
 *
 * @brief Offscreen EGL video recorder piping raw frames to ffmpeg.
 */

#ifndef MICRAS_SIM_VIEW_VIDEO_RECORDER_HPP
#define MICRAS_SIM_VIEW_VIDEO_RECORDER_HPP

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "micras/sim/view/view_options.hpp"

#include "micras/sim/core/simulation.hpp"
#include "micras/sim/telemetry/telemetry.hpp"

namespace micras::sim {
/**
 * @brief Values drawn in the top-left HUD overlay of every frame.
 */
struct VideoHud {
    double sim_time{0.0};
    double fsm_state{0.0};
    double desired_linear{0.0};
    double odometry_linear{0.0};
};

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
     * @note The caller must have set model->vis.global.offwidth/offheight to at
     *       least the requested frame size before calling this, because
     *       mjr_makeContext sizes the offscreen framebuffer from them.
     *
     * @param model Model to render.
     * @param config Recording configuration.
     * @param error Filled with a human readable reason when creation fails.
     * @return The recorder, or nullptr if anything failed.
     */
    static std::unique_ptr<VideoRecorder> create(mjModel* model, const VideoConfig& config, std::string& error);

    /**
     * @brief Start recording a run, from the world and the pool it reads.
     *
     * @param world World whose state is rendered.
     * @param telemetry Decoder feeding the overlay.
     * @param ticks_per_frame Number of firmware ticks between frames.
     */
    void attach(MujocoWorld& world, const Telemetry& telemetry, uint64_t ticks_per_frame);

    /**
     * @brief Render the frame this tick falls on, if any.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

    VideoRecorder(const VideoRecorder&) = delete;
    VideoRecorder(VideoRecorder&&) = delete;
    VideoRecorder& operator=(const VideoRecorder&) = delete;
    VideoRecorder& operator=(VideoRecorder&&) = delete;

    /**
     * @brief Render one frame of the current state and write it to ffmpeg.
     *
     * @param data Simulation state to render; mjv_updateScene needs a mutable
     *             pointer but never touches the physics state.
     * @param hud Values drawn in the overlay.
     */
    void capture(mjData* data, const VideoHud& hud);

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
    VideoRecorder() = default;

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
     * @brief Model being rendered, owned by the World.
     */
    mjModel* model{nullptr};

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
     * @brief World being rendered, set by attach().
     */
    MujocoWorld* world{nullptr};

    /**
     * @brief Decoder feeding the overlay, set by attach().
     */
    const Telemetry* telemetry{nullptr};

    /**
     * @brief Number of firmware ticks between frames.
     */
    uint64_t ticks_per_frame{1};

    /**
     * @brief Whether the render context and scene were created.
     */
    bool context_ready{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_VIEW_VIDEO_RECORDER_HPP

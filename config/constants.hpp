/**
 * @file
 *
 * @brief Navigation constants for the ROS-simulation baseline.
 *
 * @note This is a copy of MicrasFirmware/config/constants.hpp that
 *       INTENTIONALLY diverges from it. The values below reproduce the tuning
 *       the ROS simulation was validated against, not the values flashed on the
 *       real robot, so the two files are expected to differ. Every intentional
 *       divergence is listed here and mirrored in the allowlist of
 *       tools/check_config_drift.py; anything else showing up in that script is
 *       accidental drift and must be fixed rather than allowlisted.
 *
 *       - crash_acceleration: 1e6 instead of 35, which disables crash detection
 *         altogether. The simulated chassis sees impulses the real one never
 *         does and would abort the run on contact with a wall.
 *       - speed_controller linear_pid: kp = 120, ki = 40 instead of kp = 10,
 *         ki = 1; angular_pid: kp = 16, ki = 10 instead of kp = 2, ki = 1.
 *         Same structure as the firmware, rescaled to MuJoCo command units.
 *         The PID response is summed straight onto the feed-forward command,
 *         so kp scales with the feed-forward gain of its own axis: linear
 *         102 / 12.706 = 8.0 times the firmware kp, angular 4.9 / 0.971 = 5.0
 *         times it, rounded to 120 and 16. ki is 10 instead of 1 because a
 *         180 deg pivot only lasts 1.6 s: at ki = 1 the integral had closed
 *         less than half of the 4 percent steady-state angular error by the
 *         end of the turn, and since TurnAction is open loop in time the
 *         missing area came out as a 21 deg heading error per turn. With
 *         ki = 10 the executed turn angle is 98.7 percent of the commanded
 *         one. ki is 40 on the linear axis rather than 10 because
 *         Micras::run calls speed_controller.reset() at every action boundary
 *         and an exploring action is only 45 to 180 mm long, so the integral
 *         gets 0.16 to 0.6 s to close the 5 percent steady-state speed error
 *         before being wiped. That error matters because TurnAction holds a
 *         constant linear speed for a fixed time: at -5.5 percent the exploring
 *         curve displaces 41.7 mm instead of the 45 mm the ActionQueuer
 *         geometry assumes, and MoveAction measures distance travelled rather
 *         than position in the cell, so the missing millimetres never come
 *         back. ki = 40 took the first ERROR of a 120 s explore from 22.3 s to
 *         29.0 s and the maximum odometry error from 131 to 57 mm.
 *         BEWARE: the firmware PID evaluates kp * (error + ki * integral -
 *         kd * derivative), so kp = 0 would make the whole PID output
 *         identically zero. That is what this file used to do.
 *       - action_queuer exploring max_centrifugal_acceleration: 2.0 instead of
 *         2.78, and max_linear_speed 0.3 instead of 0.4. The exploring curve
 *         radius is max_linear_speed^2 / max_centrifugal_acceleration, so the
 *         pair that used to be here (0.3 and 1.0) asked for a 90 mm radius.
 *         The corridor is 180 - 12.6 = 167.4 mm wide and the chassis is 66 mm
 *         wide with a 66 mm front overhang, so a 90 mm curve puts the outer
 *         front corner into the wall: the robot wedged for 8.4 s of a 120 s
 *         explore. 0.3^2 / 2.0 = 45 mm clears it and cuts the wedged time to
 *         1.3 s and the chassis contact fraction from 0.074 to 0.013.
 *       - left/right feed_forward linear_speed: 102 instead of 12.706/13.319.
 *         Calibrated to the MuJoCo motor, whose no-load speed is 1.01 m/s at
 *         ctrl = 100, rather than to the real motor and gearbox.
 *       - follow_wall pid saturation: 4.0 instead of 0.5, and kd 0.05 instead of
 *         0.0. FollowWall returns state.velocity.linear * pid(error), where the
 *         error is the left minus right rangefinder reading; around the centre
 *         of the corridor that error is 5.34 per metre of lateral offset, so
 *         with kp = 30 the proportional term alone reaches the old saturation
 *         of 1.0 at 6.2 mm of offset. Beyond that the lateral loop was pure
 *         bang-bang at +-v rad/s and limit-cycled: the robot crossed the centre
 *         line with the full heading error still on it, entered the next curve
 *         8 deg off and walked into the wall. Raising the saturation to 4.0
 *         keeps the loop proportional out to 25 mm, a full half corridor.
 *         The undamped loop has omega_n = v * sqrt(kp * 5.34) = 3.4 rad/s, a
 *         1.9 s period, i.e. half a metre of travel per swing; kd adds
 *         zeta = v * sqrt(kp * 5.34) * kd / 2, so 0.05 is the largest value
 *         that damps it without letting the derivative kick on the step the
 *         reading takes when a wall ends. Measured over a 120 s explore:
 *         lateral rms 25.0 -> 8.7 mm, chassis contact 0.0027 -> 0.0049, first
 *         ERROR 29.0 -> 98.2 s and the goal reached at 38.8 s instead of never.
 *       - follow_wall pid kp: 30 instead of 0.5, and post_threshold 400 instead
 *         of 16.5, because the simulated rangefinders return metres over a very
 *         different scale than the real ADC readings.
 *       - odometry linear_cutoff_frequency: 15.27 instead of 7.64, since the
 *         simulated encoders carry no noise to filter out. Both numbers are
 *         1.527 times the pre-2026-09 ones because the ButterworthFilter was
 *         normalising by f_c instead of 2 * pi * f_c and defaulting to a 100 Hz
 *         sampling rate while running at loop_frequency; every cutoff was
 *         restated as the frequency actually in effect, so nothing changed but
 *         the label.
 *       - wall_sensors filter_cutoff: 30.55 instead of 7.64, because the
 *         simulated rangefinders are noise free and the post detection in
 *         FollowWall::check_posts needs the sharpest edge available.
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <cstdint>
#include <numbers>

#include "micras/nav/action_queuer.hpp"
#include "micras/nav/follow_wall.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/odometry.hpp"
#include "micras/nav/speed_controller.hpp"

namespace micras {
/***************
 * Constants
 ***************/

constexpr bool     debug_mode{false};
constexpr uint8_t  maze_width{16};
constexpr uint8_t  maze_height{16};
constexpr float    cell_size{0.18};
constexpr uint32_t loop_time_us{1042};
constexpr float    loop_frequency{1000000.0F / loop_time_us};
constexpr float    wall_thickness{0.012F};
constexpr float    start_offset{0.05F};
constexpr float    max_angular_acceleration{400.0F};
constexpr float    crash_acceleration{1000000.0F};
constexpr float    fan_speed{100.0F};

constexpr core::WallSensorsIndex wall_sensors_index{
    .left_front = 0,
    .left = 1,
    .right = 2,
    .right_front = 3,
};

/***************
 * Template Instantiations
 ***************/

namespace nav {
using Maze = TMaze<maze_width, maze_height>;
}  // namespace nav

/***************
 * Configurations
 ***************/

const nav::ActionQueuer::Config action_queuer_config{
    .cell_size = cell_size,
    .start_offset = start_offset,
    .curve_safety_margin = 0.053F,
    .exploring =
        {
            .max_linear_speed = 0.3F,
            .max_linear_acceleration = 5.0F,
            .max_linear_deceleration = 5.0F,
            .max_centrifugal_acceleration = 2.0F,
            .max_angular_acceleration = 200.0F,
        },
    .solving =
        {
            .max_linear_speed = 5.0F,
            .max_linear_acceleration = 12.0F,
            .max_linear_deceleration = 20.0F,
            .max_centrifugal_acceleration = 40.0F,
            .max_angular_acceleration = max_angular_acceleration,
        },
};

const nav::FollowWall::Config follow_wall_config{
    .pid =
        {
            .kp = 30.0F,
            .ki = 0.0F,
            .kd = 0.05F,
            .setpoint = 0.0F,
            .saturation = 4.0F,
            .max_integral = -1.0F,
        },
    .wall_sensor_index = wall_sensors_index,
    .max_angular_acceleration = max_angular_acceleration,
    .cell_size = cell_size,
    .post_threshold = 400.0F,
    .post_reference = 0.44F * cell_size,
    .post_clearance = 0.035F,
};

const nav::Maze::Config maze_config{
    .start = {{0, 0}, nav::Side::UP},
    .goal = {{
        {maze_width / 2, maze_height / 2},
        {(maze_width - 1) / 2, maze_height / 2},
        {maze_width / 2, (maze_height - 1) / 2},
        {(maze_width - 1) / 2, (maze_height - 1) / 2},
    }},
    .cost_margin = 1.2F,
    .action_queuer_config = action_queuer_config,
};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 15.27F,
    .sampling_frequency = loop_frequency,
    .wheel_radius = 0.011F,
    .initial_pose = {{cell_size / 2.0F, start_offset}, std::numbers::pi_v<float> / 2.0F},
};

const nav::SpeedController::Config speed_controller_config{
    .linear_pid =
        {
            .kp = 120.0F,
            .ki = 40.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 40.0F,
            .max_integral = -1.0F,
        },
    .angular_pid =
        {
            .kp = 16.0F,
            .ki = 10.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 40.0F,
            .max_integral = -1.0F,
        },
    .left_feed_forward =
        {
            .linear_speed = 102.0F,
            .linear_acceleration = 0.3F,
            .angular_speed = -4.9F,
            .angular_acceleration = -0.1F,
        },
    .right_feed_forward =
        {
            .linear_speed = 102.0F,
            .linear_acceleration = 0.3F,
            .angular_speed = +4.9F,
            .angular_acceleration = +0.1F,
        },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP

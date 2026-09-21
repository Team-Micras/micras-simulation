#include <algorithm>

#include <gtest/gtest.h>

#include "micras/sim/recording/csv_recorder.hpp"
#include "micras/sim/recording/ground_truth.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Ground truth header of every run recorded so far, in order.
 *
 * @note Transcribed from baseline/v0/idle/data.csv. Changing it breaks the
 *       comparison against every run already on disk, so it is pinned here
 *       instead of being derived from the code under test.
 */
const std::vector<std::string> baseline_columns{
    "tick",
    "sim_time",
    "fw_loop_time",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "vx_world",
    "vy_world",
    "vz_world",
    "wz_body",
    "v_forward",
    "wheel_qvel_left",
    "wheel_qvel_right",
    "ctrl_left",
    "ctrl_right",
    "act_force_left",
    "act_force_right",
    "left_ncon",
    "left_fn",
    "left_slip",
    "left_penetration",
    "right_ncon",
    "right_fn",
    "right_slip",
    "right_penetration",
    "caster_ncon",
    "ncon_total",
    "solver_niter",
    "warnings_total",
    "wheel_qpos_left",
    "wheel_qpos_right",
    "caster_fn",
    "base_ncon"
};

TEST(GroundTruth, ColumnsMatchTheRecordedBaseline) {
    EXPECT_EQ(GroundTruth::columns(), baseline_columns);
}

TEST(CsvRecorder, ProxyColumnsMatchTheProxyStateSections) {
    const std::vector<std::string> expected{
        "proxy_left_command",   "proxy_right_command", "proxy_fan_speed",  "proxy_encoder_left", "proxy_encoder_right",
        "proxy_gyro_x",         "proxy_gyro_y",        "proxy_gyro_z",     "proxy_accel_x",      "proxy_accel_y",
        "proxy_accel_z",        "proxy_wall_adc_0",    "proxy_wall_adc_1", "proxy_wall_adc_2",   "proxy_wall_adc_3",
        "proxy_button_pressed", "proxy_dip_0",         "proxy_dip_1",      "proxy_dip_2",        "proxy_dip_3"
    };

    EXPECT_EQ(CsvRecorder::proxy_columns(), expected);

    const std::size_t actuators = 3;
    const std::size_t sensors = Sensors::wheel_count + (2 * Sensors::axis_count) + Sensors::wall_sensor_count;
    const std::size_t input = 1 + InterfaceInput::dip_switch_count;
    EXPECT_EQ(CsvRecorder::proxy_columns().size(), actuators + sensors + input);
}

TEST(CsvRecorder, ProxyColumnsComeAfterEveryBaselineColumn) {
    for (const std::string& column : CsvRecorder::proxy_columns()) {
        EXPECT_TRUE(column.starts_with("proxy_")) << column;
        EXPECT_EQ(std::count(baseline_columns.begin(), baseline_columns.end(), column), 0) << column;
    }
}

TEST(CsvRecorder, TurnsPoolNamesIntoTheBaselineColumnNames) {
    EXPECT_EQ(CsvRecorder::column_name("Grid Pose"), "grid_pose");
    EXPECT_EQ(CsvRecorder::column_name("Desired Linear Speed"), "desired_linear_speed");
    EXPECT_EQ(CsvRecorder::column_name("Wall Sensors 0"), "wall_sensors_0");
    EXPECT_EQ(CsvRecorder::column_name("Linear/Angular PID"), "linear_angular_pid");
}

TEST(CsvRecorder, TrimsSeparatorsAtTheEdgesOfPoolNames) {
    EXPECT_EQ(CsvRecorder::column_name("  Spaced  "), "spaced");
    EXPECT_EQ(CsvRecorder::column_name("Trailing!"), "trailing");
    EXPECT_EQ(CsvRecorder::column_name("!!!"), "");
}
}  // namespace
}  // namespace micras::sim

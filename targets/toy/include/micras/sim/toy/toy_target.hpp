/**
 * @file
 *
 * @brief The toy target: the smallest robot that exercises every feature of the engine and of Target.
 */

#ifndef MICRAS_SIM_TOY_TOY_TARGET_HPP
#define MICRAS_SIM_TOY_TOY_TARGET_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <numbers>
#include <string>
#include <vector>

#include "micras/sim/app/target.hpp"
#include "micras/sim/app/wiring.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/devices/dc_motor.hpp"
#include "micras/sim/devices/digital_input.hpp"
#include "micras/sim/devices/power.hpp"
#include "micras/sim/recording/column_source.hpp"
#include "micras/sim/recording/csv_writer.hpp"
#include "micras/sim/recording/ground_truth.hpp"
#include "micras/sim/recording/run_metadata.hpp"

namespace micras::sim {
/**
 * @brief What the toy's program does.
 */
enum class ToyState : uint8_t {
    WAIT,
    DRIVE,
    TURN,
    STOP,
};

/**
 * @brief The toy's hardware as its program sees it: what the devices wrote and what the program drives.
 *
 * @note The program and the devices never run at the same time, since the firmware thread hands over
 *       strictly, so this needs no lock.
 */
struct ToyBoard {
    /**
     * @brief Duty of each motor in percent, negative backwards.
     */
    std::array<float, 2> motor_duty{};

    /**
     * @brief Whether the motor driver is enabled.
     */
    bool motors_enabled{false};

    /**
     * @brief Duty of the fan in percent.
     */
    float fan_duty{0.0F};

    /**
     * @brief Whether the wall sensors' emitters are on.
     */
    bool emitters_on{false};

    /**
     * @brief Count of each wheel's encoder.
     */
    std::array<int32_t, 2> encoder_counts{};

    /**
     * @brief Last yaw rate the IMU delivered, in raw words.
     */
    float gyro_z_word{0.0F};

    /**
     * @brief Wall sensor readings: each receiver with the first group lit, then with the second.
     */
    std::array<uint32_t, 4> wall_counts{};

    /**
     * @brief Battery divider reading.
     */
    uint32_t battery_counts{0};

    /**
     * @brief Motor current readings.
     */
    std::array<uint32_t, 2> current_counts{};

    /**
     * @brief Level of the button's pin, high when pressed.
     */
    bool button_level{false};

    /**
     * @brief Bytes the link delivered, not read yet.
     */
    std::deque<uint8_t> received;

    /**
     * @brief Bytes the program sent, not on the link yet.
     */
    std::deque<uint8_t> to_send;
};

/**
 * @brief What the toy's program knows, recorded as columns and read by the panel and the scenarios.
 */
struct ToyReadings {
    /**
     * @brief State of the program.
     */
    ToyState state{ToyState::WAIT};

    /**
     * @brief Distance the wheels rolled, in meters.
     */
    double odometry{0.0};

    /**
     * @brief Heading integrated from the gyro, in radians.
     */
    double heading{0.0};

    /**
     * @brief Front wall sensor reading, with its own emitter lit, in counts.
     */
    double front{0.0};

    /**
     * @brief Pack voltage, in volts.
     */
    double voltage{0.0};

    /**
     * @brief Current of each motor, in amperes.
     */
    std::array<double, 2> currents{};
};

/**
 * @brief The toy's variables: CSV columns and what scenarios and the overlay read.
 */
class ToyVariables : public ColumnSource, public VariableSource {
public:
    /**
     * @brief Read from the program's readings.
     *
     * @param readings The readings, which outlive the variables.
     */
    explicit ToyVariables(const ToyReadings& readings);

    /**
     * @brief Get the names of the columns.
     *
     * @return The names, in the order append() writes them.
     */
    std::vector<std::string> names() override;

    /**
     * @brief Append the current readings to a row.
     *
     * @param row Row to append to.
     */
    void append(std::vector<CsvCell>& row) override;

    /**
     * @brief Get a variable by name.
     *
     * @param name Name of a column.
     * @return Its value.
     */
    double value_of(const std::string& name) const override;

private:
    /**
     * @brief The program's readings.
     */
    const ToyReadings& readings;  // NOLINT(*-avoid-const-or-ref-data-members): the target owns both.
};

/**
 * @brief The toy robot: drives until a wall, turns, drives on, and stops when told.
 *
 * @note No HAL, nothing of a real robot: the program runs on the firmware thread and yields a tick
 *       itself once per loop, and the devices write straight into a ToyBoard.
 */
class ToyTarget : public Target {
public:
    /**
     * @brief Get the name of the robot.
     *
     * @return "toy", which its scenarios name.
     */
    std::string name() const override;

    /**
     * @brief Get the firmware commit.
     *
     * @return "none": the toy's program is part of the simulator.
     */
    std::string firmware_sha() const override;

    /**
     * @brief Get the loop period of the program.
     *
     * @return 1000 us.
     */
    uint32_t loop_time_us() const override;

    /**
     * @brief Get the toy's own option, --turn-angle.
     *
     * @return The options.
     */
    std::vector<CliOption> options() override;

    /**
     * @brief Get the context of the run.
     *
     * @return This target's context.
     */
    RunContext& context() override;

    /**
     * @brief Get the folder of this target.
     *
     * @return Path of targets/toy.
     */
    std::filesystem::path directory() const override;

    /**
     * @brief Get the robot's physical description.
     *
     * @return Path of targets/toy/robot.toml.
     */
    std::filesystem::path robot_file() const override;

    /**
     * @brief Get the ground truth columns of a toy run.
     *
     * @return The recorder configuration.
     */
    GroundTruthConfig ground_truth() const override;

    /**
     * @brief Get the camera of a video.
     *
     * @return The camera above the robot.
     */
    std::string video_camera() const override;

    /**
     * @brief Get the program, which yields on the thread it is given once per loop.
     *
     * @return The program.
     */
    FirmwareThread::Program program() override;

    /**
     * @brief Build the toy's devices, columns, panel and scenario hooks.
     *
     * @param firmware Thread that runs the program.
     * @param world The robot and the arena's surfaces.
     * @return What the toy adds to the run.
     */
    Wiring wire(FirmwareThread& firmware, const WorldInfo& world) override;

    /**
     * @brief Get the toy's counters for meta.json.
     *
     * @return The bytes it sent over the link.
     */
    std::vector<MetadataField> metadata() const override;

private:
    /**
     * @brief Run the program: one loop per tick, forever.
     *
     * @param thread Thread the program runs on, which it yields.
     */
    void run(FirmwareThread& thread);

    /**
     * @brief Update the readings from what the devices wrote.
     *
     * @param dt Time since the last loop, in seconds.
     */
    void read_sensors(double dt);

    /**
     * @brief Take the bytes that arrived over the link.
     *
     * @return "go" or "stop" when the bytes just completed one, else empty.
     */
    std::string read_link();

    /**
     * @brief Send text over the link.
     *
     * @param text Bytes to send.
     */
    void send(const std::string& text);

    /**
     * @brief Drive the motors, the fan and the emitters for the current state.
     */
    void actuate();

    /**
     * @brief Build the panel.
     *
     * @return The button, a lamp per state and the pack voltage.
     */
    PanelSpec make_panel() const;

    /**
     * @brief Build the scenario hooks.
     *
     * @return The button, the go and stop messages and the state names.
     */
    ScenarioHooks make_hooks() const;

    /**
     * @brief The run's context.
     */
    RunContext run_context;

    /**
     * @brief The toy's hardware.
     */
    ToyBoard board;

    /**
     * @brief What the program knows.
     */
    ToyReadings readings;

    /**
     * @brief The recorded variables.
     */
    ToyVariables variables{readings};

    /**
     * @brief The button's input device.
     */
    DigitalInput* button{nullptr};

    /**
     * @brief The battery, for the panel.
     */
    const Battery* battery{nullptr};

    /**
     * @brief Heading change the program turns at a wall, in radians, positive to the right.
     */
    double turn_angle{std::numbers::pi / 2.0};

    /**
     * @brief Heading when the current turn started, in radians.
     */
    double turn_start{0.0};

    /**
     * @brief Radius of the wheels, from the robot's description.
     */
    double wheel_radius{0.0};

    /**
     * @brief Encoder counts per wheel revolution, from the robot's description.
     */
    double counts_per_revolution{1.0};

    /**
     * @brief Gyro resolution, from the robot's description, in rad/s per word.
     */
    double gyro_resolution{0.0};

    /**
     * @brief Divider of the battery reading.
     */
    double battery_divider{3.0};

    /**
     * @brief Bytes of the last command, to recognize "go" and "stop" in the raw byte stream.
     */
    std::string command;

    /**
     * @brief Whether the button was pressed on the previous loop, to start on the press.
     */
    bool was_pressed{false};

    /**
     * @brief Bytes the program sent over the link.
     */
    int64_t bytes_sent{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_TOY_TOY_TARGET_HPP

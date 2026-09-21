#include <gtest/gtest.h>

#include "micras/proxy/bluetooth_serial.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "micras/proxy/wall_sensors.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
namespace {
/**
 * @brief A simulation of its own, as Appendix A of the plan requires of tests.
 *
 * @note Nothing here touches sim::SimulationContext::instance(): the proxies
 *       are handed this context through their Config, exactly as target.hpp
 *       hands them the process-wide one.
 *
 * @note SetUp advances the dynamics by one tick before any test runs, because
 *       the accelerometer and every contact derived sensor only carry a valid
 *       reading once they have, exactly as in a run.
 */
/**
 * @brief Ticks the robot needs to fall the spawn gap and land on its wheels.
 *
 * @note Until it lands the accelerometer reads free fall, so a test that wants
 *       to see gravity has to wait out the drop first.
 */
constexpr int ticks_until_the_wheels_land{200};

class ProxyBoundary : public testing::Test {
protected:
    void SetUp() override {
        this->context.world.load(MICRAS_TEST_MODEL);
        this->context.clock = sim::Clock::from_model(this->context.world.timestep(), 1042);
        this->context.world.reset();

        this->context.world.step(this->context.clock.steps_per_tick());
    }

    // NOLINTNEXTLINE(*-non-private-member-variables-in-classes): the fixture is the test's own scope.
    sim::SimulationContext context;
};

TEST_F(ProxyBoundary, ProxiesRefuseToBuildWithoutAContext) {
    EXPECT_THROW(Imu({.gyro_sensor = "gyro", .accelerometer_sensor = "accelerometer"}), std::logic_error);
    EXPECT_THROW(Motor({.actuator = "motor_left"}), std::logic_error);
}

TEST_F(ProxyBoundary, ProxiesRefuseToBuildWhenTheModelLacksTheObject) {
    EXPECT_THROW(Motor({.actuator = "no_such_actuator", .context = &this->context}), std::runtime_error);
}

TEST_F(ProxyBoundary, TheImuPublishesItsReadings) {
    Imu imu({.gyro_sensor = "gyro", .accelerometer_sensor = "accelerometer", .context = &this->context});

    for (int tick = 0; tick < ticks_until_the_wheels_land; tick++) {
        this->context.world.step(this->context.clock.steps_per_tick());
    }

    imu.update();

    const float published_rate = this->context.proxy_state.sensors.angular_velocity.at(2);
    const float published_acceleration = this->context.proxy_state.sensors.linear_acceleration.at(2);

    EXPECT_FLOAT_EQ(published_rate, imu.get_angular_velocity(Imu::Axis::Z));
    EXPECT_FLOAT_EQ(published_acceleration, imu.get_linear_acceleration(Imu::Axis::Z));
    EXPECT_NEAR(published_acceleration, 9.81F, 0.05F);
}

TEST_F(ProxyBoundary, EachEncoderPublishesIntoItsOwnWheel) {
    const RotarySensor left({.sensor = "encoder_left", .wheel = 0, .context = &this->context});
    const RotarySensor right({.sensor = "encoder_right", .wheel = 1, .context = &this->context});

    const float left_position = left.get_position();
    const float right_position = right.get_position();

    EXPECT_FLOAT_EQ(this->context.proxy_state.sensors.encoder_positions.at(0), left_position);
    EXPECT_FLOAT_EQ(this->context.proxy_state.sensors.encoder_positions.at(1), right_position);
}

TEST_F(ProxyBoundary, TheWallSensorsPublishTheirUnfilteredReadings) {
    const TWallSensors<4> sensors({
        .sensors = {"lidar_0", "lidar_1", "lidar_2", "lidar_3"},
        .uncertainty = 0.5F,
        .base_readings = {0.0696F, 0.1090F, 0.1090F, 0.0696F},
        .max_sensor_reading = 0.6F,
        .min_sensor_reading = 0.01F,
        .max_sensor_distance = 0.36F,
        .filter_cutoff = 30.55F,
        .sampling_frequency = 959.69F,
        .context = &this->context,
    });

    for (uint8_t i = 0; i < 4; i++) {
        const float reading = sensors.get_adc_reading(i);

        EXPECT_GT(reading, 0.0F) << +i;
        EXPECT_FLOAT_EQ(this->context.proxy_state.sensors.wall_adc_readings.at(i), reading) << +i;
    }
}

TEST_F(ProxyBoundary, LocomotionPublishesTheCommandsItPassesOn) {
    Locomotion locomotion({
        .left_motor = {.actuator = "motor_left", .context = &this->context},
        .right_motor = {.actuator = "motor_right", .context = &this->context},
        .context = &this->context,
    });

    locomotion.enable();
    locomotion.set_wheel_command(30.0F, -20.0F);

    EXPECT_FLOAT_EQ(this->context.proxy_state.actuators.left_command, 30.0F);
    EXPECT_FLOAT_EQ(this->context.proxy_state.actuators.right_command, -20.0F);

    locomotion.disable();
    locomotion.set_wheel_command(30.0F, -20.0F);

    EXPECT_FLOAT_EQ(this->context.proxy_state.actuators.left_command, 0.0F);
    EXPECT_FLOAT_EQ(this->context.proxy_state.actuators.right_command, 0.0F);
}

TEST_F(ProxyBoundary, TheFanHonoursTheHarnessOverride) {
    Fan fan({.actuator = "fan", .max_acceleration = 100.0F, .context = &this->context});

    fan.set_speed(50.0F);
    this->context.clock.advance();
    fan.update();
    EXPECT_GT(this->context.proxy_state.actuators.fan_speed, 0.0F);

    this->context.proxy_state.overrides.fan_enabled = false;
    fan.update();
    EXPECT_FLOAT_EQ(this->context.proxy_state.actuators.fan_speed, 0.0F);
}

TEST_F(ProxyBoundary, TheInterfaceProxiesReadTheStateTheScenarioWrites) {
    const Button        button({.name = "button", .context = &this->context});
    const TDipSwitch<4> switches({.names = {"a", "b", "c", "d"}, .context = &this->context});

    this->context.proxy_state.interface_input.button_pressed = true;
    this->context.proxy_state.interface_input.dip_switches = {true, false, true, false};

    EXPECT_FALSE(button.is_pressed());
    EXPECT_TRUE(switches.get_switch_state(0));
    EXPECT_FALSE(switches.get_switch_state(1));
    EXPECT_EQ(switches.get_switches_value(), 0b0101);
}

TEST_F(ProxyBoundary, AFreshStopwatchReadsTheNominalPeriod) {
    const Stopwatch stopwatch({.context = &this->context});

    EXPECT_EQ(stopwatch.elapsed_time_us(), 1042U);
}

TEST_F(ProxyBoundary, TheStopwatchRunsOnTheContextClock) {
    Stopwatch stopwatch({.context = &this->context});

    stopwatch.reset_us();
    this->context.clock.advance();
    this->context.clock.advance();

    EXPECT_EQ(stopwatch.elapsed_time_ms(), 2U * 1042U / 1000U);
    EXPECT_EQ(stopwatch.elapsed_time_us(), 2U * 1042U);

    stopwatch.reset_ms();
    EXPECT_EQ(stopwatch.elapsed_time_ms(), 0U);
}

TEST_F(ProxyBoundary, TheStopwatchDoesNotYieldWithoutAProgram) {
    const Stopwatch stopwatch({.context = &this->context});

    EXPECT_EQ(this->context.firmware.load(), nullptr);
    EXPECT_NO_THROW(static_cast<void>(stopwatch.elapsed_time_us()));
}

TEST_F(ProxyBoundary, TheRadioCarriesBytesBothWays) {
    BluetoothSerial radio({.context = &this->context});

    this->context.serial.queue_for_firmware(std::vector<uint8_t>{1, 2, 3});
    EXPECT_EQ(radio.get_data(), (std::vector<uint8_t>{1, 2, 3}));
    EXPECT_TRUE(radio.get_data().empty());
}
}  // namespace
}  // namespace micras::proxy

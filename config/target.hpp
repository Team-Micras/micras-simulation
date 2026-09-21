/**
 * @file target.hpp
 *
 * @brief Target specific configuration for the MuJoCo harness
 *
 * @note Every name below is resolved against the loaded MuJoCo model when the
 *       proxy is constructed. This is the one file that names
 *       sim::SimulationContext::instance(): every proxy receives the facet it
 *       needs through its Config, so nothing else has to know the context is
 *       process-wide.
 *
 * @date 03/2024
 */

#ifndef MICRAS_TARGET_HPP
#define MICRAS_TARGET_HPP

#include "constants.hpp"
#include "micras/proxy/argb.hpp"
#include "micras/proxy/battery.hpp"
#include "micras/proxy/bluetooth_serial.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/led.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/storage.hpp"
#include "micras/proxy/torque_sensors.hpp"
#include "micras/proxy/wall_sensors.hpp"
#include "micras/sim/core/simulation_context.hpp"

// clang-format off
namespace micras {
/*****************************************
 * Template Instantiations
 *****************************************/

namespace proxy {
    using Argb = proxy::TArgb<2>;
    using DipSwitch = TDipSwitch<4>;
    using TorqueSensors = TTorqueSensors<2>;
    using WallSensors = TWallSensors<4>;
}  // namespace proxy

const proxy::Stopwatch::Config stopwatch_config {
    .context = &sim::SimulationContext::instance()
};

const proxy::Storage::Config maze_storage_config {
    .start_page = 2,
    .number_of_pages = 1
};

/*****************************************
 * Interface
 *****************************************/

const proxy::Led::Config led_config {
    .name = "led",
    .context = &sim::SimulationContext::instance()
};

const proxy::Argb::Config argb_config {
    .names = {
        "rgb_0",
        "rgb_1"
    },
    .context = &sim::SimulationContext::instance()
};

const proxy::Button::Config button_config {
    .name = "button",
    .long_press_delay = 500U,
    .extra_long_press_delay = 2000U,
    .context = &sim::SimulationContext::instance()
};

const proxy::DipSwitch::Config dip_switch_config {
    .names = {
        "dip_switch_0",
        "dip_switch_1",
        "dip_switch_2",
        "dip_switch_3"
    },
    .context = &sim::SimulationContext::instance()
};

const proxy::Buzzer::Config buzzer_config {
    .name = "buzzer",
    .context = &sim::SimulationContext::instance()
};

const proxy::BluetoothSerial::Config bluetooth_config {
    .context = &sim::SimulationContext::instance()
};

/*****************************************
 * Sensors
 *****************************************/

const proxy::RotarySensor::Config rotary_sensor_left_config {
    .sensor = "encoder_left",
    .wheel = 0,
    .context = &sim::SimulationContext::instance()
};

const proxy::RotarySensor::Config rotary_sensor_right_config {
    .sensor = "encoder_right",
    .wheel = 1,
    .context = &sim::SimulationContext::instance()
};

const proxy::TorqueSensors::Config torque_sensors_config {
    .shunt_resistor = 0.04F * 20,
    .max_torque = 3.0F,
    .reference_voltage = 3.3F
};

const proxy::WallSensors::Config wall_sensors_config {
    .sensors = {
        "lidar_0",
        "lidar_1",
        "lidar_2",
        "lidar_3"
    },
    .uncertainty = 0.5F,
    .base_readings = {
        0.0696F,
        0.1090F,
        0.1090F,
        0.0696F,
    },
    .max_sensor_reading = 0.6F,
    .min_sensor_reading = 0.01F,
    .max_sensor_distance = 0.18F * 2,
    .filter_cutoff = 30.55F,
    .sampling_frequency = loop_frequency,
    .context = &sim::SimulationContext::instance()
};

const proxy::Imu::Config imu_config {
    .gyro_sensor = "gyro",
    .accelerometer_sensor = "accelerometer",
    .context = &sim::SimulationContext::instance()
};

const proxy::Battery::Config battery_config {
    .max_voltage = 9.9F,
    .max_reading = 4095
};

/*****************************************
 * Actuators
 *****************************************/

const proxy::Fan::Config fan_config {
    .actuator = "fan",
    .max_acceleration = 0.02F,
    .context = &sim::SimulationContext::instance()
};

const proxy::Locomotion::Config locomotion_config {
    .left_motor = {
        .actuator = "motor_left",
        .context = &sim::SimulationContext::instance()
    },
    .right_motor = {
        .actuator = "motor_right",
        .context = &sim::SimulationContext::instance()
    },
    .context = &sim::SimulationContext::instance()
};
}  // namespace micras

// clang-format on

#endif  // MICRAS_TARGET_HPP

/**
 * @file
 *
 * @brief Compile-time conformance of the shadow proxy configs with the firmware ones.
 *
 * @note Every Config below is initialised with the designators the firmware
 *       target.hpp uses for the fields both sides share. Renaming a field in
 *       the firmware (start_page -> start_sector, say) breaks this translation
 *       unit instead of silently drifting. Hardware-only fields (pwm, gpio,
 *       spi, adc) have no shadow counterpart and are not listed.
 */

#include <gtest/gtest.h>

#include "target.hpp"

namespace micras {
namespace {
const proxy::Storage::Config storage_shape{.start_page = 0, .number_of_pages = 0};

const proxy::Button::Config button_shape{.long_press_delay = 0, .extra_long_press_delay = 0};

const proxy::Fan::Config fan_shape{.max_acceleration = 0.0F};

const proxy::Locomotion::Config locomotion_shape{.left_motor = {}, .right_motor = {}};

const proxy::TorqueSensors::Config torque_sensors_shape{.shunt_resistor = 0.0F, .max_torque = 0.0F};

const proxy::WallSensors::Config wall_sensors_shape{
    .uncertainty = 0.0F, .base_readings = {}, .filter_cutoff = 0.0F, .sampling_frequency = 0.0F
};

TEST(ShadowConformance, SharedConfigFieldsCompile) {
    EXPECT_EQ(storage_shape.number_of_pages, 0);
    EXPECT_EQ(button_shape.long_press_delay, 0);
    EXPECT_FLOAT_EQ(fan_shape.max_acceleration, 0.0F);
    EXPECT_TRUE(locomotion_shape.left_motor.actuator.empty());
    EXPECT_FLOAT_EQ(torque_sensors_shape.max_torque, 0.0F);
    EXPECT_FLOAT_EQ(wall_sensors_shape.filter_cutoff, 0.0F);
}

TEST(ShadowConformance, TargetConfigsUseFirmwareDelays) {
    EXPECT_EQ(button_config.long_press_delay, 500U);
    EXPECT_EQ(button_config.extra_long_press_delay, 2000U);
    EXPECT_FLOAT_EQ(wall_sensors_config.sampling_frequency, loop_frequency);
}
}  // namespace
}  // namespace micras

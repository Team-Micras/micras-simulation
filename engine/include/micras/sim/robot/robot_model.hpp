/**
 * @file
 *
 * @brief The MuJoCo model of a robot, generated from its description.
 */

#ifndef MICRAS_SIM_ROBOT_ROBOT_MODEL_HPP
#define MICRAS_SIM_ROBOT_ROBOT_MODEL_HPP

#include <string>

#include "micras/sim/robot/robot_description.hpp"

namespace micras::sim {
/**
 * @brief Names the generated model gives to what devices and recorders look up.
 *
 * @note One place for every name, so a device and the generator cannot disagree.
 */
struct RobotModelNames {
    /**
     * @brief The robot's body and free joint, the robot's own name.
     */
    std::string body;

    ///@{
    std::string left_wheel{"left_wheel"};
    std::string right_wheel{"right_wheel"};
    std::string left_motor{"motor_left"};
    std::string right_motor{"motor_right"};
    std::string board{"board"};
    std::string fan{"fan"};
    std::string fan_reference{"fan_reference"};
    std::string imu{"imu"};
    std::string gyro{"imu_gyro"};
    std::string accelerometer{"imu_accelerometer"};

    ///@}

    /**
     * @brief Get the names a description's model uses.
     *
     * @param robot The description.
     * @return The names.
     */
    static RobotModelNames of(const RobotDescription& robot) { return {.body = robot.name}; }
};

/**
 * @brief Generate the MJCF of a robot, standing at the origin.
 *
 * @note The motors are general actuators whose control is the voltage across
 *       the winding: the gain is the stall torque per volt at the wheel and the
 *       velocity bias is the back-EMF damping at the wheel, so MuJoCo integrates
 *       the electrical part implicitly. The rotor inertia reflected through the
 *       gears is the wheel joint's armature, and the brushes' friction, the
 *       no-load current times the torque constant, its friction loss. The fan's
 *       downforce is a force actuator pulling straight down, its reference a
 *       site fixed in the world, so the board tilted onto its nose adds no
 *       backward pull that the frictionless skids could not hold.
 *
 * @param robot The description.
 * @return The MJCF text.
 */
std::string robot_mjcf(const RobotDescription& robot);
}  // namespace micras::sim

#endif  // MICRAS_SIM_ROBOT_ROBOT_MODEL_HPP

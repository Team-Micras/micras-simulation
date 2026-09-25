/**
 * @file
 *
 * @brief An H-bridge driving a brushed DC motor through a gearbox.
 */

#ifndef MICRAS_SIM_DEVICES_DC_MOTOR_HPP
#define MICRAS_SIM_DEVICES_DC_MOTOR_HPP

#include <functional>
#include <string>
#include <vector>

#include "micras/sim/devices/device.hpp"
#include "micras/sim/robot/robot_description.hpp"

namespace micras::sim {
/**
 * @brief Turns the bridge's two PWM inputs and its enable into the winding voltage.
 *
 * @note The motor itself is the robot model's actuator, whose control is the
 *       winding voltage (see robot_mjcf()). The bridge applies the duty cycle
 *       difference times the supply: with slow decay both inputs low short the
 *       winding, which brakes. A disabled bridge leaves the winding open, so no
 *       current flows, which this device expresses by applying exactly the
 *       back-EMF.
 */
class DcMotor : public Device {
public:
    /**
     * @brief What the motor is and what drives it.
     */
    struct Config {
        /**
         * @brief Prefix of the recorded columns.
         */
        std::string name;

        /**
         * @brief Actuator and wheel joint of the robot model.
         */
        ///@{
        std::string actuator;
        std::string joint;
        ///@}

        /**
         * @brief Motor, gearbox and supply.
         */
        DriveDescription drive;

        /**
         * @brief Duty cycles of the bridge's inputs, in percent.
         */
        ///@{
        std::function<float()> forward_duty;
        std::function<float()> backward_duty;
        ///@}

        /**
         * @brief Level of the bridge's enable input.
         */
        std::function<bool()> enabled;
    };

    /**
     * @brief Find the actuator and the joint in the model.
     *
     * @param world Loaded world.
     * @param config The motor.
     */
    DcMotor(const MujocoWorld& world, Config config);

    /**
     * @brief Apply the winding voltage.
     *
     * @param world World about to be advanced.
     * @param clock Clock of the run.
     */
    void actuate(MujocoWorld& world, const Clock& clock) override;

    /**
     * @brief Get the recorded columns: the winding voltage and current.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const override;

    /**
     * @brief Append the voltage and current applied this tick.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) const override;

    /**
     * @brief Get the winding current of this tick.
     *
     * @return Current in amperes, positive driving forward.
     */
    double current() const { return this->winding_current; }

private:
    Config config;
    int    actuator_id;
    int    velocity_address;
    double voltage{0.0};
    double winding_current{0.0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_DC_MOTOR_HPP

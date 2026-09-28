/**
 * @file
 */

#include <algorithm>
#include <span>
#include <string>
#include <utility>

#include "micras/sim/devices/dc_motor.hpp"

namespace micras::sim {
DcMotor::DcMotor(const MujocoWorld& world, Config config) :
    config{std::move(config)},
    actuator_id{world.require_id(mjOBJ_ACTUATOR, this->config.actuator)},
    velocity_address{world.joint_dof(this->config.joint)} { }

void DcMotor::actuate(MujocoWorld& world, const Clock& /*clock*/) {
    const DriveDescription&       drive = this->config.drive;
    const std::span<const mjtNum> velocities(world.data()->qvel, static_cast<std::size_t>(world.model()->nv));
    const double                  back_emf =
        drive.speed_constant * drive.gear_ratio * velocities[static_cast<std::size_t>(this->velocity_address)];

    if (this->config.enabled()) {
        const double duty = static_cast<double>(this->config.forward_duty() - this->config.backward_duty()) / 100.0;
        this->voltage = std::clamp(duty, -1.0, 1.0) * drive.supply_voltage;
    } else {
        this->voltage = back_emf;
    }

    this->winding_current = (this->voltage - back_emf) / drive.resistance();
    world.set_control(this->actuator_id, this->voltage);
}

std::vector<std::string> DcMotor::columns() const {
    return {this->config.name + "_voltage", this->config.name + "_current"};
}

void DcMotor::append(std::vector<CsvCell>& row) const {
    row.emplace_back(this->voltage);
    row.emplace_back(this->winding_current);
}
}  // namespace micras::sim

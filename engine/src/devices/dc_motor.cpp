/**
 * @file
 */

#include <algorithm>
#include <span>
#include <string>
#include <utility>

#include "micras/sim/devices/dc_motor.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Find where a joint's first velocity dof sits in qvel.
 *
 * @param world The world.
 * @param joint Name of the joint.
 * @return The joint's dof address.
 */
int dof_address(const MujocoWorld& world, const std::string& joint) {
    const std::span<const int> addresses(world.model()->jnt_dofadr, static_cast<std::size_t>(world.model()->njnt));
    return addresses[static_cast<std::size_t>(world.require_id(mjOBJ_JOINT, joint))];
}
}  // namespace

DcMotor::DcMotor(const MujocoWorld& world, Config config) :
    config{std::move(config)},
    actuator_id{world.require_id(mjOBJ_ACTUATOR, this->config.actuator)},
    velocity_address{dof_address(world, this->config.joint)} { }

void DcMotor::actuate(MujocoWorld& world, const Clock& /*clock*/) {
    const DriveDescription&       drive = this->config.drive;
    const std::span<const mjtNum> velocities(world.data()->qvel, static_cast<std::size_t>(world.model()->nv));
    const double                  back_emf =
        drive.speed_constant * drive.gear_ratio * velocities[static_cast<std::size_t>(this->velocity_address)];

    if (this->config.enabled()) {
        const double duty = (this->config.forward_duty() - this->config.backward_duty()) / 100.0;
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

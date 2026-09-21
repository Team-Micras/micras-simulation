/**
 * @file
 *
 * @brief Motor proxy writing a clamped percentage command to a MuJoCo actuator.
 */

#include <algorithm>

#include "micras/proxy/motor.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
Motor::Motor(const Config& config) :
    world{sim::require_context(config.context).world},
    actuator_id{this->world.require_id(mjOBJ_ACTUATOR, config.actuator)} {
    this->set_command(0.0F);
}

void Motor::set_command(float command) {
    this->world.set_control(this->actuator_id, std::clamp(command, -100.0F, 100.0F));
}
}  // namespace micras::proxy

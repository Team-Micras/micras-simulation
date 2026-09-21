/**
 * @file
 *
 * @brief Locomotion proxy driving the two MuJoCo motor actuators.
 */

#include <cmath>

#include "micras/proxy/locomotion.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
Locomotion::Locomotion(const Config& config) :
    state{sim::require_context(config.context).proxy_state},
    left_motor{config.left_motor},
    right_motor{config.right_motor} {
    this->stop();
    this->disable();
}

void Locomotion::enable() {
    this->enabled = true;
}

void Locomotion::disable() {
    this->enabled = false;
}

void Locomotion::set_wheel_command(float left_command, float right_command) {
    const float left = this->enabled ? left_command : 0.0F;
    const float right = this->enabled ? right_command : 0.0F;

    this->state.actuators.left_command = left;
    this->state.actuators.right_command = right;

    this->left_motor.set_command(left);
    this->right_motor.set_command(right);
}

void Locomotion::set_command(float linear, float angular) {
    float left_command = linear - angular;
    float right_command = linear + angular;

    if (std::abs(left_command) > 100.0F) {
        left_command *= 100.0F / std::abs(left_command);
        right_command *= 100.0F / std::abs(left_command);
    }

    if (std::abs(right_command) > 100.0F) {
        left_command *= 100.0F / std::abs(right_command);
        right_command *= 100.0F / std::abs(right_command);
    }

    this->set_wheel_command(left_command, right_command);
}

void Locomotion::stop() {
    this->set_wheel_command(0.0F, 0.0F);
}
}  // namespace micras::proxy

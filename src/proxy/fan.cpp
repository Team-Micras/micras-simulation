/**
 * @file
 *
 * @brief Fan proxy mirroring the firmware speed ramp on a MuJoCo actuator.
 */

#include "micras/core/utils.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::proxy {
Fan::Fan(const Config& config) :
    world{sim::require_context(config.context).world},
    state{sim::require_context(config.context).proxy_state},
    actuator_id{this->world.require_id(mjOBJ_ACTUATOR, config.actuator)},
    max_acceleration{config.max_acceleration},
    acceleration_stopwatch{Stopwatch::Config{.context = config.context}} {
    this->stop();
    this->enable();
}

void Fan::enable() {
    this->enabled = true;
    this->apply();
}

void Fan::disable() {
    this->enabled = false;
    this->apply();
}

void Fan::set_speed(float speed) {
    this->update();
    this->target_speed = speed;
}

float Fan::update() {
    this->current_speed = core::move_towards<float>(
        this->current_speed, this->target_speed, this->acceleration_stopwatch.elapsed_time_ms() * this->max_acceleration
    );

    this->acceleration_stopwatch.reset_ms();

    if (this->current_speed == 0.0F) {
        this->stop();
    } else {
        this->apply();
    }

    return this->current_speed;
}

void Fan::stop() {
    this->state.actuators.fan_speed = 0.0F;
    this->world.set_control(this->actuator_id, 0.0);
}

void Fan::apply() {
    const bool  active = this->enabled and this->state.overrides.fan_enabled;
    const float speed = active ? this->current_speed : 0.0F;

    this->state.actuators.fan_speed = speed;
    this->world.set_control(this->actuator_id, speed);
}
}  // namespace micras::proxy

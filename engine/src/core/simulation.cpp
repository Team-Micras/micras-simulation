/**
 * @file
 */

#include <cstdint>

#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Stops the program when the run leaves, however it leaves.
 *
 * @note Without it, a listener that throws would leave a thread parked on the
 *       handoff with nobody left to wake it.
 */
class FinishGuard {
public:
    explicit FinishGuard(FirmwareThread& firmware) : firmware{firmware} { }

    FinishGuard(const FinishGuard&) = delete;
    FinishGuard(FinishGuard&&) = delete;
    FinishGuard& operator=(const FinishGuard&) = delete;
    FinishGuard& operator=(FinishGuard&&) = delete;

    ~FinishGuard() { this->firmware.finish(); }

private:
    FirmwareThread& firmware;  // NOLINT(*-avoid-const-or-ref-data-members): scoped to one run() call.
};
}  // namespace

void IRunListener::on_start(const Simulation& /*simulation*/) { }

RunControl IRunListener::on_before_tick(const Simulation& /*simulation*/) {
    return RunControl::RUN;
}

void IRunListener::on_after_tick(const Simulation& /*simulation*/) { }

void IRunListener::on_finish(const Simulation& /*simulation*/) { }

Simulation::Simulation(RunContext& context, FirmwareThread& firmware) : simulation{context}, firmware{firmware} { }

void Simulation::add_listener(IRunListener& listener) {
    this->listeners.push_back(&listener);
}

void Simulation::run(uint64_t ticks) {
    const FinishGuard guard{this->firmware};

    for (IRunListener* listener : this->listeners) {
        listener->on_start(*this);
    }

    for (this->current_tick = 0; this->current_tick < ticks; this->current_tick++) {
        if (this->firmware.has_finished() or this->before_tick() == RunControl::QUIT) {
            break;
        }

        this->firmware.run_until_yield();

        if (this->firmware.has_finished()) {
            break;
        }

        this->step();
        this->completed++;

        for (IRunListener* listener : this->listeners) {
            listener->on_after_tick(*this);
        }
    }

    for (IRunListener* listener : this->listeners) {
        listener->on_finish(*this);
    }

    this->firmware.rethrow_any_error();
}

RunControl Simulation::before_tick() {
    RunControl control = RunControl::RUN;

    for (IRunListener* listener : this->listeners) {
        if (listener->on_before_tick(*this) == RunControl::QUIT) {
            control = RunControl::QUIT;
        }
    }

    return control;
}

void Simulation::step() {
    for (const auto& device : this->simulation.devices) {
        device->actuate(this->simulation.world, this->simulation.clock);
    }

    this->simulation.world.step(this->simulation.clock.steps_per_tick());
    this->simulation.clock.advance();

    for (const auto& device : this->simulation.devices) {
        device->sample(this->simulation.world, this->simulation.clock);
    }
}
}  // namespace micras::sim

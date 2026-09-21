/**
 * @file
 */

#include <stdexcept>

#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/simulation_context.hpp"

namespace micras::sim {
SimulationContext& SimulationContext::instance() {
    static SimulationContext context;
    return context;
}

SimulationContext& require_context(SimulationContext* context) {
    if (context == nullptr) {
        throw std::logic_error("a proxy was configured without a simulation context");
    }

    return *context;
}

void yield_tick() {
    FirmwareThread* firmware = SimulationContext::instance().firmware;

    if (firmware != nullptr) {
        firmware->yield_tick();
    }
}
}  // namespace micras::sim

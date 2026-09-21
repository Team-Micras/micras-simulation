/**
 * @file
 *
 * @brief Everything the proxy layer sees of the simulation.
 */

#ifndef MICRAS_SIM_CORE_SIMULATION_CONTEXT_HPP
#define MICRAS_SIM_CORE_SIMULATION_CONTEXT_HPP

#include <atomic>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/proxy_state.hpp"
#include "micras/sim/core/serial_bus.hpp"

namespace micras::sim {
class FirmwareThread;

/**
 * @brief Aggregate of the facets a proxy can reach, with no logic of its own.
 *
 * @note The firmware initialises its proxies from the const globals in
 *       config/target.hpp, before main runs, so the only route from a proxy to
 *       the simulation is something reachable from its Config. instance() is
 *       that route and is referenced by target.hpp alone: every proxy and every
 *       class under sim/ receives the facet it needs instead of calling it.
 *       Being a function-local static, it is constructed on the first call and
 *       therefore never after something that depends on it.
 */
struct SimulationContext {
    /**
     * @brief Get the process-wide context.
     *
     * @return Reference to the single instance.
     */
    static SimulationContext& instance();

    /**
     * @brief MuJoCo model and data.
     */
    MujocoWorld world;

    /**
     * @brief Simulated time and tick arithmetic.
     */
    Clock clock;

    /**
     * @brief Byte channel to and from the firmware.
     */
    SerialBus serial;

    /**
     * @brief What crosses the proxy boundary.
     */
    ProxyState proxy_state;

    /**
     * @brief Program running in lockstep, or null when nothing is driving one.
     *
     * @note Set by the wiring entry point before the run starts. The Stopwatch
     *       reaches it to hand the tick over when the firmware waits for time
     *       to pass; without it a stopwatch simply reports the clock.
     */
    std::atomic<FirmwareThread*> firmware{nullptr};
};

/**
 * @brief Dereference the context a proxy was configured with.
 *
 * @param context Context taken from the proxy Config.
 * @return Reference to the context.
 */
SimulationContext& require_context(SimulationContext* context);

/**
 * @brief Hand the current tick over to the process-wide simulation.
 *
 * @note For the wiring that stands in for the MCU startup code, which has no
 *       Config to reach the context through. Does nothing when no program is
 *       being driven.
 */
void yield_tick();
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_SIMULATION_CONTEXT_HPP

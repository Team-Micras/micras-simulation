/**
 * @file
 *
 * @brief The world, the clock and the byte channel one run advances.
 */

#ifndef MICRAS_SIM_CORE_RUN_CONTEXT_HPP
#define MICRAS_SIM_CORE_RUN_CONTEXT_HPP

#include <memory>
#include <vector>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/noise.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/devices/device.hpp"

namespace micras::sim {
/**
 * @brief Plain aggregate of what a run advances, with no logic of its own.
 *
 * @note Nothing here is a singleton. A robot target that has to reach the run
 *       from code it cannot construct, such as firmware globals initialised
 *       before main, owns one and hands it to the application.
 */
struct RunContext {
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
     * @brief Seed and switch of every device's noise.
     */
    NoiseConfig noise;

    /**
     * @brief The simulated board, in the order the run drives it.
     */
    std::vector<std::unique_ptr<Device>> devices;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_RUN_CONTEXT_HPP

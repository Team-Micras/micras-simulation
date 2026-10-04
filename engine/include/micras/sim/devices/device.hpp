/**
 * @file
 *
 * @brief A simulated piece of hardware between the firmware's ports and the physics.
 */

#ifndef MICRAS_SIM_DEVICES_DEVICE_HPP
#define MICRAS_SIM_DEVICES_DEVICE_HPP

#include <string>
#include <vector>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
/**
 * @brief One device of the simulated board: a motor, a sensor, a link.
 *
 * @note A device reaches the firmware only through the callables its
 *       configuration holds, which a robot target's bindings point at the host
 *       backend's ports; the engine never sees a HAL type. Every tick the run
 *       calls actuate() on every device, advances the physics, then calls
 *       sample() on every device, in the order they were added.
 */
class Device {
public:
    Device() = default;

    Device(const Device&) = delete;
    Device(Device&&) = delete;
    Device& operator=(const Device&) = delete;
    Device& operator=(Device&&) = delete;

    virtual ~Device() = default;

    /**
     * @brief Turn what the firmware wrote into what the physics feels.
     *
     * @param world World about to be advanced.
     * @param clock Clock, still at the start of the tick.
     */
    virtual void actuate(MujocoWorld& world, const Clock& clock);

    /**
     * @brief Turn the physics into what the firmware will read.
     *
     * @param world World that just advanced.
     * @param clock Clock, already at the end of the tick.
     */
    virtual void sample(MujocoWorld& world, const Clock& clock);

    /**
     * @brief Get the names of the columns this device records.
     *
     * @return Column names, possibly none.
     */
    virtual std::vector<std::string> columns() const;

    /**
     * @brief Append this device's cells to the current row.
     *
     * @param row Row being built; one cell per column.
     */
    virtual void append(std::vector<CsvCell>& row) const;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_DEVICE_HPP

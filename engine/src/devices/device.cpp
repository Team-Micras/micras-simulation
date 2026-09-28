/**
 * @file
 */

#include <string>
#include <vector>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/devices/device.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
void Device::actuate(MujocoWorld& /*world*/, const Clock& /*clock*/) { }

void Device::sample(MujocoWorld& /*world*/, const Clock& /*clock*/) { }

std::vector<std::string> Device::columns() const {
    return {};
}

void Device::append(std::vector<CsvCell>& /*row*/) const { }
}  // namespace micras::sim

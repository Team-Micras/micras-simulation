/**
 * @file
 */

#include "micras/sim/devices/device.hpp"

namespace micras::sim {
void Device::actuate(MujocoWorld& /*world*/, const Clock& /*clock*/) { }

void Device::sample(MujocoWorld& /*world*/, const Clock& /*clock*/) { }

std::vector<std::string> Device::columns() const {
    return {};
}

void Device::append(std::vector<CsvCell>& /*row*/) const { }
}  // namespace micras::sim

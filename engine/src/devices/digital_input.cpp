/**
 * @file
 */

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "micras/sim/devices/digital_input.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
DigitalInput::DigitalInput(Config config) : config{std::move(config)} {
    this->set(false);
}

void DigitalInput::set(bool active) {
    this->active = active;
    this->config.drive(active != this->config.active_low);
}

std::vector<std::string> DigitalInput::columns() const {
    return {this->config.name};
}

void DigitalInput::append(std::vector<CsvCell>& row) const {
    row.emplace_back(static_cast<int64_t>(this->active));
}
}  // namespace micras::sim

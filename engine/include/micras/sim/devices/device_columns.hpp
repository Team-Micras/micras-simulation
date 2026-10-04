/**
 * @file
 *
 * @brief The columns every device of a run records, in the order they were added.
 */

#ifndef MICRAS_SIM_DEVICES_DEVICE_COLUMNS_HPP
#define MICRAS_SIM_DEVICES_DEVICE_COLUMNS_HPP

#include <memory>
#include <string>
#include <vector>

#include "micras/sim/devices/device.hpp"
#include "micras/sim/recording/column_source.hpp"

namespace micras::sim {
/**
 * @brief Records what crossed the boundary between the firmware and the world.
 */
class DeviceColumns : public ColumnSource {
public:
    /**
     * @brief Take the run's devices.
     *
     * @param devices The devices; they must outlive the recorder.
     */
    explicit DeviceColumns(const std::vector<std::unique_ptr<Device>>& devices) : devices{devices} { }

    /**
     * @brief Get every device's column names.
     *
     * @return Column names.
     */
    std::vector<std::string> names() override {
        std::vector<std::string> names;

        for (const auto& device : this->devices) {
            const std::vector<std::string> columns = device->columns();
            names.insert(names.end(), columns.begin(), columns.end());
        }

        return names;
    }

    /**
     * @brief Append every device's cells.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) override {
        for (const auto& device : this->devices) {
            device->append(row);
        }
    }

private:
    const std::vector<std::unique_ptr<Device>>& devices;  // NOLINT(*-avoid-const-or-ref-data-members): the run's.
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_DEVICES_DEVICE_COLUMNS_HPP

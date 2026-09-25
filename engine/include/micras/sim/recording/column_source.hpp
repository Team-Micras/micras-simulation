/**
 * @file
 *
 * @brief Columns a robot target adds to every CSV row.
 */

#ifndef MICRAS_SIM_RECORDING_COLUMN_SOURCE_HPP
#define MICRAS_SIM_RECORDING_COLUMN_SOURCE_HPP

#include <string>
#include <vector>

#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
/**
 * @brief A block of CSV columns the recorder writes after the ground truth.
 *
 * @note names() is asked once, when the first row is written, so a source whose
 *       columns depend on something the firmware reports during the first tick
 *       can still name them.
 */
class ColumnSource {
public:
    ColumnSource() = default;

    ColumnSource(const ColumnSource&) = delete;
    ColumnSource(ColumnSource&&) = delete;
    ColumnSource& operator=(const ColumnSource&) = delete;
    ColumnSource& operator=(ColumnSource&&) = delete;

    virtual ~ColumnSource() = default;

    /**
     * @brief Get the column names, in the order append() fills them.
     *
     * @return Column names.
     */
    virtual std::vector<std::string> names() = 0;

    /**
     * @brief Append this source's cells for the current row.
     *
     * @param row Row being built; exactly one cell per name must be appended.
     */
    virtual void append(std::vector<CsvCell>& row) = 0;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_COLUMN_SOURCE_HPP

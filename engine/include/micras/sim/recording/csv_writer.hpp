/**
 * @file
 *
 * @brief Line-oriented CSV writer with a fixed numeric format.
 */

#ifndef MICRAS_SIM_RECORDING_CSV_WRITER_HPP
#define MICRAS_SIM_RECORDING_CSV_WRITER_HPP

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace micras::sim {
/**
 * @brief One CSV cell: integers are written in full, doubles with nine significant digits.
 */
using CsvCell = std::variant<int64_t, uint64_t, double>;

/**
 * @brief Writes a header and rows to a CSV file, flushing after every row.
 *
 * @note Doubles are formatted exactly as printf("%.9g") would, through
 *       std::to_chars, which is additionally immune to the global locale; NaN
 *       is written as "nan". Every row is flushed so a crash still leaves the
 *       completed rows on disk.
 */
class CsvWriter {
public:
    /**
     * @brief Open the file for writing, truncating it.
     *
     * @param path Path of the CSV file.
     */
    explicit CsvWriter(const std::filesystem::path& path);

    /**
     * @brief Write the header line.
     *
     * @param columns Column names, in order.
     */
    void write_header(std::span<const std::string> columns);

    /**
     * @brief Write one row and flush.
     *
     * @param cells Cell values, in column order.
     */
    void write_row(std::span<const CsvCell> cells);

    /**
     * @brief Format a cell the way it appears in the file.
     *
     * @param cell Cell value.
     * @return Formatted text.
     */
    static std::string format(const CsvCell& cell);

private:
    /**
     * @brief Output stream.
     */
    std::ofstream file;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_CSV_WRITER_HPP

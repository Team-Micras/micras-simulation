/**
 * @file
 */

#include <array>
#include <charconv>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <variant>

#include "micras/sim/core/span_at.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
CsvWriter::CsvWriter(const std::filesystem::path& path) : file{path} {
    if (not this->file.is_open()) {
        throw std::runtime_error("failed to open '" + path.string() + "' for writing");
    }
}

void CsvWriter::write_header(std::span<const std::string> columns) {
    for (std::size_t i = 0; i < columns.size(); i++) {
        this->file << (i > 0 ? "," : "") << at(columns, i);
    }

    this->file << '\n';
}

void CsvWriter::write_row(std::span<const CsvCell> cells) {
    for (std::size_t i = 0; i < cells.size(); i++) {
        this->file << (i > 0 ? "," : "") << format(at(cells, i));
    }

    this->file << '\n' << std::flush;
}

std::string CsvWriter::format(const CsvCell& cell) {
    return std::visit(
        [](auto value) -> std::string {
            if constexpr (std::is_same_v<decltype(value), double>) {
                std::array<char, 32> buffer{};
                const auto           result =
                    std::to_chars(buffer.data(), std::to_address(buffer.end()), value, std::chars_format::general, 9);

                if (result.ec != std::errc{}) {
                    throw std::runtime_error("a CSV cell did not fit the formatting buffer");
                }

                return std::string(buffer.data(), result.ptr);
            } else {
                return std::to_string(value);
            }
        },
        cell
    );
}
}  // namespace micras::sim

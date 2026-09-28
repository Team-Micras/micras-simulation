/**
 * @file
 */

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "micras/sim/recording/csv_recorder.hpp"

namespace micras::sim {
CsvRecorder::CsvRecorder(
    const MujocoWorld& world, GroundTruthConfig config, const std::filesystem::path& path, uint32_t every
) :
    ground_truth{world, std::move(config)}, writer{path}, every{std::max<uint32_t>(1, every)} { }

void CsvRecorder::add_source(ColumnSource& source) {
    if (this->header_written) {
        throw std::logic_error("a column source was added after the CSV header was written");
    }

    this->sources.push_back(&source);
}

void CsvRecorder::write_header() {
    std::vector<std::string> columns = this->ground_truth.columns();

    for (ColumnSource* source : this->sources) {
        const std::vector<std::string> names = source->names();
        this->widths.push_back(names.size());
        columns.insert(columns.end(), names.begin(), names.end());
    }

    std::set<std::string_view> seen;

    for (const std::string& column : columns) {
        if (not seen.insert(column).second) {
            throw std::runtime_error("two column sources both write a column named " + column);
        }
    }

    this->writer.write_header(columns);
    this->header_written = true;
}

void CsvRecorder::on_after_tick(const Simulation& simulation) {
    if (simulation.tick() % this->every == 0) {
        this->sample(simulation.tick());
    }
}

void CsvRecorder::sample(uint64_t tick) {
    if (not this->header_written) {
        if (not std::ranges::all_of(this->sources, [](const ColumnSource* source) { return source->ready(); })) {
            return;
        }

        this->write_header();
    }

    std::vector<CsvCell> cells = this->ground_truth.sample(tick);

    for (std::size_t i = 0; i < this->sources.size(); i++) {
        const std::size_t before = cells.size();
        this->sources[i]->append(cells);

        if (cells.size() - before != this->widths[i]) {
            throw std::logic_error(
                "a column source appended " + std::to_string(cells.size() - before) + " cells for " +
                std::to_string(this->widths[i]) + " columns"
            );
        }
    }

    this->writer.write_row(cells);
}
}  // namespace micras::sim

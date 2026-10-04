/**
 * @file
 *
 * @brief One CSV row per firmware tick: ground truth followed by the robot's own columns.
 */

#ifndef MICRAS_SIM_RECORDING_CSV_RECORDER_HPP
#define MICRAS_SIM_RECORDING_CSV_RECORDER_HPP

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

#include "micras/sim/core/simulation.hpp"
#include "micras/sim/recording/column_source.hpp"
#include "micras/sim/recording/csv_writer.hpp"
#include "micras/sim/recording/ground_truth.hpp"

namespace micras::sim {
/**
 * @brief Samples the simulation state once per firmware tick into a CSV file.
 *
 * @note A row is the ground truth, then every column source in the order it
 *       was added. The header is written with the first row, and the first row
 *       is the first one due once every source is ready, so a source may learn
 *       its column names while the run goes. Two sources naming the same column
 *       is an error, not a second column nobody can tell apart.
 */
class CsvRecorder : public IRunListener {
public:
    /**
     * @brief Open the CSV file and resolve every model id the ground truth needs.
     *
     * @param world Simulation world to sample.
     * @param config Robot body, forward axis and ground truth columns.
     * @param path Path of the CSV file to write.
     * @param every Ticks between two rows.
     */
    CsvRecorder(
        const MujocoWorld& world, GroundTruthConfig config, const std::filesystem::path& path, uint32_t every = 1
    );

    /**
     * @brief Add a block of columns after the ones already added.
     *
     * @param source Source to add; it must outlive the recorder.
     */
    void add_source(ColumnSource& source);

    /**
     * @brief Write one CSV row for the current simulation state, unless a source is not ready yet.
     *
     * @param tick Index of the firmware tick that was just executed.
     */
    void sample(uint64_t tick);

    /**
     * @brief Write the row of the tick that just ran, when it falls on the decimation.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

private:
    /**
     * @brief Ask every source for its names and write the header.
     */
    void write_header();

    /**
     * @brief Physical state sampler.
     */
    GroundTruth ground_truth;

    /**
     * @brief Output file.
     */
    CsvWriter writer;

    /**
     * @brief Column sources, in the order their columns appear.
     */
    std::vector<ColumnSource*> sources;

    /**
     * @brief How many columns each source named, checked on every row.
     */
    std::vector<std::size_t> widths;

    /**
     * @brief Ticks between two rows.
     */
    uint32_t every;

    /**
     * @brief Whether the header was already written.
     */
    bool header_written{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_CSV_RECORDER_HPP

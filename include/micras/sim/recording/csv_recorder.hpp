/**
 * @file
 *
 * @brief One CSV row per firmware tick: ground truth followed by the firmware pool.
 */

#ifndef MICRAS_SIM_RECORDING_CSV_RECORDER_HPP
#define MICRAS_SIM_RECORDING_CSV_RECORDER_HPP

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "micras/sim/core/proxy_state.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/recording/csv_writer.hpp"
#include "micras/sim/recording/ground_truth.hpp"
#include "micras/sim/telemetry/telemetry.hpp"

namespace micras::sim {
/**
 * @brief Samples the simulation state once per firmware tick into a CSV file.
 *
 * @note The header is written on the first sample, once the firmware has
 *       answered the variable map request, so every pool column is known.
 */
class CsvRecorder : public IRunListener {
public:
    /**
     * @brief Open the CSV file and resolve every model id it needs.
     *
     * @note A program that does not serve the firmware variable pool, such as
     *       a hardware test, passes no decoder; the run then records the ground
     *       truth and the proxy boundary only.
     *
     * @param world Simulation world to sample.
     * @param proxy_state Record of what crossed the proxy boundary.
     * @param telemetry Decoder holding the firmware pool values, or null.
     * @param path Path of the CSV file to write.
     */
    CsvRecorder(
        const MujocoWorld& world, const ProxyState& proxy_state, const Telemetry* telemetry,
        const std::filesystem::path& path
    );

    /**
     * @brief Write one CSV row for the current simulation state.
     *
     * @param tick Index of the firmware tick that was just executed.
     */
    void sample(uint64_t tick);

    /**
     * @brief Write the row of the tick that just ran.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

    /**
     * @brief Get the number of firmware pool columns written.
     *
     * @return Number of pool columns.
     */
    std::size_t pool_column_count() const { return this->pool_columns.size(); }

    /**
     * @brief Get the robot body height sampled on the last row.
     *
     * @return Height of the robot body origin in meters.
     */
    double last_z() const { return this->ground_truth.last_z(); }

    /**
     * @brief Get the proxy boundary column names, in the order sample() fills them.
     *
     * @note The interface output is deliberately absent: this firmware build
     *       never lights the LED, sets an ARGB colour or plays a tone, so the
     *       columns would be constant zero. ProxyState still carries it, for
     *       the panel and for the day the firmware uses it.
     *
     * @return Column names, all prefixed to keep them apart from the pool ones.
     */
    static std::vector<std::string> proxy_columns();

    /**
     * @brief Turn a pool variable name into a CSV friendly column name.
     *
     * @param name Original variable name.
     * @return Snake case name.
     */
    static std::string column_name(const std::string& name);

private:
    /**
     * @brief One CSV column fed by a firmware pool variable.
     *
     * @note Custom serializable types expand into several columns, so a
     *       variable may own more than one entry, one per serialized field.
     */
    struct PoolColumn {
        uint16_t    id;
        std::size_t component;
        std::string name;
    };

    /**
     * @brief Build the pool columns from the variable map and write the header.
     */
    void write_header();

    /**
     * @brief Get the monitoring variables the run records, if any.
     *
     * @return The variables, empty when the program serves no pool.
     */
    std::vector<PoolVariable> pool_variables() const;

    /**
     * @brief Decoder holding the firmware pool values, or null when there is none.
     */
    const Telemetry* telemetry;

    /**
     * @brief Record of what crossed the proxy boundary.
     *
     * @note Bound for the life of the recorder; it is fed by the caller that
     *       also owns the run.
     */
    const ProxyState& proxy_state;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Physical state sampler.
     */
    GroundTruth ground_truth;

    /**
     * @brief Output file.
     */
    CsvWriter writer;

    /**
     * @brief Pool columns written after the ground truth columns.
     */
    std::vector<PoolColumn> pool_columns;

    /**
     * @brief Whether the header was already written.
     */
    bool header_written{false};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_CSV_RECORDER_HPP

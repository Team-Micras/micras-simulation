/**
 * @file
 *
 * @brief Provenance of one run, written as meta.json next to data.csv.
 */

#ifndef MICRAS_SIM_RECORDING_RUN_METADATA_HPP
#define MICRAS_SIM_RECORDING_RUN_METADATA_HPP

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>

namespace micras::sim {
/**
 * @brief Everything needed to reproduce and compare a run.
 *
 * @note Nothing measured off the wall clock belongs here: two runs of the same
 *       binary with the same arguments must produce byte-identical files.
 */
struct RunMetadata {
    std::string firmware_sha;
    std::string model_path;
    std::string model_sha256;
    std::string maze_path;
    std::string mujoco_version;
    std::string compiler;
    std::string build_type;
    std::string args;
    uint32_t    loop_time_us{0};
    double      timestep{0.0};
    int         steps_per_tick{0};
    uint64_t    requested_ticks{0};
    uint64_t    ticks{0};
    double      sim_time{0.0};
    double      final_z{0.0};
    std::size_t pool_columns{0};
    uint32_t    telemetry_resyncs{0};
    int64_t     warnings_total{0};

    /**
     * @brief Whether a human changed anything the physics could see.
     *
     * @note An interactive run is not reproducible and is excluded from the
     *       comparisons the gate makes.
     */
    bool interactive{false};

    /**
     * @brief Serialize to the pretty-printed JSON layout of meta.json.
     *
     * @note Written in the classic locale on purpose: provenance must not
     *       depend on whatever a viewer or a bridge library installed, because
     *       meta.json is compared across runs.
     *
     * @return JSON text, newline terminated.
     */
    std::string to_json() const;

    /**
     * @brief Write meta.json.
     *
     * @param path Path of the file to write.
     */
    void write(const std::filesystem::path& path) const;

    /**
     * @brief Compute the SHA-256 digest of a file.
     *
     * @param path Path to the file.
     * @return Lowercase hexadecimal digest, or an empty string if the file is unreadable.
     */
    static std::string sha256_of(const std::filesystem::path& path);

    /**
     * @brief Join the command line arguments the way meta.json records them.
     *
     * @param arguments Whole command line, program name included.
     * @return Arguments after the program name, space separated.
     */
    static std::string join_args(std::span<char*> arguments);
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_RUN_METADATA_HPP

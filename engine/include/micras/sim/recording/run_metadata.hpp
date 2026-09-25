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
#include <vector>

namespace micras::sim {
/**
 * @brief A counter a robot target adds to meta.json.
 */
struct MetadataField {
    /**
     * @brief Key in meta.json.
     */
    std::string name;

    /**
     * @brief Value, written as a JSON integer.
     */
    int64_t value{0};
};

/**
 * @brief Something the run noticed at full rate, whatever the recording's decimation.
 */
struct RunEvent {
    /**
     * @brief Simulated time, in seconds.
     */
    double time{0.0};

    /**
     * @brief What happened, for example "collision" or "state".
     */
    std::string kind;

    /**
     * @brief With what, for example the geom or the state's name.
     */
    std::string detail;
};

/**
 * @brief Everything needed to reproduce and compare a run.
 *
 * @note Nothing measured off the wall clock belongs here: two runs of the same
 *       binary with the same arguments must produce byte-identical files.
 */
struct RunMetadata {
    /**
     * @brief Name of the robot target that ran, which the analysis plugins key on.
     */
    std::string target;

    std::string firmware_sha;
    std::string robot_path;
    std::string robot_sha256;
    std::string scenario_path;
    std::string maze_path;

    /**
     * @brief Digest of the composed model, robot and arena, as saved in model.xml.
     */
    std::string model_sha256;

    std::string mujoco_version;
    std::string compiler;
    std::string build_type;
    std::string args;
    uint64_t    seed{0};
    bool        ideal{false};
    uint32_t    loop_time_us{0};
    double      timestep{0.0};
    int         steps_per_tick{0};
    uint32_t    record_every{1};
    uint64_t    requested_ticks{0};
    uint64_t    ticks{0};
    double      sim_time{0.0};

    /**
     * @brief When the scenario's stop condition held, in seconds; negative when it never did.
     */
    double stopped_at{-1.0};

    double final_z{0.0};

    /**
     * @brief The robot target's own counters, written in this order after final_z.
     */
    std::vector<MetadataField> target_fields;

    int64_t warnings_total{0};

    /**
     * @brief Bytes for the firmware dropped because the link's queue was full.
     */
    uint64_t serial_dropped_bytes{0};

    /**
     * @brief Frames dropped because a connected monitor did not keep up.
     *
     * @note Not behaviour: frames leave after the tick and never reach the firmware.
     */
    uint64_t bridge_dropped_frames{0};

    /**
     * @brief Whether a human changed anything the physics could see.
     *
     * @note An interactive run is not reproducible and is excluded from the
     *       comparisons the gate makes.
     */
    bool interactive{false};

    /**
     * @brief What the run noticed at full rate, in time order.
     */
    std::vector<RunEvent> events;

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
     * @brief Compute the SHA-256 digest of a text.
     *
     * @param text The text.
     * @return Lowercase hexadecimal digest.
     */
    static std::string sha256_of_text(const std::string& text);

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

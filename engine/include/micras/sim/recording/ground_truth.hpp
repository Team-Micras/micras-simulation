/**
 * @file
 *
 * @brief Physical state of the robot read straight from MuJoCo.
 */

#ifndef MICRAS_SIM_RECORDING_GROUND_TRUTH_HPP
#define MICRAS_SIM_RECORDING_GROUND_TRUTH_HPP

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
/**
 * @brief Body axis the robot drives along, which v_forward projects onto.
 */
enum class ForwardAxis : uint8_t {
    X,
    Y,
};

/**
 * @brief Quantity one ground truth column reads.
 *
 * @note Joint probes name a joint, actuator probes an actuator and contact
 *       probes a geom. The last three read the whole world and take no object.
 */
enum class Probe : uint8_t {
    JOINT_VELOCITY,
    JOINT_POSITION,
    ACTUATOR_CONTROL,
    ACTUATOR_FORCE,
    CONTACT_COUNT,
    CONTACT_NORMAL_FORCE,
    CONTACT_SLIP,
    CONTACT_PENETRATION,
    CONTACT_TOTAL,
    SOLVER_ITERATIONS,
    WARNINGS_TOTAL,
};

/**
 * @brief One robot-specific ground truth column.
 */
struct GroundTruthColumn {
    /**
     * @brief Column name in the CSV header.
     */
    std::string name;

    /**
     * @brief What the column reads.
     */
    Probe probe;

    /**
     * @brief Name of the MuJoCo joint, actuator or geom, empty for world probes.
     */
    std::string object;
};

/**
 * @brief What a robot target tells the recorder about its model.
 */
struct GroundTruthConfig {
    /**
     * @brief Robot body, whose first joint must be a free joint.
     */
    std::string body;

    /**
     * @brief Body axis the robot drives along.
     */
    ForwardAxis forward_axis{ForwardAxis::X};

    /**
     * @brief Columns written after the body pose and velocity, in this order.
     */
    std::vector<GroundTruthColumn> columns;
};

/**
 * @brief Resolves the model ids once and samples the ground truth columns every tick.
 *
 * @note Every row starts with the same block: tick, sim_time, the
 *       body pose and velocity, and v_forward. The robot's own columns follow,
 *       in the order its config lists them.
 *
 * @note MuJoCo free joints split their six velocity dofs across two frames:
 *       the three linear dofs are expressed in the world frame, the three
 *       angular dofs in the body frame. The columns are named accordingly
 *       (vx_world, vy_world, vz_world, wz_body) and v_forward projects the
 *       world linear velocity onto the body forward axis. This is the one class
 *       that indexes the MuJoCo C arrays directly.
 */
class GroundTruth {
public:
    /**
     * @brief Resolve every model id the columns need.
     *
     * @param world Loaded simulation world.
     * @param config Robot body, forward axis and columns.
     */
    GroundTruth(const MujocoWorld& world, GroundTruthConfig config);

    /**
     * @brief Get the column names, in the order sample() fills them.
     *
     * @return Column names.
     */
    std::vector<std::string> columns() const;

    /**
     * @brief Sample the current physical state.
     *
     * @param tick Index of the firmware tick that was just executed.
     * @return One cell per column.
     */
    std::vector<CsvCell> sample(uint64_t tick);

    /**
     * @brief Sum every MuJoCo warning counter raised so far.
     *
     * @param world Loaded simulation world.
     * @return Total number of warnings.
     */
    static int64_t warnings_total(const MujocoWorld& world);

    /**
     * @brief Get the robot body height sampled on the last row.
     *
     * @return Height of the robot body origin in meters.
     */
    double last_z() const { return this->last_body_z; }

private:
    /**
     * @brief Per-geom contact statistics for a single step.
     */
    struct ContactStats {
        int    count{0};
        double normal_force{0.0};
        double slip{0.0};

        /**
         * @brief Deepest penetration over the geom's contacts, NaN when it has none.
         */
        double penetration{std::numeric_limits<double>::quiet_NaN()};
    };

    /**
     * @brief A column with its model address resolved.
     */
    struct ResolvedColumn {
        /**
         * @brief What the column reads.
         */
        Probe probe;

        /**
         * @brief Index into the mjData array the probe reads, or the geom id.
         */
        int address;
    };

    /**
     * @brief Collect the contact statistics of one geom.
     *
     * @param geom_id Id of the geom.
     * @return Aggregated statistics.
     */
    ContactStats collect_contacts(int geom_id) const;

    /**
     * @brief Get the contact statistics of one geom for the current row.
     *
     * @note Collected once per geom and row, however many columns read them.
     *
     * @param geom_id Id of the geom.
     * @return Aggregated statistics.
     */
    const ContactStats& contacts_of(int geom_id);

    /**
     * @brief Read one robot-specific column.
     *
     * @param column Resolved column.
     * @return Its cell.
     */
    CsvCell read(const ResolvedColumn& column);

    /**
     * @brief Simulation world being sampled.
     *
     * @note Bound for the life of the recorder; it is fed by the caller that
     *       also owns the run.
     */
    const MujocoWorld& world;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Robot body, forward axis and columns.
     */
    GroundTruthConfig config;

    /**
     * @brief Id of the robot body.
     */
    int body_id;

    /**
     * @brief First velocity dof of the robot's free joint.
     */
    int free_joint_qvel;

    /**
     * @brief The robot-specific columns, resolved.
     */
    std::vector<ResolvedColumn> resolved;

    /**
     * @brief Contact statistics of the current row, per geom id.
     */
    std::unordered_map<int, ContactStats> row_contacts;

    /**
     * @brief Height of the robot body origin on the last sampled row.
     */
    double last_body_z{0.0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_GROUND_TRUTH_HPP

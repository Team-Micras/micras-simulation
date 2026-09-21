/**
 * @file
 *
 * @brief Physical state of the robot read straight from MuJoCo.
 */

#ifndef MICRAS_SIM_RECORDING_GROUND_TRUTH_HPP
#define MICRAS_SIM_RECORDING_GROUND_TRUTH_HPP

#include <limits>
#include <string>
#include <vector>

#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
/**
 * @brief Resolves the robot ids once and samples the ground truth columns every tick.
 *
 * @note MuJoCo free joints split their six velocity dofs across two frames:
 *       the three linear dofs are expressed in the world frame, the three
 *       angular dofs in the body frame. The columns are named accordingly
 *       (vx_world, vy_world, vz_world, wz_body) and v_forward projects the
 *       world linear velocity onto the body forward axis, which is body +y.
 *       This is the one class that indexes the MuJoCo C arrays directly.
 */
class GroundTruth {
public:
    /**
     * @brief Resolve every model id the columns need.
     *
     * @param world Loaded simulation world.
     */
    explicit GroundTruth(const MujocoWorld& world);

    /**
     * @brief Get the column names, in the order sample() fills them.
     *
     * @return Column names.
     */
    static std::vector<std::string> columns();

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
     * @brief Collect the contact statistics of one geom.
     *
     * @param geom_id Id of the geom.
     * @return Aggregated statistics.
     */
    ContactStats collect_contacts(int geom_id) const;

    /**
     * @brief Simulation world being sampled.
     *
     * @note Bound for the life of the recorder; it is fed by the caller that
     *       also owns the run.
     */
    const MujocoWorld& world;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Ids resolved once from the model.
     */
    ///@{
    int body_id;
    int left_wheel_geom;
    int right_wheel_geom;
    int caster_geom;
    int base_geom;
    int left_actuator;
    int right_actuator;
    int free_joint_qvel{};
    int left_wheel_qvel{};
    int right_wheel_qvel{};
    int left_wheel_qpos{};
    int right_wheel_qpos{};
    ///@}

    /**
     * @brief Height of the robot body origin on the last sampled row.
     */
    double last_body_z{0.0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_RECORDING_GROUND_TRUTH_HPP

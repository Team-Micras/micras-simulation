/**
 * @file
 *
 * @brief Owner of the MuJoCo model and data.
 */

#ifndef MICRAS_SIM_CORE_MUJOCO_WORLD_HPP
#define MICRAS_SIM_CORE_MUJOCO_WORLD_HPP

#include <filesystem>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>

namespace micras::sim {
/**
 * @brief Loads a model, owns its data and resolves model objects by name.
 *
 * @note Every accessor throws a clear error when the model is not loaded yet,
 *       so a device built too early fails at construction instead of
 *       dereferencing a null pointer in the middle of a run.
 */
class MujocoWorld {
public:
    /**
     * @brief Geom group the range sensors do not see: the robot's own geoms, and what is only paint
     *        on the arena.
     */
    static constexpr int unseen_group{3};

    MujocoWorld() = default;

    MujocoWorld(const MujocoWorld&) = delete;
    MujocoWorld(MujocoWorld&&) = delete;
    MujocoWorld& operator=(const MujocoWorld&) = delete;
    MujocoWorld& operator=(MujocoWorld&&) = delete;

    /**
     * @brief Free the MuJoCo model and data.
     */
    ~MujocoWorld();

    /**
     * @brief Load a model and allocate its data, replacing any previous one.
     *
     * @note Automatic reset is disabled on the loaded model so a diverging step
     *       is reported instead of silently restarting the run.
     *
     * @param model_path Path to the model XML file.
     */
    void load(const std::filesystem::path& model_path);

    /**
     * @brief Where a robot is placed in its arena.
     */
    struct Placement {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    /**
     * @brief Compose a robot and an arena into one model and allocate its data.
     *
     * @note The robot's model is the root, so its simulation options hold; the
     *       arena's body is attached to the world with the prefix "<body>_", and
     *       the robot's root body is moved to the placement. The composed MJCF is
     *       kept, so a run can save exactly what it simulated.
     *
     * @param robot_xml MJCF of the robot, standing at the origin.
     * @param robot_body Name of the robot's root body.
     * @param arena_xml MJCF of the arena, holding its geoms in one body.
     * @param arena_body Name of that body.
     * @param placement Where the robot starts.
     */
    void build(
        const std::string& robot_xml, std::string_view robot_body, const std::string& arena_xml,
        std::string_view arena_body, const Placement& placement
    );

    /**
     * @brief Get the MJCF of what was built.
     *
     * @return The composed model, empty after load().
     */
    const std::string& composed_xml() const { return this->composed; }

    /**
     * @brief Reset the simulation to its initial state and make sensordata valid.
     */
    void reset();

    /**
     * @brief Advance the physics by a number of steps.
     *
     * @param steps Number of mj_step calls.
     */
    void step(int steps);

    /**
     * @brief Resolve a named MuJoCo object, throwing if it does not exist.
     *
     * @param type MuJoCo object type.
     * @param name Name of the object.
     * @return Id of the object.
     */
    int require_id(mjtObj type, const std::string& name) const;

    /**
     * @brief Resolve the sensordata address of a named sensor.
     *
     * @param name Name of the sensor.
     * @return Index into mjData::sensordata.
     */
    int sensor_address(const std::string& name) const;

    /**
     * @brief Resolve where a named joint's first velocity dof sits in qvel.
     *
     * @param name Name of the joint.
     * @return Index into mjData::qvel.
     */
    int joint_dof(const std::string& name) const;

    /**
     * @brief Resolve where a named joint's first position coordinate sits in qpos.
     *
     * @param name Name of the joint.
     * @return Index into mjData::qpos.
     */
    int joint_qpos(const std::string& name) const;

    /**
     * @brief Read one sensordata entry.
     *
     * @param address Index into mjData::sensordata.
     * @return Value of the sensor channel.
     */
    double sensor_value(int address) const;

    /**
     * @brief Write one actuator control value.
     *
     * @param actuator_id Id of the actuator.
     * @param value Control value.
     */
    void set_control(int actuator_id, double value);

    /**
     * @brief Get the timestep the model integrates with.
     *
     * @return Timestep in seconds.
     */
    double timestep() const;

    /**
     * @brief Get the simulated time of the loaded data.
     *
     * @return Simulated seconds since the last reset.
     */
    double time() const;

    /**
     * @brief Get the loaded model.
     *
     * @note Exposed for the devices, the recorders and the renderer, which
     *       read the model's arrays directly.
     *
     * @return Pointer to the model.
     */
    ///@{
    const mjModel* model() const;
    mjModel*       model();
    ///@}

    /**
     * @brief Get the simulation data.
     *
     * @note Exposed for the devices, the recorders and the renderer, which
     *       read and write the state's arrays directly.
     *
     * @return Pointer to the data.
     */
    ///@{
    const mjData* data() const;
    mjData*       data();
    ///@}

private:
    /**
     * @brief Throw if no model is loaded.
     */
    void require_loaded() const;

    /**
     * @brief Release the model and data, if any.
     */
    void unload();

    /**
     * @brief Loaded MuJoCo model.
     */
    mjModel* model_handle{nullptr};

    /**
     * @brief Simulation data of the loaded model.
     */
    mjData* data_handle{nullptr};

    /**
     * @brief MJCF of the last composed model.
     */
    std::string composed;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_MUJOCO_WORLD_HPP

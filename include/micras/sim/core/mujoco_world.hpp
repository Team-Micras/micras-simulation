/**
 * @file
 *
 * @brief Owner of the MuJoCo model and data.
 */

#ifndef MICRAS_SIM_CORE_MUJOCO_WORLD_HPP
#define MICRAS_SIM_CORE_MUJOCO_WORLD_HPP

#include <filesystem>
#include <string>

#include <mujoco/mujoco.h>

namespace micras::sim {
/**
 * @brief Loads a model, owns its data and resolves model objects by name.
 *
 * @note Every accessor throws a clear error when the model is not loaded yet,
 *       so a proxy built too early fails at construction instead of
 *       dereferencing a null pointer in the middle of a run.
 */
class MujocoWorld {
public:
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
     * @brief Check whether a model is loaded.
     *
     * @return True once load() succeeded.
     */
    bool is_loaded() const { return this->model_handle != nullptr; }

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
     * @note Exposed for the recorder and the renderer, which need the whole
     *       model; the proxies use the named accessors instead.
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
     * @note Exposed for the recorder and the renderer, which need the whole
     *       state; the proxies use the named accessors instead.
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
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_MUJOCO_WORLD_HPP

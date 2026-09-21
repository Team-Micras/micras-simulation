/**
 * @file
 */

#include <array>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>

#include "micras/sim/core/mujoco_world.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Check an index against the array it is about to reach into.
 *
 * @param index Index to check.
 * @param size Size of the array.
 * @param what Name of the array, for the error message.
 * @return The index, as an unsigned offset.
 */
std::size_t require_index(int index, std::size_t size, const char* what) {
    if (index < 0 or static_cast<std::size_t>(index) >= size) {
        throw std::out_of_range(
            std::string(what) + " index " + std::to_string(index) + " is outside the " + std::to_string(size) +
            " entries the model has"
        );
    }

    return static_cast<std::size_t>(index);
}
}  // namespace

MujocoWorld::~MujocoWorld() {
    this->unload();
}

void MujocoWorld::load(const std::filesystem::path& model_path) {
    std::array<char, 1024> error{};
    mjModel* loaded = mj_loadXML(model_path.c_str(), nullptr, error.data(), static_cast<int>(error.size()));

    if (loaded == nullptr) {
        throw std::runtime_error("failed to load model '" + model_path.string() + "': " + std::string(error.data()));
    }

    mjData* allocated = mj_makeData(loaded);

    if (allocated == nullptr) {
        mj_deleteModel(loaded);
        throw std::runtime_error("failed to allocate mjData for '" + model_path.string() + "'");
    }

    this->unload();
    this->model_handle = loaded;
    this->data_handle = allocated;
    this->model_handle->opt.disableflags |= mjDSBL_AUTORESET;
}

void MujocoWorld::reset() {
    this->require_loaded();
    mj_resetData(this->model_handle, this->data_handle);
    mj_forward(this->model_handle, this->data_handle);
}

void MujocoWorld::step(int steps) {
    this->require_loaded();

    for (int i = 0; i < steps; i++) {
        mj_step(this->model_handle, this->data_handle);
    }
}

int MujocoWorld::require_id(mjtObj type, const std::string& name) const {
    this->require_loaded();
    const int id = mj_name2id(this->model_handle, type, name.c_str());

    if (id < 0) {
        throw std::runtime_error("model object not found: '" + name + "'");
    }

    return id;
}

int MujocoWorld::sensor_address(const std::string& name) const {
    const int                  id = this->require_id(mjOBJ_SENSOR, name);
    const std::span<const int> addresses(
        this->model_handle->sensor_adr, static_cast<std::size_t>(this->model_handle->nsensor)
    );
    return addresses[static_cast<std::size_t>(id)];
}

double MujocoWorld::sensor_value(int address) const {
    this->require_loaded();
    const std::span<const mjtNum> readings(
        this->data_handle->sensordata, static_cast<std::size_t>(this->model_handle->nsensordata)
    );
    return readings[require_index(address, readings.size(), "sensordata")];
}

void MujocoWorld::set_control(int actuator_id, double value) {
    this->require_loaded();
    const std::span<mjtNum> controls(this->data_handle->ctrl, static_cast<std::size_t>(this->model_handle->nu));
    controls[require_index(actuator_id, controls.size(), "actuator")] = value;
}

double MujocoWorld::timestep() const {
    this->require_loaded();
    return this->model_handle->opt.timestep;
}

double MujocoWorld::time() const {
    this->require_loaded();
    return this->data_handle->time;
}

const mjModel* MujocoWorld::model() const {
    this->require_loaded();
    return this->model_handle;
}

mjModel* MujocoWorld::model() {
    this->require_loaded();
    return this->model_handle;
}

const mjData* MujocoWorld::data() const {
    this->require_loaded();
    return this->data_handle;
}

mjData* MujocoWorld::data() {
    this->require_loaded();
    return this->data_handle;
}

void MujocoWorld::require_loaded() const {
    if (this->model_handle == nullptr or this->data_handle == nullptr) {
        throw std::logic_error("the MuJoCo world was used before a model was loaded");
    }
}

void MujocoWorld::unload() {
    if (this->data_handle != nullptr) {
        mj_deleteData(this->data_handle);
        this->data_handle = nullptr;
    }

    if (this->model_handle != nullptr) {
        mj_deleteModel(this->model_handle);
        this->model_handle = nullptr;
    }
}
}  // namespace micras::sim

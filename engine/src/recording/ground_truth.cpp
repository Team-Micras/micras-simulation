/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <stdexcept>
#include <utility>

#include "micras/sim/recording/ground_truth.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): mjData exposes flat C arrays indexed by id.

namespace micras::sim {
namespace {
/**
 * @brief Convert a MuJoCo quaternion to roll, pitch and yaw.
 *
 * @param quaternion Quaternion as w, x, y, z.
 * @return Roll, pitch and yaw in radians, ZYX convention.
 */
std::array<double, 3> to_euler(const mjtNum* quaternion) {
    const double scalar = quaternion[0];
    const double x = quaternion[1];
    const double y = quaternion[2];
    const double z = quaternion[3];

    const double roll = std::atan2(2.0 * (scalar * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sin_pitch = std::clamp(2.0 * (scalar * y - z * x), -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);
    const double yaw = std::atan2(2.0 * (scalar * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    return {roll, pitch, yaw};
}

/**
 * @brief Find the first velocity dof of a body's free joint.
 *
 * @param model Loaded model.
 * @param body_id Id of the body.
 * @param body Name of the body, for the error message.
 * @return Index into mjData::qvel.
 */
int free_joint_dof(const mjModel* model, int body_id, const std::string& body) {
    if (model->body_jntnum[body_id] < 1) {
        throw std::runtime_error("body '" + body + "' has no joint, a free joint is required");
    }

    const int root_joint = model->body_jntadr[body_id];

    if (model->jnt_type[root_joint] != mjJNT_FREE) {
        throw std::runtime_error(
            "the first joint of body '" + body + "' must be a free joint, got mjtJoint " +
            std::to_string(model->jnt_type[root_joint])
        );
    }

    return model->jnt_dofadr[root_joint];
}

/**
 * @brief Column names every row starts with, before the robot's own.
 */
const std::array<const char*, 13> common_columns{
    "tick", "sim_time", "x",        "y",        "z",       "roll",      "pitch",
    "yaw",  "vx_world", "vy_world", "vz_world", "wz_body", "v_forward",
};
}  // namespace

GroundTruth::GroundTruth(const MujocoWorld& world, GroundTruthConfig config) :
    world{world},
    config{std::move(config)},
    body_id{world.require_id(mjOBJ_BODY, this->config.body)},
    free_joint_qvel{free_joint_dof(world.model(), this->body_id, this->config.body)} {
    for (const GroundTruthColumn& column : this->config.columns) {
        int address = 0;

        switch (column.probe) {
            case Probe::JOINT_VELOCITY:
                address = world.joint_dof(column.object);
                break;

            case Probe::JOINT_POSITION:
                address = world.joint_qpos(column.object);
                break;

            case Probe::ACTUATOR_FORCE:
                address = world.require_id(mjOBJ_ACTUATOR, column.object);
                break;

            case Probe::CONTACT_COUNT:
            case Probe::CONTACT_NORMAL_FORCE:
            case Probe::CONTACT_SLIP:
            case Probe::CONTACT_PENETRATION:
                address = world.require_id(mjOBJ_GEOM, column.object);
                break;

            case Probe::SOLVER_ITERATIONS:
                break;
        }

        this->resolved.push_back({column.probe, address});
    }
}

int64_t GroundTruth::warnings_total(const MujocoWorld& world) {
    int64_t warnings = 0;

    for (const mjWarningStat& warning : std::span(world.data()->warning)) {
        warnings += warning.number;
    }

    return warnings;
}

std::vector<std::string> GroundTruth::columns() const {
    std::vector<std::string> names(common_columns.begin(), common_columns.end());

    for (const GroundTruthColumn& column : this->config.columns) {
        names.push_back(column.name);
    }

    return names;
}

GroundTruth::ContactStats GroundTruth::collect_contacts(int geom_id) const {
    const mjModel* model = this->world.model();
    const mjData*  data = this->world.data();
    ContactStats   stats;

    for (int i = 0; i < data->ncon; i++) {
        const mjContact& contact = data->contact[i];

        if (contact.efc_address < 0) {
            continue;
        }

        if (contact.geom[0] != geom_id and contact.geom[1] != geom_id) {
            continue;
        }

        stats.count++;
        stats.penetration = stats.count == 1 ? contact.dist : std::min(stats.penetration, contact.dist);

        std::array<mjtNum, 6> force{};
        mj_contactForce(model, data, i, force.data());
        stats.normal_force += force[0];

        if (contact.dim >= 3) {
            const int address = contact.efc_address;
            stats.slip = std::max(stats.slip, std::hypot(data->efc_vel[address + 1], data->efc_vel[address + 2]));
        }
    }

    return stats;
}

const GroundTruth::ContactStats& GroundTruth::contacts_of(int geom_id) {
    const auto found = this->row_contacts.find(geom_id);

    if (found != this->row_contacts.end()) {
        return found->second;
    }

    return this->row_contacts.emplace(geom_id, this->collect_contacts(geom_id)).first->second;
}

CsvCell GroundTruth::read(const ResolvedColumn& column) {
    const mjData* data = this->world.data();

    switch (column.probe) {
        case Probe::JOINT_VELOCITY:
            return data->qvel[column.address];

        case Probe::JOINT_POSITION:
            return data->qpos[column.address];

        case Probe::ACTUATOR_FORCE:
            return data->actuator_force[column.address];

        case Probe::CONTACT_COUNT:
            return static_cast<int64_t>(this->contacts_of(column.address).count);

        case Probe::CONTACT_NORMAL_FORCE:
            return this->contacts_of(column.address).normal_force;

        case Probe::CONTACT_SLIP:
            return this->contacts_of(column.address).slip;

        case Probe::CONTACT_PENETRATION:
            return this->contacts_of(column.address).penetration;

        case Probe::SOLVER_ITERATIONS:
            return static_cast<int64_t>(data->solver_niter[0]);
    }

    throw std::logic_error("unknown ground truth probe");
}

std::vector<CsvCell> GroundTruth::sample(uint64_t tick) {
    const mjData* data = this->world.data();
    const mjtNum* position = data->xpos + 3L * this->body_id;
    const auto    euler = to_euler(data->xquat + 4L * this->body_id);
    const mjtNum* velocity = data->qvel + this->free_joint_qvel;

    const double forward_speed = velocity[0] * std::cos(euler[2]) + velocity[1] * std::sin(euler[2]);

    std::vector<CsvCell> cells{
        tick,     data->time,  position[0], position[1], position[2], euler[0],      euler[1],
        euler[2], velocity[0], velocity[1], velocity[2], velocity[5], forward_speed,
    };

    this->row_contacts.clear();

    for (const ResolvedColumn& column : this->resolved) {
        cells.push_back(this->read(column));
    }

    return cells;
}
}  // namespace micras::sim

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

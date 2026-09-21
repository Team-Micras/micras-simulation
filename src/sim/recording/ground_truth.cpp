/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <stdexcept>

#include "micras/sim/recording/ground_truth.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): mjData exposes flat C arrays indexed by id.

namespace micras::sim {
namespace {
/**
 * @brief Nominal firmware loop period in seconds, written on every row.
 */
constexpr double firmware_loop_time = 1042.0e-6;

/**
 * @brief Convert a MuJoCo quaternion to roll, pitch and yaw.
 *
 * @param quaternion Quaternion in w, x, y, z order.
 * @return Array with roll, pitch and yaw in radians.
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
 * @brief Resolve the free joint the chassis floats on.
 *
 * @param model Loaded model.
 * @param body_id Id of the robot body.
 * @return Address of the first velocity dof of the free joint.
 */
int free_joint_dof(const mjModel* model, int body_id) {
    if (model->body_jntnum[body_id] < 1) {
        throw std::runtime_error("body 'micras' has no joint, a free joint is required");
    }

    const int root_joint = model->body_jntadr[body_id];

    if (model->jnt_type[root_joint] != mjJNT_FREE) {
        throw std::runtime_error(
            "the first joint of body 'micras' must be a free joint, got mjtJoint " +
            std::to_string(model->jnt_type[root_joint])
        );
    }

    return model->jnt_dofadr[root_joint];
}
}  // namespace

GroundTruth::GroundTruth(const MujocoWorld& world) :
    world{world},
    body_id{world.require_id(mjOBJ_BODY, "micras")},
    left_wheel_geom{world.require_id(mjOBJ_GEOM, "left_wheel")},
    right_wheel_geom{world.require_id(mjOBJ_GEOM, "right_wheel")},
    caster_geom{world.require_id(mjOBJ_GEOM, "front wheel")},
    base_geom{world.require_id(mjOBJ_GEOM, "base")},
    left_actuator{world.require_id(mjOBJ_ACTUATOR, "motor_left")},
    right_actuator{world.require_id(mjOBJ_ACTUATOR, "motor_right")},
    free_joint_qvel{free_joint_dof(world.model(), this->body_id)} {
    const mjModel* model = world.model();
    const int      left_wheel = world.require_id(mjOBJ_JOINT, "left_wheel");
    const int      right_wheel = world.require_id(mjOBJ_JOINT, "right_wheel");

    this->left_wheel_qvel = model->jnt_dofadr[left_wheel];
    this->right_wheel_qvel = model->jnt_dofadr[right_wheel];
    this->left_wheel_qpos = model->jnt_qposadr[left_wheel];
    this->right_wheel_qpos = model->jnt_qposadr[right_wheel];
}

int64_t GroundTruth::warnings_total(const MujocoWorld& world) {
    int64_t warnings = 0;

    for (const mjWarningStat& warning : std::span(world.data()->warning)) {
        warnings += warning.number;
    }

    return warnings;
}

std::vector<std::string> GroundTruth::columns() {
    return {
        "tick",
        "sim_time",
        "fw_loop_time",
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
        "vx_world",
        "vy_world",
        "vz_world",
        "wz_body",
        "v_forward",
        "wheel_qvel_left",
        "wheel_qvel_right",
        "ctrl_left",
        "ctrl_right",
        "act_force_left",
        "act_force_right",
        "left_ncon",
        "left_fn",
        "left_slip",
        "left_penetration",
        "right_ncon",
        "right_fn",
        "right_slip",
        "right_penetration",
        "caster_ncon",
        "ncon_total",
        "solver_niter",
        "warnings_total",
        "wheel_qpos_left",
        "wheel_qpos_right",
        "caster_fn",
        "base_ncon"
    };
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

std::vector<CsvCell> GroundTruth::sample(uint64_t tick) {
    const mjData* data = this->world.data();
    const mjtNum* position = data->xpos + 3L * this->body_id;
    const auto    euler = to_euler(data->xquat + 4L * this->body_id);
    const mjtNum* velocity = data->qvel + this->free_joint_qvel;

    this->last_body_z = position[2];

    const double forward_speed = -velocity[0] * std::sin(euler[2]) + velocity[1] * std::cos(euler[2]);

    const ContactStats left = this->collect_contacts(this->left_wheel_geom);
    const ContactStats right = this->collect_contacts(this->right_wheel_geom);
    const ContactStats caster = this->collect_contacts(this->caster_geom);
    const ContactStats base = this->collect_contacts(this->base_geom);

    return {
        tick,
        data->time,
        firmware_loop_time,
        position[0],
        position[1],
        position[2],
        euler[0],
        euler[1],
        euler[2],
        velocity[0],
        velocity[1],
        velocity[2],
        velocity[5],
        forward_speed,
        data->qvel[this->left_wheel_qvel],
        data->qvel[this->right_wheel_qvel],
        data->ctrl[this->left_actuator],
        data->ctrl[this->right_actuator],
        data->actuator_force[this->left_actuator],
        data->actuator_force[this->right_actuator],
        static_cast<int64_t>(left.count),
        left.normal_force,
        left.slip,
        left.penetration,
        static_cast<int64_t>(right.count),
        right.normal_force,
        right.slip,
        right.penetration,
        static_cast<int64_t>(caster.count),
        static_cast<int64_t>(data->ncon),
        static_cast<int64_t>(data->solver_niter[0]),
        warnings_total(this->world),
        data->qpos[this->left_wheel_qpos],
        data->qpos[this->right_wheel_qpos],
        caster.normal_force,
        static_cast<int64_t>(base.count),
    };
}
}  // namespace micras::sim

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

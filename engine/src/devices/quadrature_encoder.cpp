/**
 * @file
 */

#include <cmath>
#include <numbers>
#include <span>
#include <string>
#include <utility>

#include "micras/sim/devices/quadrature_encoder.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Find where a joint's first position coordinate sits in qpos.
 *
 * @param world The world.
 * @param joint Name of the joint.
 * @return The joint's qpos address.
 */
int qpos_address(const MujocoWorld& world, const std::string& joint) {
    const std::span<const int> addresses(world.model()->jnt_qposadr, static_cast<std::size_t>(world.model()->njnt));
    return addresses[static_cast<std::size_t>(world.require_id(mjOBJ_JOINT, joint))];
}
}  // namespace

QuadratureEncoder::QuadratureEncoder(const MujocoWorld& world, Config config) :
    config{std::move(config)}, position_address{qpos_address(world, this->config.joint)} { }

void QuadratureEncoder::sample(MujocoWorld& world, const Clock& /*clock*/) {
    const std::span<const mjtNum> positions(world.data()->qpos, static_cast<std::size_t>(world.model()->nq));
    const double                  angle = positions[static_cast<std::size_t>(this->position_address)];

    this->count =
        static_cast<int32_t>(std::floor(angle * this->config.counts_per_revolution / (2.0 * std::numbers::pi)));
    this->config.write(this->count);
}

std::vector<std::string> QuadratureEncoder::columns() const {
    return {this->config.name + "_count"};
}

void QuadratureEncoder::append(std::vector<CsvCell>& row) const {
    row.emplace_back(static_cast<int64_t>(this->count));
}
}  // namespace micras::sim

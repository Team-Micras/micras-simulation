/**
 * @file
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mjtype.h>

#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/span_at.hpp"
#include "micras/sim/devices/quadrature_encoder.hpp"
#include "micras/sim/recording/csv_writer.hpp"

namespace micras::sim {
QuadratureEncoder::QuadratureEncoder(const MujocoWorld& world, Config config) :
    config{std::move(config)}, position_address{world.joint_qpos(this->config.joint)} { }

void QuadratureEncoder::sample(MujocoWorld& world, const Clock& /*clock*/) {
    const std::span<const mjtNum> positions(world.data()->qpos, static_cast<std::size_t>(world.model()->nq));
    const double                  angle = at(positions, static_cast<std::size_t>(this->position_address));

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

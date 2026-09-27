/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>
#include <span>
#include <stdexcept>
#include <utility>

#include "micras/sim/devices/wall_sensors.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Angle, in half angles, the cone is traced out to.
 */
constexpr double cone_extent{2.5};

/**
 * @brief Relative intensity of an emitter or sensitivity of a receiver off its axis.
 *
 * @param angle Angle off the axis.
 * @param half_angle Angle at which it halves.
 * @return Factor between 0 and 1.
 */
double lobe(double angle, double half_angle) {
    const double ratio = angle / half_angle;
    return std::exp2(-ratio * ratio);
}

/**
 * @brief Lay out a cone of rays: one on the axis and rings around it.
 *
 * @param count Number of rays, one plus a multiple of the rings.
 * @param half_angle Emitter half angle.
 * @param intensity Radiant intensity on the axis, in W/sr.
 * @return The rays, each carrying the flux of its patch of the cone.
 */
std::vector<std::array<double, 4>> cone(int count, double half_angle, double intensity) {
    constexpr int                      rings = 3;
    const int                          per_ring = std::max(1, (count - 1) / rings);
    const double                       step = cone_extent * half_angle / (rings + 0.5);
    std::vector<std::array<double, 4>> rays;

    const double center_solid_angle = 2.0 * std::numbers::pi * (1.0 - std::cos(step / 2));
    rays.push_back({1.0, 0.0, 0.0, intensity * center_solid_angle});

    for (int ring = 1; ring <= rings; ring++) {
        const double angle = ring * step;
        const double solid_angle =
            2.0 * std::numbers::pi * (std::cos(angle - step / 2) - std::cos(angle + step / 2)) / per_ring;

        for (int index = 0; index < per_ring; index++) {
            const double around = 2.0 * std::numbers::pi * (index + 0.5 * (ring % 2)) / per_ring;
            rays.push_back(
                {std::cos(angle), std::sin(angle) * std::cos(around), std::sin(angle) * std::sin(around),
                 intensity * lobe(angle, half_angle) * solid_angle}
            );
        }
    }

    return rays;
}
}  // namespace

WallSensors::WallSensors(const MujocoWorld& world, Config config, const NoiseConfig& noise) :
    config{std::move(config)}, noise{noise, this->config.name} {
    const WallSensorsDescription& optics = this->config.description;

    for (const WallSensorDescription& sensor : optics.sensors) {
        this->emitter_sites.push_back(world.require_id(mjOBJ_SITE, sensor.name + "_emitter"));
        this->receiver_sites.push_back(world.require_id(mjOBJ_SITE, sensor.name + "_receiver"));
    }

    for (const auto& ray : cone(optics.rays, optics.emitter_half_angle, optics.emitter_intensity)) {
        this->rays.push_back({{ray[0], ray[1], ray[2]}, ray[3]});
    }

    std::set<int> group_ids;

    for (const WallSensorDescription& sensor : optics.sensors) {
        group_ids.insert(sensor.group);
    }

    if (group_ids.size() != this->groups.size()) {
        throw std::runtime_error(this->config.name + ": the emitters must fire in two groups");
    }

    std::ranges::copy(group_ids, this->groups.begin());
    std::ranges::fill(this->geom_groups, 1);
    this->geom_groups.at(MujocoWorld::unseen_group) = 0;
    this->counts.assign(2 * optics.sensors.size(), 0);
    this->intensities.assign(optics.sensors.size(), 0.0);
    this->directions.resize(3 * this->rays.size());
    this->distances.resize(this->rays.size());
    this->normals.resize(3 * this->rays.size());
    this->geoms.resize(this->rays.size());
}

double WallSensors::irradiance(MujocoWorld& world, std::size_t emitter, std::size_t receiver) {
    const mjModel*                model = world.model();
    mjData*                       data = world.data();
    const WallSensorsDescription& optics = this->config.description;
    const std::span<const mjtNum> site_positions(data->site_xpos, static_cast<std::size_t>(3 * model->nsite));
    const std::span<const mjtNum> site_frames(data->site_xmat, static_cast<std::size_t>(9 * model->nsite));
    const auto                    emitter_site = static_cast<std::size_t>(this->emitter_sites.at(emitter));
    const auto                    receiver_site = static_cast<std::size_t>(this->receiver_sites.at(receiver));
    const std::span<const mjtNum> origin = site_positions.subspan(3 * emitter_site, 3);
    const std::span<const mjtNum> frame = site_frames.subspan(9 * emitter_site, 9);
    const std::span<const mjtNum> receiver_position = site_positions.subspan(3 * receiver_site, 3);
    const std::span<const mjtNum> receiver_frame = site_frames.subspan(9 * receiver_site, 9);

    for (std::size_t ray = 0; ray < this->rays.size(); ray++) {
        const std::array<double, 3>& local = this->rays[ray].direction;

        for (std::size_t axis = 0; axis < 3; axis++) {
            this->directions[3 * ray + axis] =
                frame[3 * axis] * local[0] + frame[3 * axis + 1] * local[1] + frame[3 * axis + 2] * local[2];
        }
    }

    mj_multiRay(
        model, data, origin.data(), this->directions.data(), this->geom_groups.data(), true, -1, this->geoms.data(),
        this->distances.data(), this->normals.data(), static_cast<int>(this->rays.size()), cutoff
    );

    double total = 0.0;

    for (std::size_t ray = 0; ray < this->rays.size(); ray++) {
        const double distance = this->distances[ray];

        if (distance < 0.0 or this->geoms[ray] < 0) {
            continue;
        }

        std::array<double, 3> to_receiver{};
        double                length = 0.0;

        for (std::size_t axis = 0; axis < 3; axis++) {
            const double hit = origin[axis] + distance * this->directions[3 * ray + axis];
            to_receiver.at(axis) = receiver_position[axis] - hit;
            length += to_receiver.at(axis) * to_receiver.at(axis);
        }

        length = std::sqrt(length);

        if (length <= 0.0) {
            continue;
        }

        double emission = 0.0;
        double incidence = 0.0;

        for (std::size_t axis = 0; axis < 3; axis++) {
            emission += this->normals[3 * ray + axis] * to_receiver.at(axis) / length;
            incidence -= receiver_frame[3 * axis] * to_receiver.at(axis) / length;
        }

        if (emission <= 0.0 or incidence <= 0.0) {
            continue;
        }

        const double off_axis = std::acos(std::min(1.0, incidence));
        total += this->config.reflectance(this->geoms[ray]) * this->rays[ray].flux * emission * incidence *
                 lobe(off_axis, optics.receiver_half_angle) / (std::numbers::pi * length * length);
    }

    return total;
}

void WallSensors::sample(MujocoWorld& world, const Clock& clock) {
    if (clock.tick_count() % this->config.scan_ticks != 0) {
        return;
    }

    const WallSensorsDescription& optics = this->config.description;
    const std::size_t             sensors = optics.sensors.size();
    const std::size_t             scan = this->scans % this->groups.size();
    const int                     lit = this->groups.at(scan);

    for (std::size_t receiver = 0; receiver < sensors; receiver++) {
        double signal = 0.0;

        for (std::size_t emitter = 0; emitter < sensors; emitter++) {
            if (optics.sensors.at(emitter).group == lit and this->config.emitter_duty(emitter) > 0.0F) {
                signal += this->irradiance(world, emitter, receiver);
            }
        }

        const double current = optics.receiver_responsivity * optics.sensors.at(receiver).gain *
                               (optics.settle_fraction * signal + optics.ambient_irradiance);
        const double voltage = std::min(current * optics.load_resistance, optics.saturation_voltage);
        const double reading =
            voltage / optics.adc_reference * optics.adc_max_counts + this->noise.gaussian(optics.adc_noise_counts);
        const auto counts = static_cast<uint32_t>(std::clamp(std::round(reading), 0.0, optics.adc_max_counts));

        this->counts.at(scan * sensors + receiver) = counts;
        this->config.write(scan * sensors + receiver, counts);
    }

    this->scans++;

    if (scan + 1 == this->groups.size()) {
        for (std::size_t sensor = 0; sensor < sensors; sensor++) {
            const std::size_t own = optics.sensors.at(sensor).group == this->groups[0] ? 0 : 1;
            const std::size_t other = 1 - own;
            this->intensities.at(sensor) = (static_cast<double>(this->counts.at(own * sensors + sensor)) -
                                            static_cast<double>(this->counts.at(other * sensors + sensor))) /
                                           optics.adc_max_counts;
        }

        this->config.finish_sequence();
    }
}

std::vector<std::string> WallSensors::columns() const {
    std::vector<std::string> names;
    names.reserve(this->config.description.sensors.size());

    for (const WallSensorDescription& sensor : this->config.description.sensors) {
        names.push_back(this->config.name + "_" + sensor.name);
    }

    return names;
}

void WallSensors::append(std::vector<CsvCell>& row) const {
    for (const double intensity : this->intensities) {
        row.emplace_back(intensity);
    }
}
}  // namespace micras::sim

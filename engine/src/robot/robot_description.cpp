/**
 * @file
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <toml++/toml.hpp>

#include "micras/sim/core/text_file.hpp"
#include "micras/sim/robot/robot_description.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Radians per degree.
 */
constexpr double radians_per_degree{std::numbers::pi / 180.0};

/**
 * @brief One table of the file, read key by key, which refuses keys nobody read.
 */
class Section {
public:
    /**
     * @brief Wrap a table.
     *
     * @param table Table to read.
     * @param path Dotted path of the table, for messages.
     */
    Section(const toml::table& table, std::string path) : table{&table}, path{std::move(path)} { }

    /**
     * @brief Read a number.
     *
     * @param key Key in this table.
     * @return The number.
     */
    double number(std::string_view key) {
        const toml::node& node = this->value(key);

        if (const auto number = node.value<double>(); number.has_value()) {
            return *number;
        }

        throw this->error(key, "a number");
    }

    /**
     * @brief Read an angle given in degrees.
     *
     * @param key Key in this table, which should end in _deg.
     * @return The angle in radians.
     */
    double degrees(std::string_view key) { return this->number(key) * radians_per_degree; }

    /**
     * @brief Read a whole number.
     *
     * @param key Key in this table.
     * @return The number.
     */
    int64_t integer(std::string_view key) {
        const toml::node& node = this->value(key);

        if (const auto number = node.value<int64_t>(); number.has_value() and node.is_integer()) {
            return *number;
        }

        throw this->error(key, "a whole number");
    }

    /**
     * @brief Read a text.
     *
     * @param key Key in this table.
     * @return The text.
     */
    std::string text(std::string_view key) {
        const toml::node& node = this->value(key);

        if (const auto text = node.value<std::string>(); text.has_value()) {
            return *text;
        }

        throw this->error(key, "a text");
    }

    /**
     * @brief Read three numbers.
     *
     * @param key Key in this table.
     * @return The numbers.
     */
    Vector3 vector3(std::string_view key) {
        const std::vector<double> numbers = this->numbers(key, 3);
        return {numbers.at(0), numbers.at(1), numbers.at(2)};
    }

    /**
     * @brief Read a list of points in the plane.
     *
     * @param key Key in this table.
     * @return The points.
     */
    std::vector<std::array<double, 2>> points(std::string_view key) {
        const toml::array* array = this->value(key).as_array();

        if (array == nullptr or array->empty()) {
            throw this->error(key, "a list of [x, y] points");
        }

        std::vector<std::array<double, 2>> points;

        for (const toml::node& point : *array) {
            const toml::array* pair = point.as_array();

            if (pair == nullptr or pair->size() != 2 or not pair->at(0).is_number() or not pair->at(1).is_number()) {
                throw this->error(key, "a list of [x, y] points");
            }

            const std::optional<double> x = pair->at(0).value<double>();
            const std::optional<double> y = pair->at(1).value<double>();

            if (not x.has_value() or not y.has_value()) {
                throw this->error(key, "a list of [x, y] points");
            }

            points.push_back({*x, *y});
        }

        return points;
    }

    /**
     * @brief Open a sub-table.
     *
     * @param key Key in this table.
     * @return The sub-table.
     */
    Section section(std::string_view key) {
        this->used.insert(std::string{key});
        const toml::table* table = this->table->get_as<toml::table>(key);

        if (table == nullptr) {
            throw std::runtime_error(std::format("{}: [{}] is missing", this->where(), this->child(key)));
        }

        return {*table, this->child(key)};
    }

    /**
     * @brief Open every table of an array of tables.
     *
     * @param key Key in this table.
     * @return The tables, possibly none.
     */
    std::vector<Section> sections(std::string_view key) {
        this->used.insert(std::string{key});
        std::vector<Section>     sections;
        const toml::array* const array = this->table->get_as<toml::array>(key);

        if (array == nullptr) {
            return sections;
        }

        for (std::size_t i = 0; i < array->size(); i++) {
            const toml::table* table = array->at(i).as_table();

            if (table == nullptr) {
                throw std::runtime_error(std::format("{}: {} must be an array of tables", this->where(), key));
            }

            sections.emplace_back(*table, std::format("{}[{}]", this->child(key), i));
        }

        return sections;
    }

    /**
     * @brief Refuse any key that was not read.
     */
    void finish() const {
        for (const auto& [key, node] : *this->table) {
            if (not this->used.contains(std::string{key.str()})) {
                throw std::runtime_error(std::format("{}: unknown key {}", this->where(), this->child(key.str())));
            }
        }
    }

private:
    /**
     * @brief Find a value, unwrapping { value, source }.
     *
     * @param key Key in this table.
     * @return The value node.
     */
    const toml::node& value(std::string_view key) {
        this->used.insert(std::string{key});
        const toml::node* node = this->table->get(key);

        if (node == nullptr) {
            throw std::runtime_error(std::format("{}: {} is missing", this->where(), this->child(key)));
        }

        if (const toml::table* sourced = node->as_table(); sourced != nullptr) {
            const toml::node* value = sourced->get("value");
            const auto* const source = sourced->get_as<std::string>("source");

            if (value == nullptr or source == nullptr or sourced->size() != 2) {
                throw this->error(key, "a value or a { value = ..., source = \"...\" } table");
            }

            return *value;
        }

        return *node;
    }

    /**
     * @brief Read a fixed number of numbers.
     *
     * @param key Key in this table.
     * @param count Number of numbers.
     * @return The numbers.
     */
    std::vector<double> numbers(std::string_view key, std::size_t count) {
        const toml::array* array = this->value(key).as_array();

        if (array == nullptr or array->size() != count) {
            throw this->error(key, std::format("{} numbers", count));
        }

        std::vector<double> numbers;

        for (const toml::node& element : *array) {
            const std::optional<double> number = element.value<double>();

            if (not element.is_number() or not number.has_value()) {
                throw this->error(key, std::format("{} numbers", count));
            }

            numbers.push_back(*number);
        }

        return numbers;
    }

    /**
     * @brief Build an error for a value of the wrong kind.
     *
     * @param key Key in this table.
     * @param expected What was expected.
     * @return The error.
     */
    std::runtime_error error(std::string_view key, std::string_view expected) const {
        return std::runtime_error(std::format("{}: {} must be {}", this->where(), this->child(key), expected));
    }

    /**
     * @brief Get the dotted path of a key of this table.
     *
     * @param key Key in this table.
     * @return The path.
     */
    std::string child(std::string_view key) const {
        return this->path.empty() ? std::string{key} : this->path + "." + std::string{key};
    }

    /**
     * @brief Name the file, for messages.
     *
     * @return The origin of the table.
     */
    std::string where() const {
        const auto& source = this->table->source();
        return source.path == nullptr ? std::string{"robot.toml"} : std::string{*source.path};
    }

    const toml::table*    table;
    std::string           path;
    std::set<std::string> used;
};
}  // namespace

/**
 * @brief Read the chassis table.
 *
 * @param section The [chassis] table.
 * @return The chassis.
 */
static ChassisDescription read_chassis(Section section) {
    ChassisDescription chassis{
        .mass = section.number("mass"),
        .center_of_mass = section.vector3("center_of_mass"),
        .inertia = section.vector3("inertia"),
        .outline = section.points("outline"),
        .board_bottom = section.number("board_bottom"),
        .board_top = section.number("board_top"),
        .friction = section.number("friction"),
        .boxes = {},
        .cylinders = {},
        .skids = {},
    };

    for (Section part : section.sections("boxes")) {
        chassis.boxes.push_back({.name = part.text("name"), .min = part.vector3("min"), .max = part.vector3("max")});
        part.finish();
    }

    for (Section part : section.sections("skids")) {
        chassis.skids.push_back(
            {.name = part.text("name"), .center = part.vector3("center"), .radius = part.number("radius")}
        );
        part.finish();
    }

    for (Section part : section.sections("cylinders")) {
        chassis.cylinders.push_back(
            {.name = part.text("name"),
             .base = part.vector3("base"),
             .radius = part.number("radius"),
             .height = part.number("height")}
        );
        part.finish();
    }

    section.finish();
    return chassis;
}

/**
 * @brief Read the wall sensors table.
 *
 * @param section The [wall_sensors] table.
 * @return The wall sensors.
 */
static WallSensorsDescription read_wall_sensors(Section section) {
    WallSensorsDescription sensors{
        .emitter_half_angle = section.degrees("emitter_half_angle_deg"),
        .emitter_intensity = section.number("emitter_intensity"),
        .receiver_half_angle = section.degrees("receiver_half_angle_deg"),
        .receiver_responsivity = section.number("receiver_responsivity"),
        .load_resistance = section.number("load_resistance"),
        .saturation_voltage = section.number("saturation_voltage"),
        .adc_reference = section.number("adc_reference"),
        .adc_max_counts = section.number("adc_max_counts"),
        .adc_noise_counts = section.number("adc_noise_counts"),
        .settle_fraction = section.number("settle_fraction"),
        .ambient_irradiance = section.number("ambient_irradiance"),
        .rays = static_cast<int>(section.integer("rays")),
        .sensors = {},
    };

    for (Section sensor : section.sections("sensors")) {
        sensors.sensors.push_back(
            {.name = sensor.text("name"),
             .emitter = sensor.vector3("emitter"),
             .receiver = sensor.vector3("receiver"),
             .yaw = sensor.degrees("yaw_deg"),
             .group = static_cast<int>(sensor.integer("group")),
             .gain = sensor.number("gain")}
        );
        sensor.finish();
    }

    section.finish();
    return sensors;
}

/**
 * @brief Read everything but the chassis and the wall sensors.
 *
 * @param root The whole file.
 * @param robot Description to fill.
 */
static void read_components(Section& root, RobotDescription& robot) {
    Section integration = root.section("integration");
    robot.integration = {.timestep = integration.number("timestep")};
    integration.finish();

    Section wheels = root.section("wheels");
    robot.wheels = {
        .radius = wheels.number("radius"),
        .width = wheels.number("width"),
        .track = wheels.number("track"),
        .mass = wheels.number("mass"),
        .spin_inertia = wheels.number("spin_inertia"),
        .friction = wheels.number("friction"),
        .torsional_friction = wheels.number("torsional_friction"),
        .contact_time_constant = wheels.number("contact_time_constant"),
        .contact_damping_ratio = wheels.number("contact_damping_ratio"),
    };
    wheels.finish();

    Section drive = root.section("drive");
    robot.drive = {
        .winding_resistance = drive.number("winding_resistance"),
        .bridge_resistance = drive.number("bridge_resistance"),
        .torque_constant = drive.number("torque_constant"),
        .speed_constant = drive.number("speed_constant"),
        .no_load_current = drive.number("no_load_current"),
        .rotor_inertia = drive.number("rotor_inertia"),
        .gear_ratio = drive.number("gear_ratio"),
        .gear_efficiency = drive.number("gear_efficiency"),
        .supply_voltage = drive.number("supply_voltage"),
    };
    drive.finish();

    Section encoders = root.section("encoders");
    robot.encoders = {.counts_per_revolution = static_cast<uint32_t>(encoders.integer("counts_per_revolution"))};
    encoders.finish();

    Section imu = root.section("imu");
    robot.imu = {
        .position = imu.vector3("position"),
        .axes = {imu.vector3("chip_x"), imu.vector3("chip_y")},
        .gyro_noise_density = imu.number("gyro_noise_density"),
        .gyro_bias = imu.number("gyro_bias"),
        .gyro_scale_error = imu.number("gyro_scale_error"),
        .gyro_resolution = imu.number("gyro_resolution"),
        .accel_noise_density = imu.number("accel_noise_density"),
        .accel_bias = imu.number("accel_bias"),
        .accel_resolution = imu.number("accel_resolution"),
        .bandwidth = imu.number("bandwidth"),
        .output_rate = imu.number("output_rate"),
        .clock_error = imu.number("clock_error"),
    };
    imu.finish();

    Section fan = root.section("fan");
    robot.fan = {
        .position = fan.vector3("position"),
        .max_downforce = fan.number("max_downforce"),
        .nominal_voltage = fan.number("nominal_voltage"),
        .time_constant = fan.number("time_constant"),
    };
    fan.finish();

    Section battery = root.section("battery");
    robot.battery = {
        .cells = static_cast<int>(battery.integer("cells")),
        .cell_voltage = battery.number("cell_voltage"),
    };
    battery.finish();

    Section link = root.section("link");
    robot.link = {.baud_rate = static_cast<uint32_t>(link.integer("baud_rate"))};
    link.finish();
}

RobotDescription RobotDescription::load(const std::filesystem::path& path) {
    return parse(read_text_file(path, "robot description"), path.string());
}

RobotDescription RobotDescription::parse(std::string_view text, const std::string& origin) {
    toml::table table;

    try {
        table = toml::parse(text, origin);
    } catch (const toml::parse_error& error) {
        throw std::runtime_error(std::format("{}: {}", origin, error.description()));
    }

    RobotDescription robot;
    Section          root{table, ""};

    const int64_t schema = root.integer("schema");

    if (schema != schema_version) {
        throw std::runtime_error(
            std::format("{}: schema {} is not the {} this build reads", origin, schema, schema_version)
        );
    }

    robot.name = root.text("name");
    read_components(root, robot);
    robot.chassis = read_chassis(root.section("chassis"));
    robot.wall_sensors = read_wall_sensors(root.section("wall_sensors"));
    root.finish();

    if (robot.chassis.outline.size() < 3) {
        throw std::runtime_error(origin + ": chassis.outline needs at least three points");
    }

    return robot;
}
}  // namespace micras::sim

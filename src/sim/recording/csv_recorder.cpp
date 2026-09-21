/**
 * @file
 */

#include <cctype>
#include <stdexcept>

#include "micras/sim/recording/csv_recorder.hpp"

namespace micras::sim {
CsvRecorder::CsvRecorder(
    const MujocoWorld& world, const ProxyState& proxy_state, const Telemetry* telemetry,
    const std::filesystem::path& path
) :
    telemetry{telemetry}, proxy_state{proxy_state}, ground_truth{world}, writer{path} { }

std::vector<std::string> CsvRecorder::proxy_columns() {
    std::vector<std::string> columns{"proxy_left_command",  "proxy_right_command", "proxy_fan_speed",
                                     "proxy_encoder_left",  "proxy_encoder_right", "proxy_gyro_x",
                                     "proxy_gyro_y",        "proxy_gyro_z",        "proxy_accel_x",
                                     "proxy_accel_y",       "proxy_accel_z",       "proxy_wall_adc_0",
                                     "proxy_wall_adc_1",    "proxy_wall_adc_2",    "proxy_wall_adc_3",
                                     "proxy_button_pressed"};

    for (std::size_t i = 0; i < InterfaceInput::dip_switch_count; i++) {
        columns.push_back("proxy_dip_" + std::to_string(i));
    }

    return columns;
}

std::string CsvRecorder::column_name(const std::string& name) {
    std::string column;
    column.reserve(name.size());

    for (const char character : name) {
        if (std::isalnum(static_cast<unsigned char>(character)) != 0) {
            column.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
        } else if (not column.empty() and column.back() != '_') {
            column.push_back('_');
        }
    }

    while (not column.empty() and column.back() == '_') {
        column.pop_back();
    }

    return column;
}

void CsvRecorder::write_header() {
    if (this->telemetry != nullptr and not this->telemetry->has_variable_map()) {
        const std::string reason = this->telemetry->map_failure_count() > 0 ?
                                       "the firmware answered the variable map request with " +
                                           std::to_string(this->telemetry->map_failure_count()) +
                                           " malformed response(s)" :
                                       "the firmware did not answer the variable map request on tick 0";
        throw std::runtime_error(reason + "; the CSV header would silently lose every pool column");
    }

    for (const auto& variable : this->pool_variables()) {
        const std::string              base = column_name(variable.name);
        const std::vector<std::string> suffixes = VariableMap::type_column_suffixes(variable.type);

        if (suffixes.empty()) {
            this->pool_columns.push_back({variable.id, 0, base});
            continue;
        }

        for (std::size_t component = 0; component < suffixes.size(); component++) {
            this->pool_columns.push_back({variable.id, component, base + '_' + suffixes[component]});
        }
    }

    std::vector<std::string> columns = GroundTruth::columns();

    for (const auto& column : this->pool_columns) {
        columns.push_back(column.name);
    }

    for (const auto& column : proxy_columns()) {
        columns.push_back(column);
    }

    this->writer.write_header(columns);
    this->header_written = true;
}

void CsvRecorder::on_after_tick(const Simulation& simulation) {
    this->sample(simulation.tick());
}

std::vector<PoolVariable> CsvRecorder::pool_variables() const {
    return this->telemetry == nullptr ? std::vector<PoolVariable>{} :
                                        this->telemetry->variable_map().monitoring_variables();
}

void CsvRecorder::sample(uint64_t tick) {
    if (not this->header_written) {
        this->write_header();
    }

    std::vector<CsvCell> cells = this->ground_truth.sample(tick);

    if (cells.size() != GroundTruth::columns().size()) {
        throw std::logic_error("the ground truth sampled a different number of cells than it has columns");
    }

    for (const auto& column : this->pool_columns) {
        cells.emplace_back(this->telemetry->value(column.id, column.component));
    }

    const Actuators& actuators = this->proxy_state.actuators;
    const Sensors&   sensors = this->proxy_state.sensors;

    cells.emplace_back(actuators.left_command);
    cells.emplace_back(actuators.right_command);
    cells.emplace_back(actuators.fan_speed);

    for (const float position : sensors.encoder_positions) {
        cells.emplace_back(position);
    }

    for (const float rate : sensors.angular_velocity) {
        cells.emplace_back(rate);
    }

    for (const float acceleration : sensors.linear_acceleration) {
        cells.emplace_back(acceleration);
    }

    for (const float reading : sensors.wall_adc_readings) {
        cells.emplace_back(reading);
    }

    const InterfaceInput& input = this->proxy_state.interface_input;

    cells.emplace_back(static_cast<int64_t>(input.button_pressed));

    for (const bool state : input.dip_switches) {
        cells.emplace_back(static_cast<int64_t>(state));
    }

    this->writer.write_row(cells);
}
}  // namespace micras::sim

/**
 * @file
 */

#include <algorithm>
#include <cstring>
#include <limits>
#include <string_view>

#include "micras/comm/packet.hpp"
#include "micras/sim/telemetry/telemetry.hpp"

namespace micras::sim {
namespace {
constexpr double not_a_number = std::numeric_limits<double>::quiet_NaN();

/**
 * @brief Read a little-endian unsigned 16 bit integer.
 *
 * @param data Source buffer.
 * @param offset Offset of the first byte.
 * @return Decoded value.
 */
uint16_t read_u16_le(const std::vector<uint8_t>& data, std::size_t offset) {
    return static_cast<uint16_t>(data.at(offset) | (data.at(offset + 1) << 8));
}

/**
 * @brief Read a little-endian 32 bit float.
 *
 * @param payload Source buffer.
 * @param offset Offset of the first byte.
 * @return Decoded value.
 */
double read_f32_le(const std::vector<uint8_t>& payload, std::size_t offset) {
    float value{};
    std::memcpy(&value, &payload.at(offset), sizeof(value));
    return value;
}

/**
 * @brief Strip the namespace qualification off a firmware type name.
 *
 * @param type Type name as reported in the variable map response.
 * @return Last component of the qualified name.
 */
std::string_view bare_type(const std::string& type) {
    const std::size_t separator = type.rfind("::");
    return separator == std::string::npos ? std::string_view{type} : std::string_view{type}.substr(separator + 2);
}

/**
 * @brief Read one length-prefixed string out of the map payload.
 *
 * @param payload Map response payload.
 * @param offset Offset of the length byte, advanced past the string.
 * @return The string, or nullopt if the payload ends early.
 */
std::optional<std::string> read_string(const std::vector<uint8_t>& payload, std::size_t& offset) {
    if (offset >= payload.size()) {
        return std::nullopt;
    }

    const std::size_t length = payload.at(offset++);

    if (offset + length > payload.size()) {
        return std::nullopt;
    }

    const auto  first = payload.begin() + static_cast<std::ptrdiff_t>(offset);
    std::string text(first, first + static_cast<std::ptrdiff_t>(length));
    offset += length;
    return text;
}

/**
 * @brief Read one pool variable entry out of the map payload.
 *
 * @param payload Map response payload.
 * @param offset Offset of the entry, advanced past it.
 * @return The variable, or nullopt if the payload ends early.
 */
std::optional<PoolVariable> read_variable(const std::vector<uint8_t>& payload, std::size_t& offset) {
    if (offset + 3 > payload.size()) {
        return std::nullopt;
    }

    PoolVariable variable{};
    variable.id = read_u16_le(payload, offset);
    offset += 2;

    const std::optional<std::string> name = read_string(payload, offset);

    if (not name.has_value()) {
        return std::nullopt;
    }

    const std::optional<std::string> type = read_string(payload, offset);

    if (not type.has_value() or offset >= payload.size()) {
        return std::nullopt;
    }

    variable.name = name.value();
    variable.type = type.value();
    variable.read_only = payload.at(offset++) != 0;
    return variable;
}
}  // namespace

std::optional<VariableMap> VariableMap::parse(const std::vector<uint8_t>& payload) {
    if (payload.size() < 2) {
        return std::nullopt;
    }

    VariableMap map;

    std::vector<PoolVariable> variables;
    const uint16_t            count = read_u16_le(payload, 0);
    std::size_t               offset = 2;

    for (uint16_t i = 0; i < count; i++) {
        const std::optional<PoolVariable> variable = read_variable(payload, offset);

        if (not variable.has_value()) {
            return std::nullopt;
        }

        variables.push_back(*variable);
    }

    std::sort(variables.begin(), variables.end(), [](const PoolVariable& left, const PoolVariable& right) {
        return left.id < right.id;
    });

    for (const auto& variable : variables) {
        map.types[variable.id] = variable.type;
        map.ids_by_name[variable.name] = variable.id;

        if (variable.read_only) {
            map.monitoring.push_back(variable);
        }
    }

    return map;
}

std::vector<std::string> VariableMap::type_column_suffixes(const std::string& type) {
    const std::string_view bare = bare_type(type);

    if (bare == "GridPose") {
        return {"x", "y", "side"};
    }

    if (bare == "State") {
        return {"x", "y", "orientation", "v_linear", "v_angular"};
    }

    return {};
}

std::optional<uint16_t> VariableMap::find_id(const std::string& name) const {
    const auto entry = this->ids_by_name.find(name);
    return entry == this->ids_by_name.end() ? std::nullopt : std::optional<uint16_t>{entry->second};
}

std::string VariableMap::type_of(uint16_t id) const {
    const auto entry = this->types.find(id);
    return entry == this->types.end() ? std::string{} : entry->second;
}

std::vector<double> PoolValues::decode(const std::string& type, const std::vector<uint8_t>& payload) {
    const std::string_view bare = bare_type(type);

    if (bare == "float" and payload.size() == sizeof(float)) {
        return {read_f32_le(payload, 0)};
    }

    if (bare == "double" and payload.size() == sizeof(double)) {
        double value{};
        std::memcpy(&value, payload.data(), sizeof(value));
        return {value};
    }

    if (bare == "bool" and payload.size() == sizeof(bool)) {
        return {payload.front() != 0 ? 1.0 : 0.0};
    }

    if (bare == "signed char" and payload.size() == 1) {
        return {static_cast<double>(static_cast<int8_t>(payload.front()))};
    }

    if ((bare == "unsigned char" or bare == "char") and payload.size() == 1) {
        return {static_cast<double>(payload.front())};
    }

    if (bare == "GridPose" and payload.size() == 3) {
        return {static_cast<double>(payload[0]), static_cast<double>(payload[1]), static_cast<double>(payload[2])};
    }

    if (bare == "State" and payload.size() == 5 * sizeof(float)) {
        return {
            read_f32_le(payload, 0), read_f32_le(payload, 4), read_f32_le(payload, 8), read_f32_le(payload, 12),
            read_f32_le(payload, 16)
        };
    }

    const std::size_t columns = std::max<std::size_t>(1, VariableMap::type_column_suffixes(type).size());
    return std::vector<double>(columns, not_a_number);
}

void PoolValues::store(uint16_t id, const std::string& type, const std::vector<uint8_t>& payload) {
    this->values[id] = decode(type, payload);
}

double PoolValues::get(uint16_t id, std::size_t component) const {
    const auto entry = this->values.find(id);

    if (entry == this->values.end() or component >= entry->second.size()) {
        return not_a_number;
    }

    return entry->second[component];
}

Telemetry::Telemetry(SerialBus& serial) : serial{serial} {
    this->serial.add_listener(*this);
}

Telemetry::~Telemetry() {
    this->serial.remove_listener(*this);
}

void Telemetry::on_firmware_bytes(std::span<const uint8_t> bytes) {
    this->framer.push(bytes);
}

void Telemetry::request_variable_map() {
    this->serial.queue_for_firmware(comm::Packet(comm::Packet::MessageType::SERIAL_VARIABLE_MAP_REQUEST).serialize());
}

void Telemetry::send_variable(uint16_t id, const std::vector<uint8_t>& payload) {
    this->serial.queue_for_firmware(comm::Packet(comm::Packet::MessageType::SERIAL_VARIABLE, id, payload).serialize());
}

void Telemetry::on_after_tick(const Simulation& /*simulation*/) {
    this->poll();
}

void Telemetry::poll() {
    for (const auto& raw : this->framer.drain()) {
        this->consume(raw);
    }
}

double Telemetry::value_of(const std::string& name) const {
    const std::optional<uint16_t> id = this->map.find_id(name);
    return id.has_value() ? this->values.get(*id) : not_a_number;
}

void Telemetry::consume(const std::vector<uint8_t>& raw) {
    const comm::Packet packet(raw);

    switch (packet.get_type()) {
        case comm::Packet::MessageType::SERIAL_VARIABLE_MAP_RESPONSE: {
            const std::optional<VariableMap> parsed = VariableMap::parse(packet.get_payload());

            if (parsed.has_value()) {
                this->map = *parsed;
                this->map_received = true;
            } else {
                this->map_failures++;
            }

            break;
        }

        case comm::Packet::MessageType::SERIAL_VARIABLE:
            this->values.store(packet.get_id(), this->map.type_of(packet.get_id()), packet.get_payload());
            break;

        default:
            break;
    }
}
}  // namespace micras::sim

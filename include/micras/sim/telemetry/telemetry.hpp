/**
 * @file
 *
 * @brief Decoder for the firmware serial protocol over the in-process channel.
 */

#ifndef MICRAS_SIM_TELEMETRY_TELEMETRY_HPP
#define MICRAS_SIM_TELEMETRY_TELEMETRY_HPP

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/telemetry/packet_framer.hpp"

namespace micras::sim {
/**
 * @brief One entry of the firmware serial variable pool.
 */
struct PoolVariable {
    uint16_t    id;
    std::string name;
    std::string type;
    bool        read_only;
};

/**
 * @brief Variable map reported by the firmware, indexed by id and by name.
 */
class VariableMap {
public:
    /**
     * @brief Parse a variable map response payload.
     *
     * @param payload Unescaped payload of the map response.
     * @return The decoded map, or nullopt if the payload is truncated.
     */
    static std::optional<VariableMap> parse(const std::vector<uint8_t>& payload);

    /**
     * @brief Get the column suffixes a custom serializable type expands into.
     *
     * @note The firmware reports types through core::type_name, so they arrive
     *       fully qualified ("micras::nav::GridPose"); only the last component
     *       is matched. Scalars expand into a single unnamed column.
     *
     * @param type Type name as reported in the variable map response.
     * @return Suffixes in serialization order, empty for scalar types.
     */
    static std::vector<std::string> type_column_suffixes(const std::string& type);

    /**
     * @brief Get the read-only pool variables, ordered by id.
     *
     * @return Vector of monitoring variables.
     */
    const std::vector<PoolVariable>& monitoring_variables() const { return this->monitoring; }

    /**
     * @brief Look up a pool variable id by its firmware name.
     *
     * @param name Name as reported in the variable map response.
     * @return Id of the variable, or nullopt if the pool has no such name.
     */
    std::optional<uint16_t> find_id(const std::string& name) const;

    /**
     * @brief Get the type name of a variable.
     *
     * @param id Id of the variable in the pool.
     * @return Type name, or an empty string for unknown ids.
     */
    std::string type_of(uint16_t id) const;

private:
    /**
     * @brief Read-only variables, ordered by id.
     */
    std::vector<PoolVariable> monitoring;

    /**
     * @brief Type of every variable, by id.
     */
    std::unordered_map<uint16_t, std::string> types;

    /**
     * @brief Id of every variable, by name.
     */
    std::unordered_map<std::string, uint16_t> ids_by_name;
};

/**
 * @brief Latest decoded value of every pool variable, one entry per serialized field.
 */
class PoolValues {
public:
    /**
     * @brief Decode a payload according to the type reported in the variable map.
     *
     * @note The custom serializable layouts mirror nav::GridPose::serialize
     *       (three bytes: x, y, orientation as Side, RIGHT=0 UP=1 LEFT=2 DOWN=3)
     *       and nav::State::serialize (five little-endian floats: x, y,
     *       orientation, linear velocity, angular velocity). Unknown types
     *       decode to NaN rather than to a plausible looking wrong number.
     *
     * @param type Type name of the variable.
     * @param payload Little-endian serialized value.
     * @return One value per column of the type.
     */
    static std::vector<double> decode(const std::string& type, const std::vector<uint8_t>& payload);

    /**
     * @brief Store the decoded value of a variable.
     *
     * @param id Id of the variable in the pool.
     * @param type Type name of the variable.
     * @param payload Little-endian serialized value.
     */
    void store(uint16_t id, const std::string& type, const std::vector<uint8_t>& payload);

    /**
     * @brief Get the latest value decoded for a variable.
     *
     * @param id Id of the variable in the pool.
     * @param component Index of the field for custom serializable types.
     * @return Latest value, or NaN if nothing was decoded yet.
     */
    double get(uint16_t id, std::size_t component = 0) const;

private:
    /**
     * @brief Latest value of each variable, by id.
     */
    std::unordered_map<uint16_t, std::vector<double>> values;
};

/**
 * @brief Drains the firmware output and keeps the latest value of every pool variable.
 */
class Telemetry : public ISerialListener, public IRunListener {
public:
    /**
     * @brief Listen to a serial bus.
     *
     * @param serial Bus shared with the firmware.
     */
    explicit Telemetry(SerialBus& serial);

    Telemetry(const Telemetry&) = delete;
    Telemetry(Telemetry&&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;
    Telemetry& operator=(Telemetry&&) = delete;

    /**
     * @brief Stop listening to the bus.
     */
    ~Telemetry() override;

    /**
     * @brief Buffer bytes the firmware wrote, to be decoded on the next poll.
     *
     * @param bytes Bytes leaving the firmware.
     */
    void on_firmware_bytes(std::span<const uint8_t> bytes) override;

    /**
     * @brief Queue a variable map request for the firmware.
     *
     * @note The firmware clears its whole receive deque after framing a single
     *       packet, so the caller must guarantee that at most one packet is
     *       queued per tick; nothing here enforces it.
     */
    void request_variable_map();

    /**
     * @brief Queue a value for a bidirectional variable.
     *
     * @param id Id of the variable in the pool.
     * @param payload Little-endian serialized value.
     */
    void send_variable(uint16_t id, const std::vector<uint8_t>& payload);

    /**
     * @brief Decode every packet buffered since the last call.
     */
    void poll();

    /**
     * @brief Decode what the firmware wrote during the tick.
     *
     * @note Registered before the recorder so every column it feeds is already
     *       up to date when the row is written.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

    /**
     * @brief Check whether the variable map has been received.
     *
     * @return True once the firmware answered the map request.
     */
    bool has_variable_map() const { return this->map_received; }

    /**
     * @brief Get the variable map as last reported by the firmware.
     *
     * @return The variable map, empty before the response arrives.
     */
    const VariableMap& variable_map() const { return this->map; }

    /**
     * @brief Look up a pool variable id by its firmware name.
     *
     * @param name Name as reported in the variable map response.
     * @return Id of the variable, or nullopt if the pool has no such name.
     */
    std::optional<uint16_t> find_variable_id(const std::string& name) const { return this->map.find_id(name); }

    /**
     * @brief Get the latest value decoded for a variable.
     *
     * @param id Id of the variable in the pool.
     * @param component Index of the field for custom serializable types.
     * @return Latest value, or NaN if nothing was decoded yet.
     */
    double value(uint16_t id, std::size_t component = 0) const { return this->values.get(id, component); }

    /**
     * @brief Get the latest value of a scalar variable by name.
     *
     * @param name Firmware name of the variable.
     * @return Latest value, or NaN when the map is not in yet.
     */
    double value_of(const std::string& name) const;

    /**
     * @brief Get how many map responses were dropped as malformed.
     *
     * @return Number of unparsable variable map responses.
     */
    uint32_t map_failure_count() const { return this->map_failures; }

    /**
     * @brief Get how many times the receive buffer had to be truncated.
     *
     * @return Number of resynchronisations, always 0 on a healthy stream.
     */
    uint32_t resync_count() const { return this->framer.resync_count(); }

private:
    /**
     * @brief Handle one framed packet.
     *
     * @param raw Serialized packet, header to tail inclusive.
     */
    void consume(const std::vector<uint8_t>& raw);

    /**
     * @brief Bus the firmware writes to and reads from.
     *
     * @note Bound for the life of the recorder; it is fed by the caller that
     *       also owns the run.
     */
    SerialBus& serial;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Splits the firmware stream into packets.
     */
    PacketFramer framer;

    /**
     * @brief Variable map as last reported by the firmware.
     */
    VariableMap map;

    /**
     * @brief Latest value of every variable.
     */
    PoolValues values;

    /**
     * @brief Flag set once the variable map response was decoded.
     */
    bool map_received{false};

    /**
     * @brief Number of map responses that could not be parsed.
     */
    uint32_t map_failures{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_TELEMETRY_TELEMETRY_HPP

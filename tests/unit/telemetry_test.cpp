#include <cmath>
#include <cstring>

#include <gtest/gtest.h>

#include "micras/comm/packet.hpp"
#include "micras/sim/telemetry/telemetry.hpp"

namespace micras::sim {
namespace {
void append_string(std::vector<uint8_t>& payload, const std::string& text) {
    payload.push_back(static_cast<uint8_t>(text.size()));
    payload.insert(payload.end(), text.begin(), text.end());
}

void append_variable(
    std::vector<uint8_t>& payload, uint16_t id, const std::string& name, const std::string& type, bool read_only
) {
    payload.push_back(static_cast<uint8_t>(id & 0xFF));
    payload.push_back(static_cast<uint8_t>(id >> 8));
    append_string(payload, name);
    append_string(payload, type);
    payload.push_back(read_only ? 1 : 0);
}

std::vector<uint8_t> map_payload() {
    std::vector<uint8_t> payload{3, 0};
    append_variable(payload, 7, "Grid Pose", "micras::nav::GridPose", true);
    append_variable(payload, 2, "Explore", "bool", false);
    append_variable(payload, 5, "Linear Speed", "float", true);
    return payload;
}

std::vector<uint8_t> float_bytes(float value) {
    std::vector<uint8_t> bytes(sizeof(float));
    std::memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

TEST(VariableMap, OrdersMonitoringVariablesById) {
    const VariableMap map = VariableMap::parse(map_payload()).value_or(VariableMap{});

    const auto& monitoring = map.monitoring_variables();
    ASSERT_EQ(monitoring.size(), 2U);
    EXPECT_EQ(monitoring[0].id, 5);
    EXPECT_EQ(monitoring[0].name, "Linear Speed");
    EXPECT_EQ(monitoring[1].id, 7);
    EXPECT_EQ(monitoring[1].type, "micras::nav::GridPose");
}

TEST(VariableMap, ResolvesNamesAndTypes) {
    const VariableMap map = VariableMap::parse(map_payload()).value_or(VariableMap{});

    EXPECT_EQ(map.find_id("Explore"), std::optional<uint16_t>{2});
    EXPECT_EQ(map.find_id("Missing"), std::nullopt);
    EXPECT_EQ(map.type_of(7), "micras::nav::GridPose");
    EXPECT_EQ(map.type_of(99), "");
}

TEST(VariableMap, RejectsTruncatedPayload) {
    auto payload = map_payload();
    payload.pop_back();
    EXPECT_FALSE(VariableMap::parse(payload).has_value());
    EXPECT_FALSE(VariableMap::parse({1}).has_value());
}

TEST(VariableMap, ExpandsCustomSerializables) {
    EXPECT_EQ(VariableMap::type_column_suffixes("micras::nav::GridPose"), (std::vector<std::string>{"x", "y", "side"}));
    EXPECT_EQ(VariableMap::type_column_suffixes("micras::nav::State").size(), 5U);
    EXPECT_TRUE(VariableMap::type_column_suffixes("float").empty());
}

TEST(PoolValues, DecodesScalars) {
    EXPECT_FLOAT_EQ(static_cast<float>(PoolValues::decode("float", float_bytes(1.5F))[0]), 1.5F);
    EXPECT_EQ(PoolValues::decode("bool", {1}), (std::vector<double>{1.0}));
    EXPECT_EQ(PoolValues::decode("unsigned char", {200}), (std::vector<double>{200.0}));
}

TEST(PoolValues, DecodesGridPoseAndState) {
    EXPECT_EQ(PoolValues::decode("micras::nav::GridPose", {3, 4, 2}), (std::vector<double>{3.0, 4.0, 2.0}));

    std::vector<uint8_t> state;

    for (const float value : {1.0F, 2.0F, 3.0F, 4.0F, 5.0F}) {
        const auto bytes = float_bytes(value);
        state.insert(state.end(), bytes.begin(), bytes.end());
    }

    EXPECT_EQ(PoolValues::decode("micras::nav::State", state), (std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0}));
}

TEST(PoolValues, UnknownTypesDecodeToNan) {
    const auto decoded = PoolValues::decode("uint32_t", {1, 2, 3, 4});
    ASSERT_EQ(decoded.size(), 1U);
    EXPECT_TRUE(std::isnan(decoded[0]));

    const auto mismatched = PoolValues::decode("micras::nav::GridPose", {1});
    ASSERT_EQ(mismatched.size(), 3U);
    EXPECT_TRUE(std::isnan(mismatched[2]));
}

TEST(PoolValues, StoresLatestValue) {
    PoolValues values;
    EXPECT_TRUE(std::isnan(values.get(5)));

    values.store(5, "float", float_bytes(2.0F));
    values.store(5, "float", float_bytes(3.0F));
    EXPECT_DOUBLE_EQ(values.get(5), 3.0);
    EXPECT_TRUE(std::isnan(values.get(5, 1)));
}

std::vector<uint8_t> map_response() {
    return comm::Packet(comm::Packet::MessageType::SERIAL_VARIABLE_MAP_RESPONSE, map_payload()).serialize();
}

TEST(Telemetry, DecodesTheMapAndTheValuesTheFirmwareSends) {
    SerialBus bus;
    Telemetry telemetry(bus);

    EXPECT_FALSE(telemetry.has_variable_map());

    bus.send_from_firmware(map_response());
    telemetry.poll();

    ASSERT_TRUE(telemetry.has_variable_map());
    EXPECT_EQ(telemetry.variable_map().monitoring_variables().size(), 2U);
    EXPECT_TRUE(std::isnan(telemetry.value_of("Linear Speed")));

    bus.send_from_firmware(comm::Packet(comm::Packet::MessageType::SERIAL_VARIABLE, 5, float_bytes(1.25F)).serialize());
    telemetry.poll();

    EXPECT_DOUBLE_EQ(telemetry.value_of("Linear Speed"), 1.25);
    EXPECT_TRUE(std::isnan(telemetry.value_of("Missing")));
    EXPECT_EQ(telemetry.resync_count(), 0U);
    EXPECT_EQ(telemetry.map_failure_count(), 0U);
}

TEST(Telemetry, QueuesRequestsForTheFirmware) {
    SerialBus bus;
    Telemetry telemetry(bus);

    telemetry.request_variable_map();
    const std::vector<uint8_t> request = bus.take_for_firmware();
    ASSERT_FALSE(request.empty());
    EXPECT_EQ(comm::Packet(request).get_type(), comm::Packet::MessageType::SERIAL_VARIABLE_MAP_REQUEST);

    telemetry.send_variable(2, {0x01});
    const std::vector<uint8_t> command = bus.take_for_firmware();
    ASSERT_FALSE(command.empty());

    const comm::Packet packet(command);
    EXPECT_EQ(packet.get_type(), comm::Packet::MessageType::SERIAL_VARIABLE);
    EXPECT_EQ(packet.get_id(), 2);
    EXPECT_EQ(packet.get_payload(), (std::vector<uint8_t>{0x01}));
}

TEST(Telemetry, CountsMalformedMapResponsesInsteadOfAcceptingThem) {
    SerialBus bus;
    Telemetry telemetry(bus);

    std::vector<uint8_t> truncated = map_payload();
    truncated.pop_back();
    bus.send_from_firmware(comm::Packet(comm::Packet::MessageType::SERIAL_VARIABLE_MAP_RESPONSE, truncated).serialize()
    );
    telemetry.poll();

    EXPECT_FALSE(telemetry.has_variable_map());
    EXPECT_EQ(telemetry.map_failure_count(), 1U);
}
}  // namespace
}  // namespace micras::sim

#include <cstdint>
#include <vector>

#include <doctest/doctest.h>

#include "micras/sim/core/serial_bus.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
TEST_CASE("SerialBus.FansFirmwareOutputOutToEveryListener") {
    SerialBus     bus;
    ByteCollector first;
    ByteCollector second;

    bus.add_listener(first);
    bus.add_listener(second);
    bus.send_from_firmware(std::vector<uint8_t>{1, 2});
    bus.send_from_firmware(std::vector<uint8_t>{3});

    CHECK_EQ(first.received, (std::vector<uint8_t>{1, 2, 3}));
    CHECK_EQ(second.received, first.received);
}

TEST_CASE("SerialBus.StopsDeliveringToARemovedListener") {
    SerialBus     bus;
    ByteCollector listener;

    bus.add_listener(listener);
    bus.send_from_firmware(std::vector<uint8_t>{1});
    bus.remove_listener(listener);
    bus.send_from_firmware(std::vector<uint8_t>{2});

    CHECK_EQ(listener.received, (std::vector<uint8_t>{1}));
}

TEST_CASE("SerialBus.DropsFirmwareOutputWhenNobodyIsListening") {
    SerialBus bus;
    CHECK_NOTHROW(bus.send_from_firmware(std::vector<uint8_t>{1}));
}

TEST_CASE("SerialBus.HandsTheFirmwareTheBytesInTheOrderTheyWereQueued") {
    SerialBus bus;
    bus.queue_for_firmware(std::vector<uint8_t>{1, 2});
    bus.queue_for_firmware(std::vector<uint8_t>{3});

    CHECK_EQ(bus.take_for_firmware(2), (std::vector<uint8_t>{1, 2}));
    CHECK_EQ(bus.take_for_firmware(5), (std::vector<uint8_t>{3}));
    CHECK(bus.take_for_firmware(1).empty());
}

TEST_CASE("SerialBus.DropsAndCountsBytesPastTheBound") {
    SerialBus                  bus;
    const std::vector<uint8_t> bytes(SerialBus::max_pending_bytes + 3, 7);

    bus.queue_for_firmware(bytes);

    CHECK_EQ(bus.dropped_bytes(), 3U);
    CHECK_EQ(bus.take_for_firmware(SerialBus::max_pending_bytes * 2).size(), SerialBus::max_pending_bytes);
}
}  // namespace
}  // namespace micras::sim

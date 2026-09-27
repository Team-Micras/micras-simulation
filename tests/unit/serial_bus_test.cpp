#include <gtest/gtest.h>

#include "micras/sim/core/serial_bus.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
TEST(SerialBus, FansFirmwareOutputOutToEveryListener) {
    SerialBus     bus;
    ByteCollector first;
    ByteCollector second;

    bus.add_listener(first);
    bus.add_listener(second);
    bus.send_from_firmware(std::vector<uint8_t>{1, 2});
    bus.send_from_firmware(std::vector<uint8_t>{3});

    EXPECT_EQ(first.received, (std::vector<uint8_t>{1, 2, 3}));
    EXPECT_EQ(second.received, first.received);
}

TEST(SerialBus, StopsDeliveringToARemovedListener) {
    SerialBus     bus;
    ByteCollector listener;

    bus.add_listener(listener);
    bus.send_from_firmware(std::vector<uint8_t>{1});
    bus.remove_listener(listener);
    bus.send_from_firmware(std::vector<uint8_t>{2});

    EXPECT_EQ(listener.received, (std::vector<uint8_t>{1}));
}

TEST(SerialBus, DropsFirmwareOutputWhenNobodyIsListening) {
    SerialBus bus;
    EXPECT_NO_THROW(bus.send_from_firmware(std::vector<uint8_t>{1}));
}

TEST(SerialBus, HandsTheFirmwareTheBytesInTheOrderTheyWereQueued) {
    SerialBus bus;
    bus.queue_for_firmware(std::vector<uint8_t>{1, 2});
    bus.queue_for_firmware(std::vector<uint8_t>{3});

    EXPECT_EQ(bus.take_for_firmware(2), (std::vector<uint8_t>{1, 2}));
    EXPECT_EQ(bus.take_for_firmware(5), (std::vector<uint8_t>{3}));
    EXPECT_TRUE(bus.take_for_firmware(1).empty());
}

TEST(SerialBus, DropsAndCountsBytesPastTheBound) {
    SerialBus                  bus;
    const std::vector<uint8_t> bytes(SerialBus::max_pending_bytes + 3, 7);

    bus.queue_for_firmware(bytes);

    EXPECT_EQ(bus.dropped_bytes(), 3U);
    EXPECT_EQ(bus.take_for_firmware(SerialBus::max_pending_bytes * 2).size(), SerialBus::max_pending_bytes);
}
}  // namespace
}  // namespace micras::sim

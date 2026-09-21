#include <gtest/gtest.h>

#include "micras/sim/core/serial_bus.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Listener that keeps every byte it was handed.
 */
class RecordingListener : public ISerialListener {
public:
    void on_firmware_bytes(std::span<const uint8_t> bytes) override {
        this->received.insert(this->received.end(), bytes.begin(), bytes.end());
    }

    std::vector<uint8_t> received;
};

TEST(SerialBus, FansFirmwareOutputOutToEveryListener) {
    SerialBus         bus;
    RecordingListener first;
    RecordingListener second;

    bus.add_listener(first);
    bus.add_listener(second);
    bus.send_from_firmware(std::vector<uint8_t>{1, 2});
    bus.send_from_firmware(std::vector<uint8_t>{3});

    EXPECT_EQ(first.received, (std::vector<uint8_t>{1, 2, 3}));
    EXPECT_EQ(second.received, first.received);
}

TEST(SerialBus, StopsDeliveringToARemovedListener) {
    SerialBus         bus;
    RecordingListener listener;

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

TEST(SerialBus, HandsTheFirmwareOnePacketAtATime) {
    SerialBus bus;
    bus.queue_for_firmware(std::vector<uint8_t>{1, 2});
    bus.queue_for_firmware(std::vector<uint8_t>{3});

    EXPECT_EQ(bus.pending_packets(), 2U);
    EXPECT_EQ(bus.take_for_firmware(), (std::vector<uint8_t>{1, 2}));
    EXPECT_EQ(bus.take_for_firmware(), (std::vector<uint8_t>{3}));
    EXPECT_TRUE(bus.take_for_firmware().empty());
    EXPECT_EQ(bus.pending_packets(), 0U);
}

TEST(SerialBus, ClearDropsEveryQueuedPacket) {
    SerialBus bus;
    bus.queue_for_firmware(std::vector<uint8_t>{1});
    bus.clear();

    EXPECT_TRUE(bus.take_for_firmware().empty());
}
}  // namespace
}  // namespace micras::sim

#include <gtest/gtest.h>

#include "micras/comm/packet.hpp"
#include "micras/sim/telemetry/packet_framer.hpp"

namespace micras::sim {
namespace {
std::vector<uint8_t> serialized(comm::Packet::MessageType type, uint16_t id, const std::vector<uint8_t>& payload) {
    return comm::Packet(type, id, payload).serialize();
}

TEST(PacketFramer, YieldsOnePacketPerFrame) {
    PacketFramer framer;
    const auto   first = serialized(comm::Packet::MessageType::SERIAL_VARIABLE, 3, {1, 2, 3});
    const auto   second = serialized(comm::Packet::MessageType::SERIAL_VARIABLE, 4, {9});

    std::vector<uint8_t> stream = first;
    stream.insert(stream.end(), second.begin(), second.end());
    framer.push(stream);

    const auto packets = framer.drain();
    ASSERT_EQ(packets.size(), 2U);
    EXPECT_EQ(packets[0], first);
    EXPECT_EQ(packets[1], second);
    EXPECT_TRUE(framer.drain().empty());
}

TEST(PacketFramer, KeepsIncompleteTail) {
    PacketFramer framer;
    const auto   packet = serialized(comm::Packet::MessageType::SERIAL_VARIABLE, 3, {1, 2, 3});

    framer.push(std::span(packet).first(packet.size() - 2));
    EXPECT_TRUE(framer.drain().empty());

    framer.push(std::span(packet).last(2));
    const auto packets = framer.drain();
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets[0], packet);
}

TEST(PacketFramer, SkipsGarbageBeforeHeader) {
    PacketFramer framer;
    const auto   packet = serialized(comm::Packet::MessageType::SERIAL_VARIABLE, 3, {1, 2, 3});

    std::vector<uint8_t> stream{0x00, 0x11, 0x22};
    stream.insert(stream.end(), packet.begin(), packet.end());
    framer.push(stream);

    const auto packets = framer.drain();
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets[0], packet);
}

TEST(PacketFramer, HandlesControlBytesInsidePayload) {
    PacketFramer framer;
    const auto   packet = serialized(
        comm::Packet::MessageType::SERIAL_VARIABLE, comm::Packet::tail_byte,
        {comm::Packet::tail_byte, comm::Packet::escape_byte, comm::Packet::header_byte}
    );
    framer.push(packet);

    const auto packets = framer.drain();
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets[0], packet);
    EXPECT_EQ(comm::Packet(packets[0]).get_payload(), (std::vector<uint8_t>{0x7F, 0x7D, 0x42}));
}

TEST(PacketFramer, HandlesControlBytesInTheLengthField) {
    for (const std::size_t length : {std::size_t{comm::Packet::escape_byte}, std::size_t{comm::Packet::tail_byte}}) {
        PacketFramer framer;
        const auto   packet =
            serialized(comm::Packet::MessageType::SERIAL_VARIABLE, 1, std::vector<uint8_t>(length, 0x01));
        framer.push(packet);

        const auto packets = framer.drain();
        ASSERT_EQ(packets.size(), 1U) << "payload length " << length;
        EXPECT_EQ(packets[0], packet);
    }
}

TEST(PacketFramer, FramesAPacketPushedOneByteAtATime) {
    PacketFramer framer;
    const auto   packet = serialized(comm::Packet::MessageType::SERIAL_VARIABLE, 3, {0x7F, 0x7D, 0x42});

    for (std::size_t i = 0; i + 1 < packet.size(); i++) {
        framer.push(std::span(packet).subspan(i, 1));
        EXPECT_TRUE(framer.drain().empty()) << "framed early at byte " << i;
    }

    framer.push(std::span(packet).last(1));
    const auto packets = framer.drain();
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets[0], packet);
}

TEST(PacketFramer, ResyncsWhenNoFrameCloses) {
    PacketFramer               framer;
    const std::vector<uint8_t> noise(PacketFramer::max_buffer_size + 1, comm::Packet::header_byte);
    framer.push(noise);

    EXPECT_TRUE(framer.drain().empty());
    EXPECT_EQ(framer.resync_count(), 1U);
}

TEST(PacketFramer, FramesAgainAfterAResync) {
    PacketFramer               framer;
    const std::vector<uint8_t> noise(PacketFramer::max_buffer_size + 1, comm::Packet::header_byte);
    framer.push(noise);
    EXPECT_TRUE(framer.drain().empty());

    const auto packet = serialized(comm::Packet::MessageType::SERIAL_VARIABLE, 3, {1, 2, 3});
    framer.push(packet);

    const auto packets = framer.drain();
    ASSERT_FALSE(packets.empty());
    EXPECT_EQ(packets.back(), packet);
}
}  // namespace
}  // namespace micras::sim

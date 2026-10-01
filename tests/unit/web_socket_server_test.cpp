/**
 * @file
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <netinet/in.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <doctest/doctest.h>

#include "micras/sim/bridge/monitor_bridge.hpp"
#include "micras/sim/bridge/real_time_pacer.hpp"
#include "micras/sim/bridge/web_socket_server.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::steady_clock;

/**
 * @brief Owns a POSIX socket and closes it.
 */
class Socket {
public:
    Socket() : descriptor{socket(AF_INET, SOCK_STREAM, 0)} { }

    Socket(const Socket&) = delete;
    Socket(Socket&&) = delete;
    Socket& operator=(const Socket&) = delete;
    Socket& operator=(Socket&&) = delete;

    ~Socket() { close(this->descriptor); }

    int get() const { return this->descriptor; }

private:
    int descriptor;
};
}  // namespace

/**
 * @brief Build a loopback address.
 *
 * @param port Port in host order, 0 for any.
 * @return The address.
 */
static sockaddr_in loopback(uint16_t port) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return address;
}

/**
 * @brief Bind a socket to a port the system picks, listening.
 *
 * @param socket Socket to bind.
 * @return The port it holds.
 */
static uint16_t hold_free_port(const Socket& socket) {
    sockaddr_in address = loopback(0);
    socklen_t   length = sizeof(address);

    REQUIRE_GE(socket.get(), 0);

    // NOLINTBEGIN(*-reinterpret-cast): the sockets API takes its addresses as sockaddr.
    CHECK_EQ(bind(socket.get(), reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
    CHECK_EQ(listen(socket.get(), 1), 0);
    CHECK_EQ(getsockname(socket.get(), reinterpret_cast<sockaddr*>(&address), &length), 0);
    // NOLINTEND(*-reinterpret-cast)

    return ntohs(address.sin_port);
}

/**
 * @brief Find a port nobody listens on.
 *
 * @return The port, released again.
 */
static uint16_t free_port() {
    const Socket socket;
    return hold_free_port(socket);
}

/**
 * @brief Open a WebSocket connection, then never read from it again.
 *
 * @note The receive buffer is made as small as the system allows, so the
 *       server's writes back up after a few frames.
 *
 * @param socket Socket to connect.
 * @param port Port the server listens on.
 * @return True once the server answered the handshake.
 */
static bool connect_and_stall(const Socket& socket, uint16_t port) {
    const int receive_buffer = 1024;
    // NOLINTNEXTLINE(misc-include-cleaner): <sys/socket.h> provides them through a kernel header.
    setsockopt(socket.get(), SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));

    sockaddr_in address = loopback(port);

    // NOLINTNEXTLINE(*-reinterpret-cast): the sockets API takes its addresses as sockaddr.
    if (connect(socket.get(), reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        return false;
    }

    const std::string request = "GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                                "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";

    if (send(socket.get(), request.data(), request.size(), 0) != static_cast<ssize_t>(request.size())) {
        return false;
    }

    std::string         response;
    std::array<char, 1> character{};

    while (!response.contains("\r\n\r\n")) {
        if (recv(socket.get(), character.data(), 1, 0) != 1) {
            return false;
        }

        response.push_back(character.at(0));
    }

    return response.starts_with("HTTP/1.1 101");
}

/**
 * @brief Send one binary frame from the client side, masked as the protocol requires.
 *
 * @param socket Connected socket.
 * @param payload Bytes to send, fewer than 126.
 * @return True when the whole frame was written.
 */
static bool send_binary(const Socket& socket, const std::vector<uint8_t>& payload) {
    const std::array<uint8_t, 4> mask{0x12, 0x34, 0x56, 0x78};
    std::vector<uint8_t>         frame{0x82, static_cast<uint8_t>(0x80 | payload.size())};
    frame.insert(frame.end(), mask.begin(), mask.end());

    for (std::size_t index = 0; index < payload.size(); index++) {
        frame.push_back(payload.at(index) ^ mask.at(index % mask.size()));
    }

    return send(socket.get(), frame.data(), frame.size(), 0) == static_cast<ssize_t>(frame.size());
}

/**
 * @brief Read one unmasked binary frame the server sent.
 *
 * @param socket Connected socket.
 * @return The frame's payload, empty when it was not a short binary frame.
 */
static std::vector<uint8_t> receive_binary(const Socket& socket) {
    std::array<uint8_t, 2> head{};

    if (recv(socket.get(), head.data(), head.size(), MSG_WAITALL) != 2 or head.at(0) != 0x82 or head.at(1) >= 126) {
        return {};
    }

    std::vector<uint8_t> payload(head.at(1));

    if (recv(socket.get(), payload.data(), payload.size(), MSG_WAITALL) != static_cast<ssize_t>(payload.size())) {
        return {};
    }

    return payload;
}

/**
 * @brief Wait until a server has a number of clients, for a few seconds at most.
 *
 * @param server The server.
 * @param count How many clients to wait for.
 * @return True once it has them.
 */
static bool clients_become(const WebSocketServer& server, std::size_t count) {
    const auto deadline = steady_clock::now() + seconds(10);

    while (server.connected_clients() != count) {
        if (steady_clock::now() > deadline) {
            return false;
        }

        std::this_thread::sleep_for(milliseconds(1));
    }

    return true;
}

/**
 * @brief Get whether anything arrives on a socket within a while.
 *
 * @param socket Connected socket.
 * @param patience How long to wait.
 * @return True when bytes arrived.
 */
static bool readable_within(const Socket& socket, milliseconds patience) {
    pollfd ready{.fd = socket.get(), .events = POLLIN, .revents = 0};
    return poll(&ready, 1, static_cast<int>(patience.count())) == 1;
}

namespace {
/**
 * @brief What a close frame said.
 */
struct Closing {
    uint16_t    code{};
    std::string reason;
};
}  // namespace

/**
 * @brief Read the close frame the server sent.
 *
 * @param socket Connected socket.
 * @return Its code and reason, a code of 0 when the next frame was not a close
 *         frame with one or none came within 10 s.
 */
static Closing receive_close(const Socket& socket) {
    const int patience_ms = 10'000;
    pollfd    ready{.fd = socket.get(), .events = POLLIN, .revents = 0};

    if (poll(&ready, 1, patience_ms) != 1) {
        return {};
    }

    std::array<uint8_t, 4> head{};

    if (recv(socket.get(), head.data(), head.size(), MSG_WAITALL) != 4 or head.at(0) != 0x88 or head.at(1) < 2 or
        head.at(1) >= 126) {
        return {};
    }

    std::string reason(head.at(1) - 2U, '\0');

    if (recv(socket.get(), reason.data(), reason.size(), MSG_WAITALL) != static_cast<ssize_t>(reason.size())) {
        return {};
    }

    return {.code = static_cast<uint16_t>((head.at(2) << 8U) | head.at(3)), .reason = reason};
}

namespace {
/**
 * @brief The frames a server handed back, collected from its threads.
 */
class Inbox {
public:
    /**
     * @brief Get the handler that collects into this inbox.
     *
     * @return The handler.
     */
    WebSocketServer::BinaryHandler handler() {
        return [this](std::span<const uint8_t> bytes) {
            const std::scoped_lock lock(this->mutex);
            this->frames.emplace_back(bytes.begin(), bytes.end());
        };
    }

    /**
     * @brief Wait until a frame arrived, for a few seconds at most.
     *
     * @param frame The frame to wait for.
     * @return True when it arrived.
     */
    bool wait_for(const std::vector<uint8_t>& frame) {
        const auto deadline = steady_clock::now() + seconds(10);

        while (steady_clock::now() < deadline) {
            if (this->contains(frame)) {
                return true;
            }

            std::this_thread::sleep_for(milliseconds(1));
        }

        return false;
    }

    /**
     * @brief Get whether a frame arrived.
     *
     * @param frame The frame.
     * @return True when it did.
     */
    bool contains(const std::vector<uint8_t>& frame) {
        const std::scoped_lock lock(this->mutex);
        return std::ranges::find(this->frames, frame) != this->frames.end();
    }

private:
    std::mutex                        mutex;
    std::vector<std::vector<uint8_t>> frames;
};

TEST_CASE("MonitorBridge.CarriesRawBytesBothWays") {
    RunContext context;
    load_tiny_world(context.world, context.clock);
    FirmwareThread   firmware{[](FirmwareThread& thread) {
        while (thread.yield_tick()) { }
    }};
    const Simulation simulation(context, firmware);

    std::vector<uint8_t> inbound;
    std::string          error;
    const uint16_t       port = free_port();
    MonitorBridge        bridge(context.serial, port, error);
    REQUIRE_MESSAGE(bridge.is_open(), error);

    const Socket client;
    REQUIRE(connect_and_stall(client, port));

    const std::vector<uint8_t> command{0x00, 0x01, 0x7E, 0xFF, 0x00};
    REQUIRE(send_binary(client, command));

    const auto deadline = steady_clock::now() + seconds(10);

    while (inbound.empty() and steady_clock::now() < deadline) {
        bridge.on_before_tick(simulation);
        inbound = context.serial.take_for_firmware(command.size());
        std::this_thread::sleep_for(milliseconds(1));
    }

    CHECK_EQ(inbound, command);
    CHECK(bridge.was_interactive());

    const std::vector<uint8_t> telemetry{0x02, 0x00, 0xAA, 0x00};
    context.serial.send_from_firmware(telemetry);
    bridge.on_after_tick(simulation);

    CHECK_EQ(receive_binary(client), telemetry);
}

TEST_CASE("MonitorBridge.HoldsTheRunToTheWallClock") {
    RunContext context;
    load_tiny_world(context.world, context.clock);
    FirmwareThread firmware{[](FirmwareThread& thread) {
        while (thread.yield_tick()) { }
    }};
    Simulation     simulation(context, firmware);
    FakeWallClock  wall;
    const auto     start = wall.now();

    std::string   error;
    MonitorBridge bridge(context.serial, free_port(), error, wall);
    REQUIRE_MESSAGE(bridge.is_open(), error);
    simulation.add_listener(bridge);
    simulation.run(1000);

    const auto lead = microseconds(context.clock.now_us() - context.clock.us_per_tick()) - (wall.now() - start);
    CHECK_GT(wall.sleep_count(), 0);
    CHECK_LE(lead, RealTimePacer::ahead_slack);
    CHECK_GE(lead, microseconds(0));
}

TEST_CASE("MonitorBridge.AnchorsTheWallClockWhereTheRunStarts") {
    RunContext context;
    load_tiny_world(context.world, context.clock);
    FirmwareThread firmware{[](FirmwareThread& thread) {
        while (thread.yield_tick()) { }
    }};
    Simulation     simulation(context, firmware);
    FakeWallClock  wall;

    std::string   error;
    MonitorBridge bridge(context.serial, free_port(), error, wall);
    REQUIRE_MESSAGE(bridge.is_open(), error);
    simulation.add_listener(bridge);

    wall.advance(milliseconds(3));
    const auto start = wall.now();
    simulation.run(2);

    CHECK_EQ(wall.sleep_count(), 1);
    CHECK_EQ(wall.now() - start, microseconds(context.clock.us_per_tick()));
}

TEST_CASE("MonitorBridge.NeverPacesARunWhenItIsNotListening") {
    RunContext context;
    load_tiny_world(context.world, context.clock);
    FirmwareThread firmware{[](FirmwareThread& thread) {
        while (thread.yield_tick()) { }
    }};
    Simulation     simulation(context, firmware);
    FakeWallClock  wall;

    const Socket   holder;
    const uint16_t port = hold_free_port(holder);
    std::string    error;
    MonitorBridge  bridge(context.serial, port, error, wall);
    REQUIRE_FALSE(bridge.is_open());
    simulation.add_listener(bridge);
    simulation.run(1000);

    CHECK_EQ(context.clock.tick_count(), 1000U);
    CHECK_EQ(wall.sleep_count(), 0);
}

TEST_CASE("WebSocketServer.AClientThatStopsReadingNeverStallsTheCaller") {
    WebSocketServer server;
    std::string     error;
    const uint16_t  port = free_port();
    REQUIRE_MESSAGE(server.start(port, error), error);

    const Socket client;
    REQUIRE(connect_and_stall(client, port));

    const std::vector<uint8_t> frame(std::size_t{64} * 1024, 0xAB);
    const auto                 deadline = steady_clock::now() + seconds(10);
    auto                       slowest = steady_clock::duration::zero();

    while (server.dropped_frames() == 0 and steady_clock::now() < deadline) {
        const auto start = steady_clock::now();
        server.broadcast(frame);
        slowest = std::max(slowest, steady_clock::now() - start);
    }

    CHECK_GT(server.dropped_frames(), 0U);
    CHECK_LT(slowest, milliseconds(100));

    auto stopping = std::async(std::launch::async, [&server] { server.stop(); });
    CHECK_EQ(stopping.wait_for(seconds(10)), std::future_status::ready);
}

TEST_CASE("WebSocketServer.TheClientThatSendsTakesTheRadio") {
    WebSocketServer server;
    Inbox           inbox;
    std::string     error;
    const uint16_t  port = free_port();
    server.set_on_binary(inbox.handler());
    REQUIRE_MESSAGE(server.start(port, error), error);

    std::optional<Socket>      first{std::in_place};
    const std::vector<uint8_t> from_first{0x01};
    REQUIRE(connect_and_stall(*first, port));
    REQUIRE(send_binary(*first, from_first));
    REQUIRE(inbox.wait_for(from_first));

    std::optional<Socket> second{std::in_place};
    REQUIRE(connect_and_stall(*second, port));
    REQUIRE(clients_become(server, 2));

    const std::vector<uint8_t> from_second{0x02};
    REQUIRE(send_binary(*second, from_second));

    const Closing closing = receive_close(*first);
    CHECK_EQ(closing.code, WebSocketServer::taken_over_close_code);
    CHECK_EQ(closing.reason, WebSocketServer::taken_over_reason);
    REQUIRE(inbox.wait_for(from_second));

    const std::vector<uint8_t> late{0x03};
    send_binary(*first, late);
    first.reset();
    REQUIRE(clients_become(server, 1));

    const std::vector<uint8_t> again{0x04};
    REQUIRE(send_binary(*second, again));
    REQUIRE(inbox.wait_for(again));
    CHECK_FALSE(inbox.contains(late));

    const std::vector<uint8_t> telemetry{0xAA, 0x55};
    server.broadcast(telemetry);
    CHECK_EQ(receive_binary(*second), telemetry);

    second.reset();
    CHECK(clients_become(server, 0));
}

TEST_CASE("WebSocketServer.AClientThatSaysNothingTakesNothing") {
    WebSocketServer server;
    Inbox           inbox;
    std::string     error;
    const uint16_t  port = free_port();
    server.set_on_binary(inbox.handler());
    REQUIRE_MESSAGE(server.start(port, error), error);

    {
        const Socket               holder;
        const std::vector<uint8_t> from_holder{0x01};
        REQUIRE(connect_and_stall(holder, port));
        REQUIRE(send_binary(holder, from_holder));
        REQUIRE(inbox.wait_for(from_holder));

        const Socket idle;
        REQUIRE(connect_and_stall(idle, port));
        REQUIRE(clients_become(server, 2));

        const std::vector<uint8_t> telemetry{0xAA, 0x55};
        server.broadcast(telemetry);
        CHECK_EQ(receive_binary(holder), telemetry);
        CHECK_FALSE(readable_within(idle, milliseconds(200)));

        const std::vector<uint8_t> again{0x02};
        REQUIRE(send_binary(holder, again));
        CHECK(inbox.wait_for(again));
    }

    CHECK(clients_become(server, 0));
}

TEST_CASE("WebSocketServer.RefusesAPortSomebodyElseHolds") {
    const Socket   holder;
    const uint16_t port = hold_free_port(holder);

    WebSocketServer server;
    std::string     error;

    CHECK_FALSE(server.start(port, error));
    CHECK_FALSE(error.empty());
}
}  // namespace
}  // namespace micras::sim

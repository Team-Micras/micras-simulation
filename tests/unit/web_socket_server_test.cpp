/**
 * @file
 */

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <doctest/doctest.h>

#include "micras/sim/bridge/monitor_bridge.hpp"
#include "micras/sim/bridge/web_socket_server.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/simulation.hpp"
#include "support.hpp"

namespace micras::sim {
namespace {
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

/**
 * @brief Build a loopback address.
 *
 * @param port Port in host order, 0 for any.
 * @return The address.
 */
sockaddr_in loopback(uint16_t port) {
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
uint16_t hold_free_port(const Socket& socket) {
    sockaddr_in address = loopback(0);
    socklen_t   length = sizeof(address);

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
uint16_t free_port() {
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
bool connect_and_stall(const Socket& socket, uint16_t port) {
    const int receive_buffer = 1024;
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

    while (response.find("\r\n\r\n") == std::string::npos) {
        if (recv(socket.get(), character.data(), 1, 0) != 1) {
            return false;
        }

        response.push_back(character[0]);
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
bool send_binary(const Socket& socket, const std::vector<uint8_t>& payload) {
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
std::vector<uint8_t> receive_binary(const Socket& socket) {
    std::array<uint8_t, 2> head{};

    if (recv(socket.get(), head.data(), head.size(), MSG_WAITALL) != 2 or head[0] != 0x82 or head[1] >= 126) {
        return {};
    }

    std::vector<uint8_t> payload(head[1]);

    if (recv(socket.get(), payload.data(), payload.size(), MSG_WAITALL) != static_cast<ssize_t>(payload.size())) {
        return {};
    }

    return payload;
}

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

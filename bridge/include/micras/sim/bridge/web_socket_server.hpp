/**
 * @file
 *
 * @brief A local WebSocket server, behind an interface the harness owns.
 */

#ifndef MICRAS_SIM_BRIDGE_WEB_SOCKET_SERVER_HPP
#define MICRAS_SIM_BRIDGE_WEB_SOCKET_SERVER_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace micras::sim {
/**
 * @brief Serves binary frames to one client at a time, and hands back what it sends.
 *
 * @note One client holds the radio, as a radio pairs with one peer: two
 *       monitors on one robot would each configure it and each read the
 *       answers to the other. A client takes it with its first binary frame,
 *       so one that connects and says nothing never does, and the one that
 *       held it is closed with taken_over_close_code and taken_over_reason;
 *       what that one still sends before it is gone is ignored.
 *
 * @note A facade on purpose: it keeps IXWebSocket out of every header, and lets
 *       -DMICRAS_SIM_BRIDGE=OFF build a stub in its place. Callbacks arrive on the
 *       server's own threads.
 */
class WebSocketServer {
public:
    /**
     * @brief Called with every binary frame the client that holds the radio sends.
     */
    using BinaryHandler = std::function<void(std::span<const uint8_t>)>;

    WebSocketServer();

    WebSocketServer(const WebSocketServer&) = delete;
    WebSocketServer(WebSocketServer&&) = delete;
    WebSocketServer& operator=(const WebSocketServer&) = delete;
    WebSocketServer& operator=(WebSocketServer&&) = delete;

    /**
     * @brief Stop the server and disconnect every client.
     */
    ~WebSocketServer();

    /**
     * @brief Set the handler for incoming frames, before starting.
     *
     * @param handler Handler called on a server thread.
     */
    void set_on_binary(BinaryHandler handler);

    /**
     * @brief Start listening.
     *
     * @param port TCP port to listen on.
     * @param error Filled with a human readable reason when the port cannot be taken.
     * @return False when the server could not start.
     */
    bool start(int port, std::string& error);

    /**
     * @brief Stop listening and disconnect every client.
     */
    void stop();

    /**
     * @brief Queue one binary frame for the client that holds the radio.
     *
     * @note Never blocks on a client. The frame goes into a bounded queue that a
     *       sender thread drains, because the library writes synchronously and
     *       waits without a deadline for a client that stopped reading. When the
     *       queue is full the oldest frame is dropped and counted.
     *
     * @param bytes Frame payload.
     */
    void broadcast(std::span<const uint8_t> bytes);

    /**
     * @brief Get how many frames were dropped because the send queue was full.
     *
     * @return Number of dropped frames, always 0 when every client keeps up.
     */
    uint64_t dropped_frames() const;

    /**
     * @brief Get how many clients are connected, whether they hold the radio or not.
     *
     * @return Number of clients.
     */
    std::size_t connected_clients() const;

    /**
     * @brief Largest number of frames allowed to wait for the sender thread.
     */
    static constexpr std::size_t max_queued_frames{256};

    /**
     * @brief The close code a client gets when a newer one takes the radio by sending.
     *
     * @note In the range RFC 6455 leaves to applications; micras-monitor stops
     *       reconnecting when it gets it.
     */
    static constexpr uint16_t taken_over_close_code{4001};

    /**
     * @brief The close reason that goes with taken_over_close_code.
     */
    static constexpr std::string_view taken_over_reason{"another monitor took the link"};

private:
    struct Impl;

    /**
     * @brief The library this facade hides.
     */
    std::unique_ptr<Impl> impl;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_BRIDGE_WEB_SOCKET_SERVER_HPP

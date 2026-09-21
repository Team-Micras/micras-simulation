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
#include <vector>

namespace micras::sim {
/**
 * @brief Serves binary frames to whoever connects, and hands back what they send.
 *
 * @note A facade on purpose: the harness only needs start, stop, broadcast and
 *       a callback, so the library behind it can be replaced without touching
 *       anything that uses it. Callbacks arrive on the server's own threads.
 */
class WebSocketServer {
public:
    /**
     * @brief Called with every binary frame a client sends.
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
     * @brief Send one binary frame to every connected client.
     *
     * @param bytes Frame payload.
     */
    void broadcast(std::span<const uint8_t> bytes);

    /**
     * @brief Get how many clients are connected.
     *
     * @return Number of clients.
     */
    std::size_t client_count() const;

private:
    struct Impl;

    /**
     * @brief The library this facade hides.
     */
    std::unique_ptr<Impl> impl;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_BRIDGE_WEB_SOCKET_SERVER_HPP

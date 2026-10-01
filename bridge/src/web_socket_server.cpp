/**
 * @file
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>

#include "micras/sim/bridge/web_socket_server.hpp"

#ifdef MICRAS_SIM_BRIDGE

    #include <condition_variable>
    #include <deque>
    #include <mutex>
    #include <set>
    #include <thread>

    #include <ixwebsocket/IXWebSocket.h>
    #include <ixwebsocket/IXWebSocketMessage.h>
    #include <ixwebsocket/IXWebSocketMessageType.h>
    #include <ixwebsocket/IXWebSocketServer.h>

namespace micras::sim {
/**
 * @brief Everything IXWebSocket needs, kept out of the header.
 */
struct WebSocketServer::Impl {
    /**
     * @brief Send every queued frame until asked to stop.
     *
     * @note Runs on its own thread, so a client that stops reading blocks this
     *       thread and never the one that calls broadcast().
     */
    void send_loop();

    /**
     * @brief What a client that sent a frame got.
     */
    struct Claim {
        /**
         * @brief Whether it holds the radio now; false for one a newer client took it from.
         */
        bool granted;

        /**
         * @brief The client it took the radio from, null when none.
         */
        const ix::WebSocket* displaced;
    };

    /**
     * @brief Give the radio to a client that sends a frame, unless a newer one took it from it.
     *
     * @param client The client.
     * @return What it got.
     */
    Claim claim_radio(const ix::WebSocket& client);

    /**
     * @brief Forget a client that left, freeing the radio when it held it.
     *
     * @param client The client that left.
     */
    void release(const ix::WebSocket& client);

    /**
     * @brief Close the client a newer one took the radio from.
     *
     * @param client The client.
     */
    void close_displaced(const ix::WebSocket* client) const;

    /**
     * @brief Give the radio to the client whose binary frame this is and close the
     *        one that held it, hand on the frame, and forget a client that leaves.
     *
     * @note Only a frame claims the radio, so a client that connects and says
     *       nothing, such as a probe or an idle tab, never takes it.
     *
     * @param client The client the message is about.
     * @param message What happened.
     */
    void on_message(ix::WebSocket& client, const ix::WebSocketMessage& message);

    std::unique_ptr<ix::WebSocketServer> server;
    BinaryHandler                        on_binary;

    /**
     * @brief Guards the queue, the drop counter, the stop flag, the peer and the displaced clients.
     */
    std::mutex mutex;

    /**
     * @brief Signals a queued frame or a request to stop.
     */
    std::condition_variable queued;

    /**
     * @brief Frames waiting for the sender thread, oldest first.
     */
    std::deque<std::string> frames;

    /**
     * @brief Frames dropped because the queue was full.
     */
    uint64_t dropped{0};

    /**
     * @brief Set when the sender thread must leave.
     */
    bool stopping{false};

    /**
     * @brief The client that holds the radio, null while none does.
     */
    const ix::WebSocket* peer{nullptr};

    /**
     * @brief Clients a newer one took the radio from, ignored until they leave.
     */
    std::set<const ix::WebSocket*> displaced;

    /**
     * @brief Drains the queue into the clients.
     */
    std::thread sender;
};

void WebSocketServer::Impl::send_loop() {
    while (true) {
        std::string          frame;
        const ix::WebSocket* receiver{};

        {
            std::unique_lock lock(this->mutex);
            this->queued.wait(lock, [this] { return this->stopping or not this->frames.empty(); });

            if (this->stopping) {
                return;
            }

            frame = std::move(this->frames.front());
            this->frames.pop_front();
            receiver = this->peer;
        }

        for (const auto& client : this->server->getClients()) {
            if (client.get() == receiver) {
                client->sendBinary(frame);
            }
        }
    }
}

WebSocketServer::Impl::Claim WebSocketServer::Impl::claim_radio(const ix::WebSocket& client) {
    const std::scoped_lock lock(this->mutex);

    if (this->displaced.contains(&client)) {
        return {.granted = false, .displaced = nullptr};
    }

    const ix::WebSocket* previous = this->peer == &client ? nullptr : this->peer;

    if (previous != nullptr) {
        this->displaced.insert(previous);
    }

    this->peer = &client;
    return {.granted = true, .displaced = previous};
}

void WebSocketServer::Impl::release(const ix::WebSocket& client) {
    const std::scoped_lock lock(this->mutex);
    this->displaced.erase(&client);

    if (this->peer == &client) {
        this->peer = nullptr;
    }
}

void WebSocketServer::Impl::close_displaced(const ix::WebSocket* client) const {
    for (const auto& other : this->server->getClients()) {
        if (other.get() == client) {
            other->close(taken_over_close_code, std::string{taken_over_reason});
        }
    }
}

void WebSocketServer::Impl::on_message(ix::WebSocket& client, const ix::WebSocketMessage& message) {
    if (message.type == ix::WebSocketMessageType::Close) {
        this->release(client);
        return;
    }

    if (message.type != ix::WebSocketMessageType::Message or not message.binary) {
        return;
    }

    const Claim claim = this->claim_radio(client);

    if (not claim.granted) {
        return;
    }

    if (claim.displaced != nullptr) {
        this->close_displaced(claim.displaced);
    }

    if (this->on_binary) {
        const std::span<const uint8_t> bytes(
            reinterpret_cast<const uint8_t*>(message.str.data()),  // NOLINT(*-reinterpret-cast): byte view.
            message.str.size()
        );
        this->on_binary(bytes);
    }
}

WebSocketServer::WebSocketServer() : impl{std::make_unique<Impl>()} { }

WebSocketServer::~WebSocketServer() {
    this->stop();
}

void WebSocketServer::set_on_binary(BinaryHandler handler) {
    this->impl->on_binary = std::move(handler);
}

bool WebSocketServer::start(int port, std::string& error) {
    this->impl->server = std::make_unique<ix::WebSocketServer>(port);
    this->impl->server->disablePerMessageDeflate();

    this->impl->server->setOnClientMessageCallback(
        [this](const auto& /*state*/, ix::WebSocket& client, const ix::WebSocketMessagePtr& message) {
            this->impl->on_message(client, *message);
        }
    );

    const auto result = this->impl->server->listen();

    if (not result.first) {
        error = result.second;
        this->impl->server.reset();
        return false;
    }

    this->impl->server->start();
    this->impl->stopping = false;
    this->impl->sender = std::thread(&Impl::send_loop, this->impl.get());
    return true;
}

void WebSocketServer::stop() {
    if (this->impl->server == nullptr) {
        return;
    }

    {
        const std::scoped_lock lock(this->impl->mutex);
        this->impl->stopping = true;
        this->impl->frames.clear();
        this->impl->peer = nullptr;
        this->impl->displaced.clear();
    }

    this->impl->queued.notify_all();
    this->impl->server->stop();
    this->impl->sender.join();
    this->impl->server.reset();
}

void WebSocketServer::broadcast(std::span<const uint8_t> bytes) {
    if (this->impl->server == nullptr or bytes.empty()) {
        return;
    }

    {
        const std::scoped_lock lock(this->impl->mutex);

        if (this->impl->frames.size() >= max_queued_frames) {
            this->impl->frames.pop_front();
            this->impl->dropped++;
        }

        // NOLINTNEXTLINE(*-reinterpret-cast): IXWebSocket takes a std::string as its byte buffer.
        this->impl->frames.emplace_back(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    }

    this->impl->queued.notify_one();
}

uint64_t WebSocketServer::dropped_frames() const {
    const std::scoped_lock lock(this->impl->mutex);
    return this->impl->dropped;
}

std::size_t WebSocketServer::connected_clients() const {
    return this->impl->server == nullptr ? 0 : this->impl->server->getClients().size();
}
}  // namespace micras::sim

#else  // MICRAS_SIM_BRIDGE

namespace micras::sim {
/**
 * @brief Nothing to hide when the bridge is compiled out.
 */
struct WebSocketServer::Impl { };

WebSocketServer::WebSocketServer() : impl{std::make_unique<Impl>()} { }

WebSocketServer::~WebSocketServer() = default;

void WebSocketServer::set_on_binary(BinaryHandler /*handler*/) { }

bool WebSocketServer::start(int /*port*/, std::string& error) {
    error = "this binary was built with -DMICRAS_SIM_BRIDGE=OFF";
    return false;
}

void WebSocketServer::stop() { }

void WebSocketServer::broadcast(std::span<const uint8_t> /*bytes*/) { }

uint64_t WebSocketServer::dropped_frames() const {
    return 0;
}

std::size_t WebSocketServer::connected_clients() const {
    return 0;
}
}  // namespace micras::sim

#endif  // MICRAS_SIM_BRIDGE

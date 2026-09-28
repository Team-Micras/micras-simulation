/**
 * @file
 */

#include <utility>

#include "micras/sim/bridge/web_socket_server.hpp"

#ifdef MICRAS_SIM_BRIDGE

    #include <condition_variable>
    #include <deque>
    #include <mutex>
    #include <thread>

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

    std::unique_ptr<ix::WebSocketServer> server;
    BinaryHandler                        on_binary;

    /**
     * @brief Guards the queue, the drop counter and the stop flag.
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
     * @brief Drains the queue into the clients.
     */
    std::thread sender;
};

void WebSocketServer::Impl::send_loop() {
    while (true) {
        std::string frame;

        {
            std::unique_lock lock(this->mutex);
            this->queued.wait(lock, [this] { return this->stopping or not this->frames.empty(); });

            if (this->stopping) {
                return;
            }

            frame = std::move(this->frames.front());
            this->frames.pop_front();
        }

        for (const auto& client : this->server->getClients()) {
            client->sendBinary(frame);
        }
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
        [this](const std::shared_ptr<ix::ConnectionState>&, ix::WebSocket&, const ix::WebSocketMessagePtr& message) {
            if (message->type != ix::WebSocketMessageType::Message or not message->binary) {
                return;
            }

            if (this->impl->on_binary) {
                const std::span<const uint8_t> bytes(
                    reinterpret_cast<const uint8_t*>(message->str.data()),  // NOLINT(*-reinterpret-cast): byte view.
                    message->str.size()
                );
                this->impl->on_binary(bytes);
            }
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
        const std::lock_guard lock(this->impl->mutex);
        this->impl->stopping = true;
        this->impl->frames.clear();
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
        const std::lock_guard lock(this->impl->mutex);

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
    const std::lock_guard lock(this->impl->mutex);
    return this->impl->dropped;
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
}  // namespace micras::sim

#endif  // MICRAS_SIM_BRIDGE

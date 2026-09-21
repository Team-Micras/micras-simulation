/**
 * @file
 */

#include <utility>

#include "micras/sim/bridge/web_socket_server.hpp"

#ifdef MICRAS_BRIDGE

    #include <ixwebsocket/IXWebSocketServer.h>

namespace micras::sim {
/**
 * @brief Everything IXWebSocket needs, kept out of the header.
 */
struct WebSocketServer::Impl {
    std::unique_ptr<ix::WebSocketServer> server;
    BinaryHandler                        on_binary;
};

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
    return true;
}

void WebSocketServer::stop() {
    if (this->impl->server == nullptr) {
        return;
    }

    this->impl->server->stop();
    this->impl->server.reset();
}

void WebSocketServer::broadcast(std::span<const uint8_t> bytes) {
    if (this->impl->server == nullptr or bytes.empty()) {
        return;
    }

    // NOLINTNEXTLINE(*-reinterpret-cast): IXWebSocket takes a std::string as its byte buffer.
    const std::string frame(reinterpret_cast<const char*>(bytes.data()), bytes.size());

    for (const auto& client : this->impl->server->getClients()) {
        client->sendBinary(frame);
    }
}

std::size_t WebSocketServer::client_count() const {
    return this->impl->server == nullptr ? 0 : this->impl->server->getClients().size();
}
}  // namespace micras::sim

#else  // MICRAS_BRIDGE

namespace micras::sim {
/**
 * @brief Nothing to hide when the bridge is compiled out.
 */
struct WebSocketServer::Impl { };

WebSocketServer::WebSocketServer() : impl{std::make_unique<Impl>()} { }

WebSocketServer::~WebSocketServer() = default;

void WebSocketServer::set_on_binary(BinaryHandler /*handler*/) { }

bool WebSocketServer::start(int /*port*/, std::string& error) {
    error = "this binary was built with -DMICRAS_BRIDGE=OFF";
    return false;
}

void WebSocketServer::stop() { }

void WebSocketServer::broadcast(std::span<const uint8_t> /*bytes*/) { }

std::size_t WebSocketServer::client_count() const {
    return 0;
}
}  // namespace micras::sim

#endif  // MICRAS_BRIDGE

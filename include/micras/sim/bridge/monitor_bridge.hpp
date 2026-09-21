/**
 * @file
 *
 * @brief Puts the firmware's radio on a WebSocket, for micras-monitor.
 */

#ifndef MICRAS_SIM_BRIDGE_MONITOR_BRIDGE_HPP
#define MICRAS_SIM_BRIDGE_MONITOR_BRIDGE_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "micras/sim/bridge/web_socket_server.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/simulation.hpp"
#include "micras/sim/telemetry/packet_framer.hpp"

namespace micras::sim {
/**
 * @brief Carries the firmware's bytes to a monitor and its packets back.
 *
 * @note The monitor talks the same packet protocol the real radio does, so it
 *       connects to the simulation without being told it is one. Everything it
 *       sends is framed and queued on the bus, which hands the firmware one
 *       packet per tick, so a burst from the monitor cannot change how many
 *       packets a tick delivers and the run stays as reproducible as the
 *       monitor's own input allows.
 */
class MonitorBridge : public ISerialListener, public IRunListener {
public:
    /**
     * @brief Open the bridge on a port.
     *
     * @param serial Bus the firmware reads from and writes to.
     * @param port TCP port to listen on.
     * @param error Filled with a human readable reason when the port cannot be taken.
     */
    MonitorBridge(SerialBus& serial, int port, std::string& error);

    MonitorBridge(const MonitorBridge&) = delete;
    MonitorBridge(MonitorBridge&&) = delete;
    MonitorBridge& operator=(const MonitorBridge&) = delete;
    MonitorBridge& operator=(MonitorBridge&&) = delete;

    /**
     * @brief Stop the server and leave the bus.
     */
    ~MonitorBridge() override;

    /**
     * @brief Check whether the bridge is listening.
     *
     * @return True when the port was taken.
     */
    bool is_open() const { return this->open; }

    /**
     * @brief Get how many monitors are connected.
     *
     * @return Number of clients.
     */
    std::size_t client_count() const { return this->server.client_count(); }

    /**
     * @brief Collect the bytes the firmware wrote during the tick.
     *
     * @param bytes Bytes leaving the firmware.
     */
    void on_firmware_bytes(std::span<const uint8_t> bytes) override;

    /**
     * @brief Hand the monitor's packets to the firmware, one tick's worth.
     *
     * @param simulation Run about to advance.
     * @return Always RunControl::RUN; a monitor never stops a run.
     */
    RunControl on_before_tick(const Simulation& simulation) override;

    /**
     * @brief Send the tick's output to every connected monitor.
     *
     * @param simulation Run that just advanced.
     */
    void on_after_tick(const Simulation& simulation) override;

private:
    /**
     * @brief Bus the firmware reads from and writes to.
     *
     * @note Bound for the life of the bridge; the context outlives it.
     */
    SerialBus& serial;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief The server itself.
     */
    WebSocketServer server;

    /**
     * @brief Whether the port was taken.
     */
    bool open{false};

    /**
     * @brief Bytes the firmware wrote during the current tick.
     */
    std::vector<uint8_t> outgoing;

    /**
     * @brief Guards everything the server threads touch.
     */
    mutable std::mutex mutex;

    /**
     * @brief Frames what the monitors send, on the simulation thread.
     *
     * @note Guarded, because bytes arrive on a server thread and are framed
     *       when the simulation drains them.
     */
    PacketFramer incoming;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_BRIDGE_MONITOR_BRIDGE_HPP

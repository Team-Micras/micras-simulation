/**
 * @file
 *
 * @brief Puts the firmware's radio on a WebSocket, for micras-monitor.
 */

#ifndef MICRAS_SIM_BRIDGE_MONITOR_BRIDGE_HPP
#define MICRAS_SIM_BRIDGE_MONITOR_BRIDGE_HPP

#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include "micras/sim/bridge/real_time_pacer.hpp"
#include "micras/sim/bridge/web_socket_server.hpp"
#include "micras/sim/core/serial_bus.hpp"
#include "micras/sim/core/simulation.hpp"

namespace micras::sim {
/**
 * @brief Carries the firmware's bytes to a monitor and the monitor's bytes back.
 *
 * @note Bytes only: the bridge frames nothing. What a monitor sends is collected
 *       on a server thread and queued for the firmware on the simulation thread,
 *       before the tick. The firmware's output leaves as one frame per tick.
 *
 * @note A listening bridge holds the run to real time, as the robot lives in
 *       it: a monitor budgets its link in wall time, and a run left free would
 *       produce faster than the link carries. Only the wall clock waits, so
 *       simulated time and every recording stay what they are without it.
 */
class MonitorBridge : public ISerialListener, public IRunListener {
public:
    /**
     * @brief Open the bridge on a port.
     *
     * @param serial Bus the firmware reads from and writes to.
     * @param port TCP port to listen on.
     * @param error Filled with a human readable reason when the port cannot be taken.
     * @param wall Clock the run is paced against; it must outlive the bridge.
     */
    MonitorBridge(SerialBus& serial, int port, std::string& error, IWallClock& wall = steady_wall_clock());

    MonitorBridge(const MonitorBridge&) = delete;
    MonitorBridge(MonitorBridge&&) = delete;
    MonitorBridge& operator=(const MonitorBridge&) = delete;
    MonitorBridge& operator=(MonitorBridge&&) = delete;

    /**
     * @brief Leave the bus; the server stops as the last member goes.
     */
    ~MonitorBridge() override;

    /**
     * @brief Check whether the bridge is listening.
     *
     * @return True when the port was taken.
     */
    bool is_open() const { return this->open; }

    /**
     * @brief Check whether a monitor sent anything the firmware was handed.
     *
     * @note A run that received bytes from outside is no longer reproducible from
     *       its arguments alone, so the application marks it interactive.
     *
     * @return True once the first bytes from a monitor were handed over.
     */
    bool was_interactive() const { return this->received; }

    /**
     * @brief Get how many frames never reached the monitors.
     *
     * @return Frames dropped because the monitors did not keep up.
     */
    uint64_t dropped_frames() const { return this->server.dropped_frames(); }

    /**
     * @brief Collect the bytes the firmware wrote during the tick.
     *
     * @param bytes Bytes leaving the firmware.
     */
    void on_firmware_bytes(std::span<const uint8_t> bytes) override;

    /**
     * @brief Anchor the run's simulated time to the wall clock.
     *
     * @param simulation Run about to start.
     */
    void on_start(const Simulation& simulation) override;

    /**
     * @brief Hold the run back to the wall clock, then queue what the monitors sent for the firmware.
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
     * @brief Whether the port was taken.
     */
    bool open{false};

    /**
     * @brief Whether bytes from a monitor were handed over.
     */
    bool received{false};

    /**
     * @brief Keeps simulated time from running ahead of the wall clock.
     */
    RealTimePacer pacer;

    /**
     * @brief Bytes the firmware wrote during the current tick.
     */
    std::vector<uint8_t> outgoing;

    /**
     * @brief Guards the incoming bytes, which a server thread appends to.
     */
    mutable std::mutex mutex;

    /**
     * @brief Bytes the monitors sent since the last tick.
     */
    std::vector<uint8_t> incoming;

    /**
     * @brief The server itself.
     *
     * @note Declared last so it is destroyed first: its threads call back into
     *       the mutex and the buffer above, which must outlive them.
     */
    WebSocketServer server;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_BRIDGE_MONITOR_BRIDGE_HPP

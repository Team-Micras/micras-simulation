/**
 * @file
 *
 * @brief The run loop and the interface everything that watches it implements.
 */

#ifndef MICRAS_SIM_CORE_SIMULATION_HPP
#define MICRAS_SIM_CORE_SIMULATION_HPP

#include <cstdint>
#include <vector>

#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"

namespace micras::sim {
/**
 * @brief Whether the next tick should run.
 */
enum class RunControl : uint8_t {
    RUN,
    QUIT,
};

class Simulation;

/**
 * @brief Watches a run without being able to change it.
 *
 * @note Listeners never touch mjData or the Clock. The only thing they may
 *       change is the interface input, which the simulation thread applies at
 *       the tick boundary, and whether the next tick runs at all.
 */
class IRunListener {
public:
    IRunListener() = default;

    IRunListener(const IRunListener&) = delete;
    IRunListener(IRunListener&&) = delete;
    IRunListener& operator=(const IRunListener&) = delete;
    IRunListener& operator=(IRunListener&&) = delete;

    virtual ~IRunListener() = default;

    /**
     * @brief Called once before the first tick.
     *
     * @param simulation Run about to start.
     */
    virtual void on_start(const Simulation& simulation);

    /**
     * @brief Called before each tick, and decides whether it runs.
     *
     * @note A frontend that pauses the run blocks inside this call. A tick that
     *       is refused here, or whose program already ended, gets no matching
     *       on_after_tick.
     *
     * @param simulation Run about to advance.
     * @return Whether to run the tick or stop the run.
     */
    virtual RunControl on_before_tick(const Simulation& simulation);

    /**
     * @brief Called after each tick, once the world and the clock have advanced.
     *
     * @param simulation Run that just advanced.
     */
    virtual void on_after_tick(const Simulation& simulation);

    /**
     * @brief Called once after the last tick.
     *
     * @param simulation Run that finished.
     */
    virtual void on_finish(const Simulation& simulation);
};

/**
 * @brief Drives the firmware and the physics, one tick at a time.
 *
 * @note The order inside a tick is the contract every recorded run depends on:
 *       the firmware body runs first, then the physics steps, then the clock
 *       advances, and only then do the listeners see the tick.
 */
class Simulation {
public:
    /**
     * @brief Take the context and the program to drive.
     *
     * @param context Simulation the run advances.
     * @param firmware Program to run in lockstep with it.
     */
    Simulation(RunContext& context, FirmwareThread& firmware);

    /**
     * @brief Add a listener, called in registration order.
     *
     * @param listener Listener to add; it must outlive the run.
     */
    void add_listener(IRunListener& listener);

    /**
     * @brief Run until the tick budget is spent, the program ends or a listener quits.
     *
     * @param ticks Number of firmware ticks to run.
     */
    void run(uint64_t ticks);

    /**
     * @brief Get the tick currently being executed.
     *
     * @return Index of the tick.
     */
    uint64_t tick() const { return this->current_tick; }

    /**
     * @brief Get how many ticks actually ran.
     *
     * @return Number of completed ticks.
     */
    uint64_t completed_ticks() const { return this->completed; }

    /**
     * @brief Get the simulation being advanced.
     *
     * @return The context.
     */
    const RunContext& context() const { return this->simulation; }

private:
    /**
     * @brief Ask every listener whether the next tick may run.
     *
     * @note Every listener is asked, even after one has refused: a frontend
     *       that pauses does so inside its own call, and skipping the rest
     *       would leave them out of step with the tick.
     *
     * @return QUIT when any listener asked for it.
     */
    RunControl before_tick();

    /**
     * @brief Advance the physics and the clock by one tick.
     *
     * @note The devices act on the world first, from what the firmware wrote
     *       during the tick, and sample it last, into what it will read.
     */
    void step();

    /**
     * @brief Simulation being advanced.
     *
     * @note Bound for the life of the run; the context and the program both
     *       outlive every simulation that drives them.
     */
    RunContext& simulation;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Program running in lockstep.
     *
     * @note Bound for the life of the run; the context and the program both
     *       outlive every simulation that drives them.
     */
    FirmwareThread& firmware;  // NOLINT(*-avoid-const-or-ref-data-members)

    /**
     * @brief Listeners, in registration order.
     */
    std::vector<IRunListener*> listeners;

    /**
     * @brief Tick currently being executed.
     */
    uint64_t current_tick{0};

    /**
     * @brief Number of ticks that completed.
     */
    uint64_t completed{0};
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_SIMULATION_HPP

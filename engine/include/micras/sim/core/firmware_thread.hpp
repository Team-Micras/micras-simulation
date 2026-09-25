/**
 * @file
 *
 * @brief Runs the firmware program in its own thread, one tick at a time.
 */

#ifndef MICRAS_SIM_CORE_FIRMWARE_THREAD_HPP
#define MICRAS_SIM_CORE_FIRMWARE_THREAD_HPP

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>

namespace micras::sim {
/**
 * @brief Thrown inside the firmware thread to unwind it when the run ends.
 *
 * @note The firmware program is an endless loop, so the only way out is to
 *       throw at the point where it yields. Unwinding runs its destructors,
 *       which is what flushes anything the program still holds.
 */
class RunFinished : public std::exception {
public:
    const char* what() const noexcept override { return "the simulation run finished"; }
};

/**
 * @brief Strict handoff between the simulation and the firmware program.
 *
 * @note The two threads never run at the same time: exactly one of them is
 *       awake at any moment, so the run stays as deterministic as it was when
 *       the firmware was called inline. The firmware gives control back from
 *       inside its own loop, through yield_tick(), which is reached from the
 *       Stopwatch the firmware waits on.
 *
 * @note The simulation side is single threaded: run_until_yield() and finish()
 *       must be called from the one thread that drives the run.
 */
class FirmwareThread {
public:
    /**
     * @brief The firmware program to run, typically the Micras loop.
     */
    using Program = std::function<void()>;

    /**
     * @brief Take the program to run.
     *
     * @param program Endless loop to run in the firmware thread.
     */
    explicit FirmwareThread(Program program);

    FirmwareThread(const FirmwareThread&) = delete;
    FirmwareThread(FirmwareThread&&) = delete;
    FirmwareThread& operator=(const FirmwareThread&) = delete;
    FirmwareThread& operator=(FirmwareThread&&) = delete;

    /**
     * @brief Stop the program and join the thread.
     */
    ~FirmwareThread();

    /**
     * @brief Longest a program may run without yielding, in wall seconds.
     *
     * @note The only wall clock in the harness, and it never affects a healthy
     *       run: it exists so a program that stops yielding is reported instead
     *       of hanging the process forever.
     */
    static constexpr int watchdog_seconds{30};

    /**
     * @brief Let the firmware run until it yields or finishes.
     *
     * @note Starts the thread on the first call. Blocks the calling thread for
     *       the whole time the firmware is awake. Throws if the program runs
     *       for longer than the watchdog allows without yielding.
     */
    void run_until_yield();

    /**
     * @brief Hand control back to the simulation and wait to be resumed.
     *
     * @note Called from the firmware thread only. Throws RunFinished once the
     *       simulation has asked the program to stop, and returns without
     *       handing the turn over if the program is already unwinding: a
     *       destructor that waits for time to pass must not park a thread the
     *       simulation is no longer listening to.
     *
     * @return False when the tick was not actually handed over, so a caller
     *         waiting for time to pass knows it never will.
     */
    bool yield_tick();

    /**
     * @brief Ask the program to unwind and join the thread.
     *
     * @note A program that catches RunFinished and carries on cannot be
     *       unwound, because nothing in C++ can force a thread to return, and
     *       letting it outlive this object would be a use after free. The run
     *       is invalid at that point, so the process is ended with a message
     *       naming the cause rather than left to race.
     */
    void finish();

    /**
     * @brief Check whether the program returned or threw on its own.
     *
     * @return True once the firmware thread is no longer running the program.
     */
    bool has_finished() const;

    /**
     * @brief Rethrow whatever the program threw, if anything.
     */
    void rethrow_any_error() const;

private:
    /**
     * @brief Which thread is allowed to run.
     */
    enum class Turn : uint8_t {
        SIMULATION,
        FIRMWARE,
    };

    /**
     * @brief Body of the firmware thread.
     *
     * @note Not called main: the hardware tests are compiled with that
     *       identifier redefined, so nothing reachable from them may use it.
     */
    void thread_body();

    /**
     * @brief Hand the turn over and wait for it to come back.
     *
     * @param lock Held lock on the handoff mutex.
     * @param to Thread that may run next.
     * @param until Thread to wait for.
     * @param watched Whether to give up after the watchdog time.
     * @return False only when the watchdog gave up.
     */
    bool hand_over(std::unique_lock<std::mutex>& lock, Turn to, Turn until, bool watched = false);

    /**
     * @brief Program the thread runs.
     */
    Program program;

    /**
     * @brief The firmware thread, started on the first run_until_yield().
     */
    std::thread thread;

    /**
     * @brief Guards every field below.
     */
    mutable std::mutex mutex;

    /**
     * @brief Signals a change of turn.
     */
    std::condition_variable handoff;

    /**
     * @brief Thread currently allowed to run.
     */
    Turn turn{Turn::SIMULATION};

    /**
     * @brief Set once the simulation asked the program to unwind.
     */
    bool stopping{false};

    /**
     * @brief Set once the program returned, threw or unwound.
     */
    bool finished{false};

    /**
     * @brief Error the program threw, rethrown on the simulation thread.
     */
    std::exception_ptr error;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_FIRMWARE_THREAD_HPP

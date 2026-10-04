/**
 * @file
 */

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "micras/sim/core/firmware_thread.hpp"

namespace micras::sim {
FirmwareThread::FirmwareThread(Program program) : program{std::move(program)} { }

FirmwareThread::~FirmwareThread() {
    this->finish();
}

void FirmwareThread::run_until_yield() {
    std::unique_lock lock(this->mutex);

    if (this->finished) {
        return;
    }

    if (not this->thread.joinable()) {
        this->thread = std::thread(&FirmwareThread::thread_body, this);
    }

    if (not this->hand_over(lock, Turn::FIRMWARE, Turn::SIMULATION, true)) {
        throw std::runtime_error(
            "the program ran for " + std::to_string(watchdog_seconds) +
            " s without handing the tick back; it is waiting for something the simulation cannot deliver"
        );
    }
}

bool FirmwareThread::yield_tick() {
    std::unique_lock lock(this->mutex);

    if (this->stopping) {
        if (std::uncaught_exceptions() > 0) {
            return false;
        }

        throw RunFinished();
    }

    this->hand_over(lock, Turn::SIMULATION, Turn::FIRMWARE);

    if (this->stopping) {
        throw RunFinished();
    }

    return true;
}

void FirmwareThread::finish() {
    {
        std::unique_lock lock(this->mutex);
        this->stopping = true;

        if (not this->thread.joinable()) {
            return;
        }

        if (not this->finished and not this->hand_over(lock, Turn::FIRMWARE, Turn::SIMULATION, true)) {
            std::cerr << "the program ignored the request to stop, so the run is invalid\n" << std::flush;
            std::_Exit(EXIT_FAILURE);
        }
    }

    this->thread.join();
}

bool FirmwareThread::has_finished() const {
    const std::scoped_lock lock(this->mutex);
    return this->finished;
}

void FirmwareThread::rethrow_any_error() const {
    const std::scoped_lock lock(this->mutex);

    if (this->error) {
        std::rethrow_exception(this->error);
    }
}

bool FirmwareThread::hand_over(std::unique_lock<std::mutex>& lock, Turn to, Turn until, bool watched) {
    const auto ready = [this, until] { return this->turn == until or this->finished; };

    this->turn = to;
    this->handoff.notify_all();

    if (not watched) {
        this->handoff.wait(lock, ready);
        return true;
    }

    return this->handoff.wait_for(lock, std::chrono::seconds(watchdog_seconds), ready);
}

void FirmwareThread::thread_body() {
    {
        std::unique_lock lock(this->mutex);
        this->handoff.wait(lock, [this] { return this->turn == Turn::FIRMWARE; });
    }

    try {
        this->program(*this);
    } catch (const RunFinished&) {  // NOLINT(bugprone-empty-catch): the expected way out, see finish().
    } catch (...) {
        const std::scoped_lock lock(this->mutex);
        this->error = std::current_exception();
    }

    const std::scoped_lock lock(this->mutex);
    this->finished = true;
    this->turn = Turn::SIMULATION;
    this->handoff.notify_all();
}
}  // namespace micras::sim

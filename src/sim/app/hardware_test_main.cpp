/**
 * @file
 *
 * @brief Runs one firmware hardware test in the deterministic MuJoCo harness.
 */

#include <span>

#include "micras/sim/app/application.hpp"

/**
 * @brief The hardware test's own entry point, renamed at compile time.
 *
 * @param argc Number of main arguments.
 * @param argv Main arguments.
 * @return The test's exit code, which the harness ignores.
 */
extern int micras_test_main(int argc, char** argv);

int main(int argc, char** argv) {
    return micras::sim::Application::main(
        std::span(argv, static_cast<std::size_t>(argc)), [argc, argv] { micras_test_main(argc, argv); },
        micras::sim::PoolTelemetry::NOT_SERVED
    );
}

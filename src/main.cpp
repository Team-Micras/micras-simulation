/**
 * @file
 *
 * @brief Runs the Micras firmware loop in the deterministic MuJoCo harness.
 */

#include <span>

#include "micras/micras.hpp"
#include "micras/sim/app/application.hpp"

int main(int argc, char** argv) {
    return micras::sim::Application::main(std::span(argv, static_cast<std::size_t>(argc)), [] {
        micras::Micras micras;

        while (true) {
            micras.update();
        }
    });
}

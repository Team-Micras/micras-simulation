/**
 * @file
 */

#include <cstddef>
#include <span>

#include "micras/sim/app/application.hpp"
#include "micras/sim/toy/toy_target.hpp"

int main(int argc, char** argv) {
    micras::sim::ToyTarget target;
    return micras::sim::run(std::span(argv, static_cast<std::size_t>(argc)), target);
}

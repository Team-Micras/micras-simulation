/**
 * @file
 */

#include "micras/sim/core/panel_spec.hpp"

namespace micras::sim {
std::string StateLabel::name_of(double state) const {
    if (not(state >= 0.0) or state >= static_cast<double>(this->names.size())) {
        return "?";
    }

    return this->names.at(static_cast<std::size_t>(state));
}
}  // namespace micras::sim

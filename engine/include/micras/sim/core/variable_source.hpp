/**
 * @file
 *
 * @brief Named firmware variables, as whatever decodes them reports them.
 */

#ifndef MICRAS_SIM_CORE_VARIABLE_SOURCE_HPP
#define MICRAS_SIM_CORE_VARIABLE_SOURCE_HPP

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace micras::sim {
/**
 * @brief Names of the values of the variables that hold a state, by variable, starting at zero.
 */
using StateNames = std::map<std::string, std::vector<std::string>, std::less<>>;

/**
 * @brief Looks up the current value of a firmware variable by name.
 *
 * @note What the panel plots, the video overlay prints, the event log watches
 *       and the scenario conditions wait for. The engine never knows how the
 *       values are obtained; a robot target implements this over its own
 *       telemetry.
 */
class VariableSource {
public:
    VariableSource() = default;

    VariableSource(const VariableSource&) = delete;
    VariableSource(VariableSource&&) = delete;
    VariableSource& operator=(const VariableSource&) = delete;
    VariableSource& operator=(VariableSource&&) = delete;

    virtual ~VariableSource() = default;

    /**
     * @brief Get the current value of a variable.
     *
     * @param name Firmware name of the variable.
     * @return Its value, or NaN when it is unknown or not reported yet.
     */
    virtual double value_of(const std::string& name) const = 0;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_VARIABLE_SOURCE_HPP

/**
 * @file
 *
 * @brief Named firmware variables, as whatever decodes them reports them.
 */

#ifndef MICRAS_SIM_CORE_VARIABLE_SOURCE_HPP
#define MICRAS_SIM_CORE_VARIABLE_SOURCE_HPP

#include <string>

namespace micras::sim {
/**
 * @brief Looks up the current value of a firmware variable by name.
 *
 * @note What the panel plots and the video overlay prints. The engine never
 *       knows how the values are obtained; a robot target implements this over
 *       its own telemetry.
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

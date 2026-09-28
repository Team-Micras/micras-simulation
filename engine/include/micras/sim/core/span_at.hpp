/**
 * @file
 *
 * @brief Checked element access for std::span, which has no at() before C++26.
 */

#ifndef MICRAS_SIM_CORE_SPAN_AT_HPP
#define MICRAS_SIM_CORE_SPAN_AT_HPP

#include <cstddef>
#include <iterator>
#include <span>
#include <stdexcept>

namespace micras::sim {
/**
 * @brief Get an element of a span, checking the index as std::vector::at does.
 *
 * @note MuJoCo's arrays reach the simulator as spans over its buffers; an index past the end is a bug
 *       in the caller, reported as an exception instead of a read out of bounds.
 *
 * @tparam T Element type.
 * @tparam Extent Extent of the span.
 * @param values Span to read.
 * @param index Index of the element.
 * @return Reference to the element.
 */
template <typename T, std::size_t Extent>
constexpr T& at(std::span<T, Extent> values, std::size_t index) {
    if (index >= values.size()) {
        throw std::out_of_range("span index out of range");
    }

    return *std::next(values.begin(), static_cast<std::ptrdiff_t>(index));
}
}  // namespace micras::sim

#endif  // MICRAS_SIM_CORE_SPAN_AT_HPP

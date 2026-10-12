#ifndef KOMORI_THRESHOLD_MATH_HPP_
#define KOMORI_THRESHOLD_MATH_HPP_
#include <algorithm>
#include <cstdint>

namespace komori::threshold_math {
// Preconditions: maximum >= 1; factor is finite and >= 1. All engine callers
// use fixed root factors or 1.7 + 0.3 * uint32 thread_id, satisfying this domain.
// Keep the original double multiplication and truncation in the ordinary range.
inline std::uint32_t Expand(std::uint32_t value, std::uint32_t current,
                            double factor, std::uint32_t maximum) {
  const double scaled = static_cast<double>(value) * factor;
  // Saturate before conversion/+1; also prevent an inverted clamp interval.
  const std::uint32_t next = scaled >= static_cast<double>(maximum - 1)
      ? maximum : static_cast<std::uint32_t>(scaled) + 1;
  return std::max(std::min(current, maximum), next);
}
}  // namespace komori::threshold_math
#endif

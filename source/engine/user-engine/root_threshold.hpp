#ifndef KOMORI_ROOT_THRESHOLD_HPP_
#define KOMORI_ROOT_THRESHOLD_HPP_
#include "typedefs.hpp"
#include "threshold_math.hpp"
#ifndef KOMORI_ROOT_THRESHOLD_TENTHS
#define KOMORI_ROOT_THRESHOLD_TENTHS 17
#endif
namespace komori::root_threshold {
inline constexpr int kTenths = KOMORI_ROOT_THRESHOLD_TENTHS;
static_assert(kTenths == 13 || kTenths == 15 || kTenths == 17 || kTenths == 20);
constexpr bool IsSearchRoot(std::uint32_t threads, std::uint32_t id, Depth depth, bool pv) {
  return threads == 1 && id == 0 && depth == 0 && !pv;
}
inline PnDn Expand(PnDn value, PnDn current, double factor) {
  return threshold_math::Expand(value, current, factor, kInfinitePnDn);
}
} // namespace komori::root_threshold
#endif

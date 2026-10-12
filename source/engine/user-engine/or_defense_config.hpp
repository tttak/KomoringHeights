#ifndef KOMORI_OR_DEFENSE_CONFIG_HPP_
#define KOMORI_OR_DEFENSE_CONFIG_HPP_
#include <cstdint>
#ifndef KOMORI_OR_DEFENSE
#define KOMORI_OR_DEFENSE 2
#endif
namespace komori {
inline constexpr std::uint32_t kOrDefenseBase = KOMORI_OR_DEFENSE;
static_assert(kOrDefenseBase == 0 || kOrDefenseBase == 2, "T009 compares only0/2");
}
#if defined(ENABLE_INITIAL_ESTIMATE_STATS) && !defined(ENABLE_TT_STATS)
#error ENABLE_INITIAL_ESTIMATE_STATS requires ENABLE_TT_STATS
#endif
#endif

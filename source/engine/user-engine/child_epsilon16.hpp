/**
 * @file child_epsilon16.hpp
 * @brief AND 子探索の 1+ε しきい値（ε = 1/16 固定）の整数算術。USI オプション `ChildEpsilon16` で有効化する
 */
#ifndef KOMORI_CHILD_EPSILON16_HPP_
#define KOMORI_CHILD_EPSILON16_HPP_

#include <algorithm>
#include <cstdint>

#include "typedefs.hpp"

namespace komori {
namespace child_epsilon16 {
/**
 * @brief 子の phi しきい値 min(parent_thphi, kInfinitePnDn, x + max(1, ceil(x/16))) を返す（x = second_phi）
 * @pre 0 < second_phi < kInfinitePnDn（呼出し側で適用条件を確認すること）
 *
 * 加算と比較は 64bit で行い、上限処理の後に PnDn へ戻すので、オーバーフローしない。
 */
inline PnDn ExpandedChildThphi(PnDn parent_thphi, PnDn second_phi) {
  const std::uint64_t x = second_phi;
  const std::uint64_t step = std::max<std::uint64_t>(1, x / 16 + (x % 16 != 0 ? 1 : 0));
  const std::uint64_t expanded = x + step;
  const std::uint64_t cap = std::min<std::uint64_t>(parent_thphi, kInfinitePnDn);
  return static_cast<PnDn>(std::min(cap, expanded));
}
}  // namespace child_epsilon16
}  // namespace komori

#endif  // KOMORI_CHILD_EPSILON16_HPP_

#ifndef KOMORI_TT_READ_LOCK_POLICY_HPP_
#define KOMORI_TT_READ_LOCK_POLICY_HPP_

#include <cstddef>
#include <shared_mutex>
#include <type_traits>

namespace komori::tt::read_lock_policy {
// Only MainThread::search sets this, after the previous slaves finish and
// before NewSearch/starting slaves. ThreadPool mutex/cv publishes the mode.
// It stays immutable throughout search and PV construction. Default: locked.
inline bool required = true;
inline void BeginSearch(std::size_t actual_threads) noexcept {
  required = actual_threads != 1;
}

class UnlockedReadGuard {
 public:
  template <typename Entry>
  explicit UnlockedReadGuard(Entry&) noexcept {}
};

template <bool kRequired, typename Entry>
using Guard = std::conditional_t<kRequired, std::shared_lock<Entry>, UnlockedReadGuard>;
}  // namespace komori::tt::read_lock_policy

#endif  // KOMORI_TT_READ_LOCK_POLICY_HPP_

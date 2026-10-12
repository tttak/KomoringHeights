/**
 * @file search_monitor.hpp
 */
#ifndef KOMORI_SEARCH_MONITOR_HPP_
#define KOMORI_SEARCH_MONITOR_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>

#include "../../thread.h"
#include "circular_array.hpp"
#include "periodic_alarm.hpp"
#include "typedefs.hpp"
#include "usi_info.hpp"

namespace komori {
/// GC をするタイミング。置換表使用率がこの値を超えていたら GC を行う。
constexpr double kExecuteGcHashRate = 0.5;
static_assert(kExecuteGcHashRate > 0 && kExecuteGcHashRate < 1.0,
              "kExecuteGcHashRate must be greater than 0 and less than 1");
/// GC をする Hashfull のしきい値
constexpr int kExecuteGcHashfullThreshold = std::max(static_cast<int>(1000 * kExecuteGcHashRate), 1);

namespace detail {
/// HashfullCheck をスキップする回数の割合
constexpr std::uint32_t kHashfullCheeckSkipRatio = 4096;
/// HashfullCheck の周期を計算する
constexpr std::uint64_t HashfullCheckInterval(std::uint64_t capacity) noexcept {
  return static_cast<std::uint64_t>(static_cast<double>(capacity) * (1.0 - kExecuteGcHashRate));
}
}  // namespace detail

/**
 * @brief 探索局面数を観測して nps を計算したり探索中断の判断をしたりするクラス。
 */
class SearchMonitor {
 public:
  // User/budget stops are separate from internal DispatchSearch synchronization.
  enum StopReason : std::uint32_t { kTimeLimit = 1, kNodeLimit = 2, kExternalStop = 4 };
  struct FinalStatistics {
    std::uint64_t nodes;
    std::uint64_t time_us;
    std::uint64_t nps;
    std::uint32_t stop_reasons;
  };
  /**
   * @brief 変数を初期化して探索を開始する。
   * @param tt_capacity 置換表のサイズ
   * @param pv_interval PV出力の間隔[ms]
   * @param move_limit 探索局面数の上限
   */
  void NewSearch(std::uint64_t tt_capacity, std::uint64_t pv_interval, std::uint64_t move_limit) {
    start_time_ = std::chrono::steady_clock::now();
    max_depth_ = 0;

    tp_hist_.Clear();
    mc_hist_.Clear();
    hist_idx_ = 0;

    move_limit_ = move_limit;
    if (Search::Limits.mate > 0) {
      time_limit_ = Search::Limits.mate;
    } else if (Search::Limits.movetime > 0) {
      time_limit_ = Search::Limits.movetime;
    } else {
      time_limit_ = std::numeric_limits<TimePoint>::max();
    }

    hashfull_check_interval_ = detail::HashfullCheckInterval(tt_capacity);
    ResetNextHashfullCheck();

    print_alarm_.Start(pv_interval);
    stop_check_.Start(100);
    stop_.store(false, std::memory_order_release);
    stop_reasons_.store(0, std::memory_order_relaxed);
  }

  void Stop() { stop_.store(true, std::memory_order_release); }
  void ResetStop() {
    stop_.store(CheckLimits(), std::memory_order_release);
  }

  /// Snapshot after all workers finish; NPS is the full-run average.
  FinalStatistics FinalInfo() const {
    const auto nodes = MoveCount();
    const auto time_us = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start_time_).count());
    const auto nps = time_us == 0 ? 0 : static_cast<std::uint64_t>(
        static_cast<long double>(nodes) * 1000000.0L / static_cast<long double>(time_us));
    return {nodes, time_us, nps, stop_reasons_.load(std::memory_order_relaxed)};
  }

  /**
   * @brief 深さ `depth` の局面に訪れたことを報告する
   * @param depth 深さ
   */
  void Visit(Depth depth) {
    const auto curr_max_depth = max_depth_.load(std::memory_order_relaxed);
    if (depth > curr_max_depth) {
      max_depth_.store(depth, std::memory_order_relaxed);
    }
  }

  /**
   * @brief 現在の探索情報を `UsiInfo` に詰めて返す。
   * @return 現在の探索情報
   */
  UsiInfo GetInfo() const {
    const auto curr_time = std::chrono::steady_clock::now();
    const auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - start_time_).count();

    const auto move_count = MoveCount();
    std::uint64_t nps = 0;
    if (hist_idx_ >= kHistLen) {
      const auto tp = tp_hist_[hist_idx_];
      const auto mc = mc_hist_[hist_idx_];
      const auto tp_diff = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - tp).count();
      nps = (move_count - mc) * 1000ULL / tp_diff;
    } else {
      if (time_ms > 0) {
        nps = move_count * 1000ULL / time_ms;
      }
    }

    UsiInfo output;
    output.Set(UsiInfoKey::kSelDepth, max_depth_.load(std::memory_order_relaxed));
    output.Set(UsiInfoKey::kTime, time_ms);
    output.Set(UsiInfoKey::kNodes, move_count);
    output.Set(UsiInfoKey::kNps, nps);

    return output;
  }

  /// 現在の探索局面数
  std::uint64_t MoveCount() const { return Threads.nodes_searched(); }
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
  // Read-only root-boundary snapshot: no new clock/budget polling locations.
  bool HasStopRequest() const { return stop_.load(std::memory_order_acquire) || Threads.stop; }
  std::uint32_t StopFlags() const { return stop_reasons_.load(std::memory_order_relaxed); }
#endif
  /// 今すぐ置換表使用率をチェックすべきなら true
  bool ShouldCheckHashfull() {
    hashfull_check_skip_--;
    if (hashfull_check_skip_ > 0) {
      return false;
    }

    hashfull_check_skip_ = detail::kHashfullCheeckSkipRatio;
    return MoveCount() >= next_hashfull_check_;
  }

  /// 次回の置換表使用率チェックタイミングを更新する
  void ResetNextHashfullCheck() {
    hashfull_check_skip_ = detail::kHashfullCheeckSkipRatio;
    next_hashfull_check_ = MoveCount() + hashfull_check_interval_;
  }

  /// 今すぐ探索をやめるべきなら true
  bool ShouldStop() {
    const auto stop = stop_.load(std::memory_order_acquire);
    if (stop) {
      return true;
    } else if (tl_thread_id != 0 || !stop_check_.Tick()) {
      return false;
    }

    // stop_ かどうか改めて判定し直す
    stop_.store(CheckLimits(), std::memory_order_release);
    return stop_;
  }

  // Reconstruction is outside the recursive search hot path. Check its budget
  // at every extension rather than waiting for the search's periodic alarm.
  bool ShouldStopPv() {
    if (CheckLimits()) stop_.store(true, std::memory_order_release);
    return stop_.load(std::memory_order_acquire);
  }

  /// 今すぐ評価値を出力すべきかどうか。定期的に呼び出す必要がある。
  bool ShouldPrint() {
    if (!print_alarm_.Tick()) {
      return false;
    }
    // print のタイミングで nps も更新しておく
    tp_hist_[hist_idx_] = std::chrono::steady_clock::now();
    mc_hist_[hist_idx_] = MoveCount();
    hist_idx_++;

    return true;
  }

 private:
  bool CheckLimits() {
    // Preserve existing check locations/cadence and latch simultaneous reasons.
    const auto elapsed = Time.elapsed_from_ponderhit();
    std::uint32_t reasons = 0;
    if (MoveCount() >= move_limit_) reasons |= kNodeLimit;
    if (elapsed >= time_limit_) reasons |= kTimeLimit;
    if (Threads.stop) reasons |= kExternalStop;
    if (reasons != 0) stop_reasons_.fetch_or(reasons, std::memory_order_relaxed);
    return reasons != 0;
  }
  /// nps の計算のために保持する探索局面数の履歴数
  static constexpr inline std::size_t kHistLen = 16;

  std::chrono::steady_clock::time_point start_time_;  ///< 探索開始時刻

  CircularArray<std::chrono::steady_clock::time_point, kHistLen> tp_hist_;  ///< mc_hist_ を観測した時刻
  CircularArray<std::uint64_t, kHistLen> mc_hist_;                          ///< 各時点での探索局面数
  std::size_t hist_idx_;  ///< `tp_hist_` と `mc_hist_` の現在の添字

  std::uint64_t move_limit_;               ///< 探索局面数の上限
  TimePoint time_limit_;                   ///< 探索の時間制限[ms]
  std::uint64_t hashfull_check_interval_;  ///< 置換表使用率をチェックする周期[探索局面数]
  std::uint32_t hashfull_check_skip_;      ///< next_hashfull_check_ のチェックをスキップする回数
  std::uint64_t next_hashfull_check_;  ///< 次に置換表使用率をチェックするタイミング[探索局面数]

  PeriodicAlarm print_alarm_;  ///< PV出力用のタイマー
  PeriodicAlarm stop_check_;   ///< 探索停止判断用のタイマー

  alignas(64) std::atomic<bool> stop_;  ///< 探索中止状態かどうか
  std::atomic<Depth> max_depth_;        ///< 最大探索深さ
  std::atomic<std::uint32_t> stop_reasons_{0};
};
}  // namespace komori

#endif  // KOMORI_SEARCH_MONITOR_HPP_

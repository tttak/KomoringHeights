#ifndef KOMORI_ROOT_ITERATION_STATS_HPP_
#define KOMORI_ROOT_ITERATION_STATS_HPP_
#ifdef ENABLE_ROOT_ITERATION_STATS
#ifndef ENABLE_TT_STATS
#error ENABLE_ROOT_ITERATION_STATS requires ENABLE_TT_STATS
#endif
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>
#include "tt_stats.hpp"

namespace komori::root_iteration_stats {
#define ROOT_ITER_COUNTERS(X) \
 X(expansion_first_by_tt) X(expansion_revisit_by_tt) X(expansion_immediate_return) \
 X(lookup_calls) X(effective_queries) X(boundary_queries) X(proven_queries) X(disproven_queries) X(repetition_queries) \
 X(impl_calls) X(impl_depth_returns) X(impl_final_returns) X(impl_threshold_returns) X(impl_other_returns) X(impl_tca_breaks)
struct Counts {
#define RI_DECLARE(n) std::uint64_t n{};
  ROOT_ITER_COUNTERS(RI_DECLARE)
#undef RI_DECLARE
  static Counts Snapshot() {
    Counts x;
    if (tt_stats::current) {
#define RI_READ(n) x.n = tt_stats::current->n;
      ROOT_ITER_COUNTERS(RI_READ)
#undef RI_READ
    }
    return x;
  }
  Counts Since(const Counts& before) const {
    Counts x;
#define RI_DIFF(n) x.n = n - before.n;
    ROOT_ITER_COUNTERS(RI_DIFF)
#undef RI_DIFF
    return x;
  }
};
struct Row {
  std::uint64_t call{}, iteration{}, nodes_begin{}, nodes_end{}, time_begin_us{}, time_end_us{};
  std::uint32_t pn_begin{}, dn_begin{}, pn_end{}, dn_end{}, thpn_begin{}, thdn_begin{}, thpn_after{}, thdn_after{};
  std::uint32_t effective_thpn_return{}, effective_thdn_return{}, stop_flags{};
  const char* state = "unknown";
  bool extended{}, tca_break{};
  Counts counts;
};
// Single main-thread writer. Output only after workers finish. Bounded storage.
class Trace {
 public:
  void Reset() { rows_.clear(); rows_.reserve(512); call_=iteration_=dropped_=0; active_=false; }
  void NewCall() { ++call_; iteration_=0; }
  bool Active() const { return active_; }
  void Begin(std::uint64_t nodes, std::uint64_t time_us, std::uint32_t pn, std::uint32_t dn,
             std::uint32_t thpn, std::uint32_t thdn) {
    ++iteration_;
    if (rows_.size() >= 4096) { ++dropped_; active_=false; return; }
    pending_ = {}; pending_.call=call_; pending_.iteration=iteration_;
    pending_.nodes_begin=nodes; pending_.time_begin_us=time_us;
    pending_.pn_begin=pn; pending_.dn_begin=dn; pending_.thpn_begin=thpn; pending_.thdn_begin=thdn;
    pending_.effective_thpn_return=thpn; pending_.effective_thdn_return=thdn;
    before_=Counts::Snapshot(); active_=true;
  }
  void RootReturn(std::uint32_t thpn, std::uint32_t thdn, bool tca_break) {
    if (!active_) return;
    pending_.effective_thpn_return=thpn; pending_.effective_thdn_return=thdn; pending_.tca_break=tca_break;
  }
  void Finish(std::uint64_t nodes, std::uint64_t time_us, std::uint32_t pn, std::uint32_t dn,
              std::uint32_t thpn, std::uint32_t thdn, std::uint32_t stop_flags, const char* state, bool extended) {
    if (!active_) return;
    pending_.nodes_end=nodes; pending_.time_end_us=time_us; pending_.pn_end=pn; pending_.dn_end=dn;
    pending_.thpn_after=thpn; pending_.thdn_after=thdn; pending_.stop_flags=stop_flags;
    pending_.state=state; pending_.extended=extended; pending_.counts=Counts::Snapshot().Since(before_);
    rows_.push_back(pending_); active_=false;
  }
  std::vector<std::string> Lines(int factor_tenths) const {
    std::vector<std::string> output;
    for (const auto& r : rows_) {
      std::ostringstream s;
      s << "{\"schema\":1,\"thread\":0,\"phase\":\"search\",\"factor_tenths\":" << factor_tenths
        << ",\"call\":" << r.call << ",\"iteration\":" << r.iteration;
#define RI_FIELD(n) s << ",\"" #n "\":" << r.n;
      RI_FIELD(nodes_begin) RI_FIELD(nodes_end) RI_FIELD(time_begin_us) RI_FIELD(time_end_us)
      RI_FIELD(pn_begin) RI_FIELD(dn_begin) RI_FIELD(pn_end) RI_FIELD(dn_end)
      RI_FIELD(thpn_begin) RI_FIELD(thdn_begin) RI_FIELD(thpn_after) RI_FIELD(thdn_after)
      RI_FIELD(effective_thpn_return) RI_FIELD(effective_thdn_return) RI_FIELD(stop_flags)
      RI_FIELD(extended) RI_FIELD(tca_break)
#undef RI_FIELD
      s << ",\"nodes\":" << r.nodes_end-r.nodes_begin << ",\"duration_us\":" << r.time_end_us-r.time_begin_us
        << ",\"root_state\":\"" << r.state << "\"";
#define RI_COUNT(n) s << ",\"" #n "\":" << r.counts.n;
      ROOT_ITER_COUNTERS(RI_COUNT)
#undef RI_COUNT
      s << '}'; output.push_back(s.str());
    }
    return output;
  }
  std::size_t Size() const { return rows_.size(); }
  std::uint64_t Dropped() const { return dropped_; }
 private:
  std::vector<Row> rows_;
  Row pending_;
  Counts before_;
  std::uint64_t call_{}, iteration_{}, dropped_{};
  bool active_{};
};
} // namespace komori::root_iteration_stats
#endif
#endif

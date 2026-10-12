#ifndef KOMORI_TT_STATS_HPP_
#define KOMORI_TT_STATS_HPP_

// No types, storage, TLS accesses, or calls survive in the ordinary build.
#ifdef ENABLE_TT_STATS
#include <array>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace komori::tt_stats {
enum class Phase : std::uint32_t { kSetup, kSearch, kPv };
enum class Relation { kExact, kSuperior, kInferior };
struct Probe {
  std::uint64_t calls{}, slots{}, maximum{};
  std::array<std::uint64_t, 11> histogram{};  // 0,1,2,3-4,5-8,...,129-256,257+
  void Add(std::uint64_t n) {
    ++calls; slots += n; if (n > maximum) maximum = n;
    std::size_t bucket = n <= 2 ? n : 3;
    if (n > 2) { std::uint64_t end = 4; while (n > end && bucket < 10) { ++bucket; end *= 2; } }
    ++histogram[bucket];
  }
  void Merge(const Probe& x) {
    calls += x.calls; slots += x.slots; if (x.maximum > maximum) maximum = x.maximum;
    for (std::size_t i = 0; i < histogram.size(); ++i) histogram[i] += x.histogram[i];
  }
};
#ifdef ENABLE_INITIAL_ESTIMATE_STATS
#define KOMORI_INITIAL_COUNTERS(X) \
 X(initial_merge_calls) X(initial_or_calls) X(initial_and_calls) X(or_defense_branches) \
 X(initial_only_wins) X(tt_only_wins) X(initial_tt_mixed) X(initial_tt_equal) \
 X(initial_raises_pn) X(initial_raises_dn) X(tt_greater_pn) X(tt_greater_dn) \
 X(defense_initial_pn_greater) X(defense_initial_pn_equal) X(defense_tt_pn_greater) \
 X(defense_coefficient_survives) X(exact_initial_bypasses)
#else
#define KOMORI_INITIAL_COUNTERS(X)
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
#define KOMORI_ITER_COUNTERS(X) \
 X(impl_calls) X(impl_depth_returns) X(impl_final_returns) X(impl_threshold_returns) X(impl_other_returns) X(impl_tca_breaks)
#else
#define KOMORI_ITER_COUNTERS(X)
#endif
#define KOMORI_TT_COUNTERS(X) \
 KOMORI_ITER_COUNTERS(X) \
 KOMORI_INITIAL_COUNTERS(X) \
 X(lookup_calls) X(strict_requested) X(exact_entries) X(superior_entries) X(inferior_entries) \
 X(exact_queries) X(superior_queries) X(inferior_queries) X(accepted_queries) \
 X(exact_effective_queries) X(superior_effective_queries) X(inferior_effective_queries) \
 X(effective_queries) X(no_effective_queries) X(boundary_entries) X(boundary_queries) \
 X(metadata_entries) X(proven_queries) X(disproven_queries) X(repetition_queries) X(initial_eval_calls) \
 X(strict_len_updates) X(update_calls) X(new_entries) X(cached_updates) X(existing_updates) \
 X(regular_gc_calls) X(regular_deleted) X(regular_moved) X(repetition_gc_calls) X(repetition_deleted) X(repetition_moved) \
 X(expansion_first_by_tt) X(expansion_revisit_by_tt) X(expansion_immediate_return) \
 X(expansion_frames) X(relookup_calls) X(entry_calls) X(entry_iterations) X(threshold_extensions) \
 X(root_entry_calls) X(root_iterations) X(root_extensions)
struct Counters {
#define TT_DECLARE(name) std::uint64_t name{};
  KOMORI_TT_COUNTERS(TT_DECLARE)
#undef TT_DECLARE
  Probe primary, strict_extra, write;
  void Merge(const Counters& x) {
#define TT_MERGE(name) name += x.name;
    KOMORI_TT_COUNTERS(TT_MERGE)
#undef TT_MERGE
    primary.Merge(x.primary); strict_extra.Merge(x.strict_extra); write.Merge(x.write);
  }
};
struct alignas(64) ThreadCounters { std::array<Counters, 3> phases{}; };
inline std::vector<ThreadCounters> banks;
inline thread_local ThreadCounters* thread_bank = nullptr;
inline thread_local Counters* current = nullptr;
#ifdef ENABLE_INITIAL_ESTIMATE_STATS
// The synchronous evaluation callback and its Query use this thread's bank.
inline thread_local bool initial_is_or = false, initial_defense_branch = false;
inline void RecordInitialMerge(std::uint32_t pn, std::uint32_t dn,
                               std::uint32_t ipn, std::uint32_t idn, std::uint32_t defense_base) {
  if (!current) return;
  auto& c = *current;
  ++c.initial_merge_calls;
  c.initial_or_calls += initial_is_or; c.initial_and_calls += !initial_is_or;
  const bool iw = ipn > pn || idn > dn, tw = pn > ipn || dn > idn;
  c.initial_only_wins += iw && !tw; c.tt_only_wins += tw && !iw;
  c.initial_tt_mixed += iw && tw; c.initial_tt_equal += !iw && !tw;
  c.initial_raises_pn += ipn > pn; c.initial_raises_dn += idn > dn;
  c.tt_greater_pn += pn > ipn; c.tt_greater_dn += dn > idn;
  if (initial_defense_branch) {
    ++c.or_defense_branches;
    c.defense_initial_pn_greater += ipn > pn;
    c.defense_initial_pn_equal += ipn == pn;
    c.defense_tt_pn_greater += pn > ipn;
    // Counterfactual in this TT context; retain the worker's perturbation.
    const auto alternate = defense_base == 2 ? ipn - 2 : ipn + 2;
    c.defense_coefficient_survives += (ipn > pn ? ipn : pn) != (alternate > pn ? alternate : pn);
  }
}
#endif
inline void Bind(std::uint32_t id, Phase phase = Phase::kSearch) {
  thread_bank = &banks.at(id); current = &thread_bank->phases[static_cast<std::size_t>(phase)];
}
inline void SetPhase(Phase phase) { current = &thread_bank->phases[static_cast<std::size_t>(phase)]; }
inline void Reset(std::uint32_t threads) {
  current = nullptr; thread_bank = nullptr;
  banks.assign(threads, ThreadCounters{}); Bind(0, Phase::kSetup);
}
inline bool Slot(std::uint64_t& slots, bool condition) { ++slots; return condition; }

// Query-local accumulation: only one TLS merge at return, including early returns.
struct LookupObservation {
  Counters* target;
  bool strict_requested, exact{}, superior{}, inferior{}, accepted{}, effective{}, boundary{};
  bool exact_effective{}, superior_effective{}, inferior_effective{};
  bool proven{}, disproven{}, repetition{}, initial{}, strict_executed{};
  Relation last_relation{};
  bool last_effective{};
  std::uint64_t exact_entries{}, superior_entries{}, inferior_entries{}, boundary_entries{}, metadata_entries{};
  std::uint64_t primary_slots{}, strict_slots{}, strict_len_updates{};
  explicit LookupObservation(bool strict) : target(current), strict_requested(strict) {}
  void MarkEffective(Relation relation) {
    effective = true;
    if (relation == Relation::kExact) exact_effective = true;
    else if (relation == Relation::kSuperior) superior_effective = true;
    else inferior_effective = true;
  }
  void Record(Relation relation, bool used, bool changed, bool metadata_changed) {
    last_relation = relation; last_effective = changed || metadata_changed;
    if (!used) return;
    accepted = true;
    if (relation == Relation::kExact) { exact = true; ++exact_entries; }
    else if (relation == Relation::kSuperior) { superior = true; ++superior_entries; }
    else { inferior = true; ++inferior_entries; }
    if (changed) { boundary = true; ++boundary_entries; }
    if (metadata_changed) ++metadata_entries;
    if (last_effective) MarkEffective(relation);
  }
  void AmountChanged() { ++metadata_entries; MarkEffective(last_relation); }
  ~LookupObservation() {
    if (!target) return;
    auto& c = *target; ++c.lookup_calls; c.strict_requested += strict_requested;
    c.exact_entries += exact_entries; c.superior_entries += superior_entries; c.inferior_entries += inferior_entries;
    c.exact_queries += exact; c.superior_queries += superior; c.inferior_queries += inferior;
    c.accepted_queries += accepted; c.boundary_entries += boundary_entries; c.boundary_queries += boundary;
    c.metadata_entries += metadata_entries;
    c.exact_effective_queries += exact_effective; c.superior_effective_queries += superior_effective;
    c.inferior_effective_queries += inferior_effective;
    c.effective_queries += effective || proven || disproven || repetition;
    c.no_effective_queries += !(effective || proven || disproven || repetition);
    c.proven_queries += proven; c.disproven_queries += disproven; c.repetition_queries += repetition;
    c.initial_eval_calls += initial;
#ifdef ENABLE_TT_STATS_DETAIL
    c.primary.Add(primary_slots);
    if (strict_executed) c.strict_extra.Add(strict_slots);
    c.strict_len_updates += strict_len_updates;
#endif
  }
};
#ifdef ENABLE_TT_STATS_DETAIL
struct WriteObservation {
  Counters* target = current;
  std::uint64_t slots{};
  ~WriteObservation() { if (target) target->write.Add(slots); }
};
#endif
inline std::string Serialize(const Counters& c, const char* phase, int thread) {
  std::ostringstream out;
  out << "{\"schema\":1,\"thread\":" << thread << ",\"phase\":\"" << phase << "\",\"detail\":";
#ifdef ENABLE_TT_STATS_DETAIL
  out << "true";
#else
  out << "false";
#endif
#define TT_PRINT(name) out << ",\"" #name "\":" << c.name;
  KOMORI_TT_COUNTERS(TT_PRINT)
#undef TT_PRINT
  const auto probe = [&out](const char* name, const Probe& p) {
    out << ",\"" << name << "\":{\"calls\":" << p.calls << ",\"slots\":" << p.slots
        << ",\"max\":" << p.maximum << ",\"hist\":[";
    for (std::size_t i = 0; i < p.histogram.size(); ++i) { if (i) out << ','; out << p.histogram[i]; }
    out << "]}";
  };
  probe("primary", c.primary); probe("strict_extra", c.strict_extra); probe("write", c.write);
  out << '}'; return out.str();
}
inline std::vector<std::string> FinalLines() {
  static constexpr const char* names[] = {"setup", "search", "pv"};
  std::vector<std::string> lines;
  Counters total;
  for (std::size_t id = 0; id < banks.size(); ++id) {
    for (std::size_t phase = 0; phase < 3; ++phase) {
      lines.push_back(Serialize(banks[id].phases[phase], names[phase], static_cast<int>(id)));
      total.Merge(banks[id].phases[phase]);
    }
  }
  lines.push_back(Serialize(total, "all", -1));
  current = nullptr;
  return lines;
}
}  // namespace komori::tt_stats
#endif

#ifdef ENABLE_TT_STATS_DETAIL
#define TT_DETAIL_INC(name) do { if (::komori::tt_stats::current) ++::komori::tt_stats::current->name; } while (false)
#define TT_SLOT(counter, expression) ::komori::tt_stats::Slot(counter, (expression))
#else
#define TT_DETAIL_INC(name) do {} while (false)
#define TT_SLOT(counter, expression) (expression)
#endif
#if defined(ENABLE_TT_STATS_DETAIL) || defined(ENABLE_ROOT_ITERATION_STATS)
#define TT_EXPANSION_INC(name) do { if (::komori::tt_stats::current) ++::komori::tt_stats::current->name; } while (false)
#else
#define TT_EXPANSION_INC(name) do {} while (false)
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
#define TT_ITER_INC(name) do { if (::komori::tt_stats::current) ++::komori::tt_stats::current->name; } while (false)
#else
#define TT_ITER_INC(name) do {} while (false)
#endif
#endif

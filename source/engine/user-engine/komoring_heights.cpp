#include "komoring_heights.hpp"

#include "../../usi.h"
#include "local_expansion.hpp"
#include "mate_len.hpp"
#include "node.hpp"
#include "score.hpp"
#include "search_result.hpp"
#include "typedefs.hpp"
#include "tt_stats.hpp"
#include "root_threshold.hpp"
#include <memory>
#include <unordered_set>
static_assert(sizeof(komori::tt::Entry) == 32);
static_assert(alignof(komori::tt::Entry) == 32);

namespace komori {
namespace {
/// GC で削除するエントリの割合
constexpr double kGcRemovalRatio = 0.5;
static_assert(kGcRemovalRatio > 0 && kGcRemovalRatio < 1.0, "kGcRemovalRatio must be greater than 0 and less than 1");

// 反復深化のしきい値を適当に伸ばす
std::pair<PnDn, PnDn> NextPnDnThresholds(PnDn pn, PnDn dn, PnDn curr_thpn, PnDn curr_thdn) {
  const double factor = 1.7 + 0.3 * tl_thread_id;
  return {threshold_math::Expand(pn, curr_thpn, factor, kInfinitePnDn),
          threshold_math::Expand(dn, curr_thdn, factor, kInfinitePnDn)};
}

Move SelectNextBestmove(const LocalExpansion& local_expansion, const std::vector<Move>& searched_moves) {
  for (const auto& [move, result] : local_expansion.GetAllResults()) {
    if (std::find(searched_moves.begin(), searched_moves.end(), move) == searched_moves.end()) {
      return move;
    }
  }

  return MOVE_NONE;
}
}  // namespace

void KomoringHeights::Init(const EngineOption& option, std::uint32_t num_threads) {
  option_ = option;
  barrier_.Initialize(option_.threads);
  tt_.Resize(option_.hash_mb);
  expansion_list_.resize(num_threads);
  expansion_list_.shrink_to_fit();

#if defined(USE_TT_SAVE_AND_LOAD)
  const auto& tt_read_path = option_.tt_read_path;
  if (!tt_read_path.empty()) {
    std::ifstream ifs(tt_read_path, std::ios::binary);
    if (ifs) {
      sync_cout << "info string load_path: " << tt_read_path << sync_endl;
      tt_.Load(ifs);
    }
  }
#endif  // defined(USE_TT_SAVE_AND_LOAD)
}

void KomoringHeights::Clear() {
  tt_.Clear();
}

void KomoringHeights::Stop() {
  monitor_.Stop();
}

void KomoringHeights::NewSearch(const Position& n, bool is_root_or_node) {
#ifdef ENABLE_TT_STATS
  tt_stats::Reset(option_.threads);
#endif
  auto& nn = const_cast<Position&>(n);
  const Node node{nn, is_root_or_node};

  tt_.NewSearch();
  monitor_.NewSearch(tt_.Capacity(), option_.pv_interval, option_.nodes_limit);
  best_moves_.clear();
  score_ = Score{};
  pv_list_.NewSearch(node);
  in_pv_search_ = false;
  pv_moves_.Clear();
  current_pv_complete_ = false;
  best_pv_complete_ = false;
  pv_path_keys_.clear();
  pv_status_ = "not_constructed";
  pv_validation_nodes_ = 0;
  pv_nodes_ = 0;
  root_decision_nodes_ = 0;
  root_decided_ = false;
#ifdef ENABLE_ROOT_ITERATION_STATS
  root_iteration_trace_.Reset();
#endif
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
  adaptive_root_log_.Reset();
#endif
#ifdef ENABLE_ROOT_THRESHOLD_STATS
  root_threshold_stats_ = {};
#endif

  if (tt_.Hashfull() >= kExecuteGcHashfullThreshold) {
    tt_.Clear();
  }

  moves_from_root_.clear();
  mate_len_ = MateLen::DepthMax();
  should_break_main_loop_.store(false, std::memory_order_relaxed);
  multi_pv_ = option_.multi_pv;

  search_promises_.resize(option_.threads);
  search_futures_.resize(option_.threads);
}

NodeState KomoringHeights::SearchMainThread(const Position& n, bool is_root_or_node) {
  auto& nn = const_cast<Position&>(n);
  Node node{nn, is_root_or_node};

  NodeState node_state = NodeState::kUnknown;
  SearchResult best_child_result;

  std::vector<Move> searched_moves{};
  std::size_t num_found_win_moves = 0;
  const std::size_t num_legal_moves = MovePicker{node}.size();
  const std::size_t max_num_win_moves = std::min<std::size_t>(num_legal_moves, option_.multi_pv);
  while (num_found_win_moves < max_num_win_moves && searched_moves.size() < num_legal_moves && !monitor_.ShouldStop()) {
    const std::uint32_t multi_pv = num_found_win_moves + 1;
    in_pv_search_ = false;
#ifdef ENABLE_TT_STATS
    tt_stats::SetPhase(tt_stats::Phase::kSearch);
#endif
    expansion_list_[tl_thread_id].Emplace(tt_, node, MateLen::DepthMax(), true, multi_pv);
    LocalExpansion& local_expansion = expansion_list_[tl_thread_id].back();

    SearchResult result;
    while (!monitor_.ShouldStop()) {
      DispatchSearch(node, MateLen::DepthMax(), multi_pv);
      result = local_expansion.CurrentResult(node);

      if (result.IsFinal()) {
        break;
      }
      local_expansion.Relookup(tt_, node, MateLen::DepthMax(), false, multi_pv, false);
    }

    // A final child alone is not a proof/disproof of the root.
    // Preserve a root conclusion while MultiPV searches additional moves.
    const auto root_result = local_expansion.CurrentResult(node);
    if (!root_decided_ && root_result.IsFinal()) {
      node_state = root_result.GetNodeState();
      root_decided_ = true;
      root_decision_nodes_ = monitor_.MoveCount();
    }
    // MultiPV can keep the remaining-moves aggregate unknown after one witness
    // already proves an OR root (or disproves an AND root).
    if (!root_decided_) {
      for (const auto [move, child] : local_expansion.GetAllResults()) {
        if ((node.IsOrNode() && child.Pn() == 0) || (!node.IsOrNode() && child.Dn() == 0)) {
          node_state = child.GetNodeState();
          root_decided_ = true;
          root_decision_nodes_ = monitor_.MoveCount();
          break;
        }
      }
    }
    const auto pv_start_nodes = monitor_.MoveCount();
    in_pv_search_ = true;
#ifdef ENABLE_TT_STATS
    tt_stats::SetPhase(tt_stats::Phase::kPv);
#endif
    for (auto [move, child_result] : local_expansion.GetAllResults()) {
      if (!child_result.IsFinal() ||
          std::find(searched_moves.begin(), searched_moves.end(), move) != searched_moves.end()) {
        continue;
      }
      searched_moves.push_back(move);

      if (searched_moves.size() == 1) {
        score_ = score_maker_.Make(child_result, node.IsRootOrNode());
        score_.AddOneIfFinal();
      }
      sync_cout << CurrentInfo() << move << " " << child_result << sync_endl;

      node.DoMoveNoRepetition(move);
      pv_moves_.AddMove(move, 0);
      current_pv_complete_ = false;
      pv_path_keys_.clear();
      if (child_result.Pn() == 0) {
        ++num_found_win_moves;
#ifdef ENABLE_PV_TEST_HOOKS
        sync_cout << "info string pv_test_phase_begin" << sync_endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
#endif
        child_result = ConstructProvenPv(node, child_result.Len());
      } else if (child_result.Dn() == 0) {
        if (!node.IsOrNode()) {
          const Move evasion = GetEvasion(node);
          pv_moves_.AddMove(evasion, 1);
        }
      }

      if (child_result.IsFinal()) {
        local_expansion.UpdateFinal(child_result, move);
      }
      pv_list_.Update(move, child_result, 0, pv_moves_.Moves());
      if (searched_moves.size() == 1 || SearchResultComparer{node.IsRootOrNode()}(child_result, best_child_result) ==
                                            SearchResultComparer::Ordering::kLess) {
        best_child_result = child_result;

        best_moves_ = pv_moves_.Moves();
        best_pv_complete_ = current_pv_complete_ && child_result.Pn() == 0;
        score_ = score_maker_.Make(child_result, node.IsRootOrNode());
        score_.AddOneIfFinal();
      }
      Print(node);

      node.UndoMoveNoRepetition();
    }
    pv_nodes_ += monitor_.MoveCount() - pv_start_nodes;

    expansion_list_[tl_thread_id].Pop();
  }

  // 待機している sub thread を解放する
  should_break_main_loop_ = true;
  barrier_.Await();

  if (num_legal_moves == 0) {
    node_state = node.IsOrNode() ? NodeState::kDisproven : NodeState::kProven;
    root_decided_ = true;
    root_decision_nodes_ = monitor_.MoveCount();
    best_pv_complete_ = node_state == NodeState::kProven;
  }

  return node_state;
}

void KomoringHeights::PrintFinalStatistics(NodeState root_state) const {
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
  for (const auto& line : adaptive_root_log_.Lines())
    sync_cout << "info string adaptive_root " << line << sync_endl;
  sync_cout << "info string adaptive_root_summary {\"rows\":" << adaptive_root_log_.Size()
            << ",\"dropped\":" << adaptive_root_log_.Dropped() << "}" << sync_endl;
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
  for (const auto& line : root_iteration_trace_.Lines(root_threshold::kTenths))
    sync_cout << "info string root_iteration " << line << sync_endl;
  sync_cout << "info string root_iteration_summary {\"rows\":" << root_iteration_trace_.Size()
            << ",\"dropped\":" << root_iteration_trace_.Dropped() << "}" << sync_endl;
#endif
#ifdef ENABLE_ROOT_THRESHOLD_STATS
  const auto& r = root_threshold_stats_;
  sync_cout << "info string root_threshold_stats {\"factor_tenths\":" << root_threshold::kTenths
            << ",\"calls\":" << r.calls << ",\"iterations\":" << r.iterations
            << ",\"extensions\":" << r.extensions << ",\"pn\":" << r.pn << ",\"dn\":" << r.dn
            << ",\"max_pn\":" << r.max_pn << ",\"max_dn\":" << r.max_dn
            << ",\"saturated_extensions\":" << r.saturated
            << ",\"legacy_conversion_risks\":" << r.legacy_risks << "}" << sync_endl;
#endif
  const auto stats = monitor_.FinalInfo();
  const char* state = root_state == NodeState::kProven ? "proven" :
      root_state == NodeState::kDisproven ? "disproven" :
      root_state == NodeState::kRepetition ? "repetition" : "unknown";
  std::string reasons;
  const auto append_reason = [&reasons](const char* reason) {
    if (!reasons.empty()) reasons += ",";
    reasons += reason;
  };
  if (stats.stop_reasons & SearchMonitor::kTimeLimit) append_reason("time_limit");
  if (stats.stop_reasons & SearchMonitor::kNodeLimit) append_reason("node_limit");
  if (stats.stop_reasons & SearchMonitor::kExternalStop) append_reason("external_stop");
  if (reasons.empty()) reasons = root_decided_ ? "completed" : "unknown";
  if (root_state == NodeState::kProven && !best_pv_complete_) {
    if (reasons == "completed") reasons.clear();
    append_reason("pv_incomplete");
  }
  sync_cout << "info time " << stats.time_us / 1000 << " nodes " << stats.nodes << " nps " << stats.nps
            << " string final_stats time_us=" << stats.time_us << " root_state=" << state
            << " stop_reason=" << reasons << " stop_flags=" << stats.stop_reasons
            << " search_nodes=" << stats.nodes - pv_nodes_ << " pv_nodes=" << pv_nodes_
            << " root_decision_nodes=" << root_decision_nodes_ << " root_decided=" << root_decided_
            << " pv_complete=" << best_pv_complete_ << " pv_moves=" << best_moves_.size()
            << " pv_status=" << pv_status_ << " pv_validation_nodes=" << pv_validation_nodes_ << sync_endl;
}

void KomoringHeights::ValidateBestPv(const Position& root, bool root_or_node) {
  const auto before = monitor_.MoveCount();
  Defer account([this, before]() {
    pv_validation_nodes_ = monitor_.MoveCount() - before;
    pv_nodes_ += pv_validation_nodes_;
  });
  best_pv_complete_ = false;
  pv_status_ = "pv_incomplete";
  if (best_moves_.size() >= static_cast<std::size_t>(kDepthMax)) {
    pv_status_ = "depth_limit";
    return;
  }
  // Node's state stack is large, so keep the replay object off the call stack.
  auto replay = std::make_unique<Position>();
  StateInfo state{};
  replay->set(root.sfen(), &state, root.this_thread());
  auto node = std::make_unique<Node>(*replay, root_or_node);
  std::unordered_set<std::string> seen;
  const auto position_id = [&]() {
    const auto sfen = replay->sfen();
    return sfen.substr(0, sfen.rfind(' ')); // Exclude the move number.
  };
  seen.insert(position_id());
  for (Move move : best_moves_) {
    const MovePicker legal{*node};
    if (std::none_of(legal.begin(), legal.end(), [move](const auto& m) { return m.move == move; })) {
      pv_status_ = "illegal_move_or_noncheck";
      return;
    }
    node->DoMoveNoRepetition(move);
    if (!seen.insert(position_id()).second) {
      pv_status_ = "repeated_position";
      return;
    }
  }
  if (node->IsOrNode() || !replay->in_check() || !MovePicker{*node}.empty()) {
    pv_status_ = "terminal_has_evasion";
    return;
  }
  best_pv_complete_ = true;
  pv_status_ = best_moves_.empty() ? "zero_ply_mate" : "complete";
}

NodeState KomoringHeights::SearchSubThread(const Position& n, bool is_root_or_node) {
  auto& nn = const_cast<Position&>(n);
  Node node{nn, is_root_or_node};

  barrier_.Await();
  while (!should_break_main_loop_) {
    // 探索本体
#ifdef ENABLE_TT_STATS
    tt_stats::SetPhase(dispatch_stats_phase_);
#endif
    RollForward(node, moves_from_root_);
    const SearchResult reuslt = SearchEntry(node, mate_len_, multi_pv_);

    monitor_.Stop();
    RollBack(node, moves_from_root_);

    search_promises_[tl_thread_id].set_value(reuslt);
    barrier_.Await();
  }

  return NodeState::kUnknown;
}

MateLen KomoringHeights::DispatchSearch(Node& n, MateLen len, std::uint32_t multi_pv) {
#ifdef ENABLE_TT_STATS
  dispatch_stats_phase_ = in_pv_search_ ? tt_stats::Phase::kPv : tt_stats::Phase::kSearch;
#endif
  const auto& moves = n.MovesFromStart();
  moves_from_root_ = {moves.begin(), moves.end()};
  mate_len_ = len;
  multi_pv_ = multi_pv;

  for (int i = 0; i < option_.threads; ++i) {
    search_promises_[i] = std::promise<SearchResult>{};
    search_futures_[i] = search_promises_[i].get_future();
  }
  barrier_.Await();
  search_promises_[tl_thread_id].set_value(SearchEntryNoEmplace(n, len));
  monitor_.Stop();

  MateLen result_len = MateLen::DepthMax();
  for (int i = 0; i < option_.threads; ++i) {
    const SearchResult thread_result = search_futures_[i].get();
    if (thread_result.Pn() == 0) {
      result_len = std::min(result_len, thread_result.Len());
    }
  }
  monitor_.ResetStop();

  return result_len;
}

SearchResult KomoringHeights::SearchEntry(Node& n, MateLen len, std::uint32_t multi_pv) {
  expansion_list_[tl_thread_id].Emplace(tt_, n, len, true, multi_pv, len != MateLen::DepthMax());
  Defer release_expansion([this]() { expansion_list_[tl_thread_id].Pop(); });
  return SearchEntryNoEmplace(n, len);
}

SearchResult KomoringHeights::SearchEntryNoEmplace(Node& n, MateLen max_len) {
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
  // Evaluate thread count first: workers must not read the main-owned PV phase.
  const bool is_adaptive_root = option_.threads == 1 && tl_thread_id == 0 && n.GetDepth() == 0 && !in_pv_search_;
  // Invocation-local history cannot survive a new root call or NewSearch.
  adaptive_root::Controller adaptive_controller;
  adaptive_controller.Reset();
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
  const auto adaptive_call = is_adaptive_root ? adaptive_root_log_.NewCall() : 0;
  std::uint64_t adaptive_iteration=0;
#endif
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
  const bool iteration_root = option_.threads == 1 && tl_thread_id == 0 && n.GetDepth() == 0 && !in_pv_search_;
  if (iteration_root) root_iteration_trace_.NewCall();
#endif
#if KOMORI_ROOT_THRESHOLD_TENTHS != 17
  // Short circuit on thread count before reading the main-owned PV phase.
  const bool custom_root = option_.threads == 1 && tl_thread_id == 0 && n.GetDepth() == 0 && !in_pv_search_;
#endif
#ifdef ENABLE_ROOT_THRESHOLD_STATS
  const bool observed_root = tl_thread_id == 0 && n.GetDepth() == 0 && !in_pv_search_;
  if (observed_root) ++root_threshold_stats_.calls;
#endif
    TT_DETAIL_INC(entry_calls);
#ifdef ENABLE_TT_STATS_DETAIL
  const bool stats_root = n.GetDepth() == 0;
  if (stats_root) TT_DETAIL_INC(root_entry_calls);
#endif
  PnDn thpn = (tl_thread_id + 1) * kPnDnUnit;
  PnDn thdn = (tl_thread_id + 1) * kPnDnUnit;
  SearchResult result;
  do {
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
    const auto adaptive_nodes_begin = is_adaptive_root ? monitor_.MoveCount() : 0;
    adaptive_root::ReturnInfo adaptive_return;
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
    adaptive_root::LogRow adaptive_row;
    if (is_adaptive_root) {
      adaptive_row.call=adaptive_call; adaptive_row.iteration=++adaptive_iteration;
      adaptive_row.nodes_begin=adaptive_nodes_begin;
      std::tie(adaptive_row.pn_begin,adaptive_row.dn_begin)=expansion_list_[tl_thread_id].back().ObservedPnDn();
    }
#endif
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
    if (iteration_root) {
      const auto [pn, dn] = expansion_list_[tl_thread_id].back().ObservedPnDn();
      const auto s = monitor_.FinalInfo();
      root_iteration_trace_.Begin(s.nodes, s.time_us, pn, dn, thpn, thdn);
    }
#endif
    std::uint32_t inc_flag = 0;
    TT_DETAIL_INC(entry_iterations);
#ifdef ENABLE_TT_STATS_DETAIL
    if (stats_root) TT_DETAIL_INC(root_iterations);
#endif
    result = SearchImpl(n, thpn, thdn, max_len, inc_flag
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
                        , is_adaptive_root ? &adaptive_return : nullptr
#endif
                        );
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
    adaptive_root::Decision adaptive_decision;
    if (is_adaptive_root) {
      const auto nodes_end=monitor_.MoveCount();
      const auto flags=monitor_.StopFlags();
      const adaptive_root::Window window{nodes_end-adaptive_nodes_begin,result.Pn(),result.Dn(),thpn,thdn,
          adaptive_return.thpn,adaptive_return.thdn,!result.IsFinal(),
          adaptive_return.returned && flags==0 && !monitor_.HasStopRequest(),
          adaptive_return.tca_changed,adaptive_return.tca_break};
      adaptive_decision=adaptive_controller.Finish(window);
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
      adaptive_row.nodes_end=nodes_end; adaptive_row.window=window; adaptive_row.decision=adaptive_decision;
      adaptive_row.stop_flags=flags; adaptive_row.extended=!result.IsFinal();
      adaptive_row.root_state=result.Pn()==0 ? "proven" : result.IsFinal() ?
          result.GetFinalData().IsRepetition() ? "repetition" : "disproven" : "unknown";
#endif
    }
#endif
#ifdef ENABLE_ROOT_THRESHOLD_STATS
    if (observed_root) {
      auto& r = root_threshold_stats_;
      ++r.iterations; r.pn = result.Pn(); r.dn = result.Dn();
      r.max_pn = std::max(r.max_pn, result.Pn()); r.max_dn = std::max(r.max_dn, result.Dn());
    }
#endif
    if (result.IsFinal()) {
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
      if (is_adaptive_root) {
        adaptive_row.next_thpn=thpn; adaptive_row.next_thdn=thdn;
        adaptive_root_log_.Add(adaptive_row);
      }
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
      if (iteration_root) {
        const auto s = monitor_.FinalInfo();
        const char* state = result.Pn() == 0 ? "proven" :
            result.GetFinalData().IsRepetition() ? "repetition" : "disproven";
        root_iteration_trace_.Finish(s.nodes, s.time_us, result.Pn(), result.Dn(), thpn, thdn, s.stop_reasons, state, false);
      }
#endif
      break;
    }

    if (tl_thread_id == 0 && !score_.IsFinal()) {
      score_ = score_maker_.Make(result, n.IsRootOrNode());
    }

    TT_DETAIL_INC(threshold_extensions);
#ifdef ENABLE_TT_STATS_DETAIL
    if (stats_root) TT_DETAIL_INC(root_extensions);
#endif
#ifdef ENABLE_ROOT_THRESHOLD_STATS
    if (tl_thread_id == 0) {
      bool legacy = true;
#if KOMORI_ROOT_THRESHOLD_TENTHS != 17
      legacy = !custom_root;
#endif
      const double scale = 1.7 + 0.3 * tl_thread_id;
      if (legacy && (static_cast<double>(result.Pn()) * scale >= std::numeric_limits<PnDn>::max() ||
                     static_cast<double>(result.Dn()) * scale >= std::numeric_limits<PnDn>::max()))
        ++root_threshold_stats_.legacy_risks;
    }
    if (observed_root) ++root_threshold_stats_.extensions;
#endif
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
    if (is_adaptive_root) {
      const double factor=adaptive_decision.tenths/10.0;
      thpn=root_threshold::Expand(result.Pn(),thpn,factor);
      thdn=root_threshold::Expand(result.Dn(),thdn,factor);
    } else
#endif
#if KOMORI_ROOT_THRESHOLD_TENTHS != 17
    if (custom_root) {
      constexpr double factor = root_threshold::kTenths / 10.0;
      thpn = root_threshold::Expand(result.Pn(), thpn, factor);
      thdn = root_threshold::Expand(result.Dn(), thdn, factor);
    } else
#endif
    {
      std::tie(thpn, thdn) = NextPnDnThresholds(result.Pn(), result.Dn(), thpn, thdn);
    }
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
    if (is_adaptive_root) {
      adaptive_row.next_thpn=thpn; adaptive_row.next_thdn=thdn;
      adaptive_root_log_.Add(adaptive_row);
    }
#endif
#ifdef ENABLE_ROOT_THRESHOLD_STATS
    if (observed_root && (thpn == kInfinitePnDn || thdn == kInfinitePnDn)) ++root_threshold_stats_.saturated;
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
    if (iteration_root) {
      const auto s = monitor_.FinalInfo();
      root_iteration_trace_.Finish(s.nodes, s.time_us, result.Pn(), result.Dn(), thpn, thdn, s.stop_reasons, "unknown", true);
    }
#endif
  } while (!monitor_.ShouldStop() && thpn <= kInfinitePnDn && thdn <= kInfinitePnDn);
  return result;
}

SearchResult KomoringHeights::ConstructProvenPv(Node& n, MateLen max_len) {
  const auto incomplete = [&]() { return SearchResult::MakeUnknown(1, 1, max_len, 1); };
  if (n.GetDepth() >= kDepthMax - 1 ||
      std::find(pv_path_keys_.begin(), pv_path_keys_.end(), n.GetKey()) != pv_path_keys_.end()) {
    return incomplete();
  }
  pv_path_keys_.push_back(n.GetKey());
  Defer pop_path([this]() { pv_path_keys_.pop_back(); });
  // 証明駒を使わずにできるだけ短い詰みを LookUp してほしいので、strict_lookup=true にしている
  const std::uint32_t multi_pv = 1;
  expansion_list_[tl_thread_id].Emplace(tt_, n, max_len, false, multi_pv, true);
  LocalExpansion& local_expansion = expansion_list_[tl_thread_id].back();
  // early exit するときに忘れずに local_expansion を開放するための RAII
  Defer release_expansion([this]() { expansion_list_[tl_thread_id].Pop(); });

  SearchResult result = local_expansion.CurrentResult(n);
  std::uint32_t loop_count = 0;
  bool materialize_drops = false;
  for (;;) {
    while (!result.IsFinal() && !monitor_.ShouldStopPv()) {
      // mate_len 以下の詰みがあるはずなので頑張って探す
      // 前の週で別スレッドが詰みを見つけているかもしれないので、改めて展開しなおす
      local_expansion.Relookup(tt_, n, max_len, false, multi_pv, !materialize_drops);

      // sub thread たちにも `n` 以下 `mate_len_` 手詰めを見つけるのを手伝ってもらう
      const MateLen max_mate_len = DispatchSearch(n, max_len, multi_pv);

      // sub thread の結果を result にコピーすることもできるが、メインスレッドの local expansion の状態が
      // 狂ってしまうので、あえて何もしない
      result = local_expansion.CurrentResult(n);

      if (++loop_count % 30 == 0) {
        // 無駄合は確率で消える可能性があるので、max_len で詰むはずでも詰みを見つけれられないことがある。
        // そんなときは、詰み手数を伸ばして親局面から探索をやり直す

        if (!materialize_drops) max_len = option_.threads == 1 ? max_len + 2 : max_mate_len;
      }
    }

    // A budget stop can leave a reconstruction node unresolved. Do not fabricate a truncated mate PV.
    if (!result.IsFinal()) return result;

    if (result.Dn() == 0) {
      // Reconstruction failure must not revoke the independently proven root.
      return incomplete();
    }

    if (result.Len() <= MateLen::Zero()) {
      // Proof length zero may mean that all legal drops were classified as
      // redundant, rather than that the physical board has no legal evasion.
      if (!n.IsOrNode() && n.Pos().in_check() && MovePicker{n}.empty()) {
        current_pv_complete_ = true;
        return SearchResult::MakeFinal<true>(result.GetFinalData().hand, MateLen::Zero(), result.Amount());
      }
      if (result.Len() < MateLen::Zero() || n.IsOrNode() || materialize_drops || monitor_.ShouldStopPv()) {
        return incomplete();
      }
  #ifdef ENABLE_PV_TEST_LEGACY_TERMINAL
      // Fault injection: exercise the output validator against the historical
      // false-completion flag. Never enabled in a production build.
      current_pv_complete_ = true;
      return result;
  #elif defined(ENABLE_PV_TEST_DISABLE_REPAIR)
      return incomplete();
  #else
      // Only this reconstruction frame uses ordinary (non-strict) lookup so
      // omitted evasions become candidates again. Search/proof rules are intact.
      materialize_drops = true;
      max_len = MateLen::DepthMax();
      local_expansion.Relookup(tt_, n, max_len, false, multi_pv, false);
      result = local_expansion.CurrentResult(n);
      continue;
  #endif
    }
    break;
  }
  if (n.IsOrNode() && n.GetDepth() > 0) {
    if (const auto [best_move, proof_hand] = CheckMate1Ply(n.Pos()); proof_hand != kNullHand) {
      pv_moves_.AddMove(best_move, n.GetDepth());
      current_pv_complete_ = true;
      return SearchResult::MakeFinal<true>(proof_hand, MateLen{1}, 1);
    }
  }

  SearchResult child_result;
  do {
    if (monitor_.ShouldStopPv()) return incomplete();
    const Move best_move = local_expansion.BestMove();
    pv_moves_.AddMove(best_move, n.GetDepth());

    // DoMove() をするとループに遭遇したときに回避できないので、千日手判定なし版を使う
    n.DoMoveNoRepetition(best_move);
    child_result = ConstructProvenPv(n, result.Len() - 1);
    n.UndoMoveNoRepetition();

    if (!child_result.IsFinal()) return child_result;

    local_expansion.UpdateBestChild(child_result);
    result = local_expansion.CurrentResult(n);

    // 子局面で見つけた手数（mate_path の depth+1 以降に書かれた手数）が現局面の詰み手数と一致しているか確認する
    // もし差異があったら、現局面の詰み手数が間違っていた可能性があるのでもう一度探索する
  } while ((result.Len() != child_result.Len() + 1) && !monitor_.ShouldStopPv());

  return result;
}

Move KomoringHeights::GetEvasion(Node& n) {
  expansion_list_[tl_thread_id].Emplace(tt_, n, MateLen::DepthMax(), false, 1, true);
  LocalExpansion& local_expansion = expansion_list_[tl_thread_id].back();
  while (!monitor_.ShouldStop()) {
    DispatchSearch(n, MateLen::DepthMax(), 1);
    if (local_expansion.CurrentResult(n).IsFinal()) {
      break;
    }

    local_expansion.Relookup(tt_, n, MateLen::DepthMax(), false, 1, true);
  }

  const Move evasion = local_expansion.BestMove();
  expansion_list_[tl_thread_id].Pop();
  return evasion;
}

SearchResult KomoringHeights::SearchImpl(Node& n, PnDn thpn, PnDn thdn, MateLen len, std::uint32_t& inc_flag
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
                                       , adaptive_root::ReturnInfo* root_return
#endif
                                       ) {
  TT_ITER_INC(impl_calls);
#ifdef ENABLE_ROOT_ITERATION_STATS
  bool observed_tca_break = false;
#endif
  const PnDn orig_thpn = thpn;
  const PnDn orig_thdn = thdn;
  const std::uint32_t orig_inc_flag = inc_flag;

  auto& local_expansion = expansion_list_[tl_thread_id].back();
  monitor_.Visit(n.GetDepth());
  if (tl_thread_id == 0 && monitor_.ShouldPrint()) {
    Print(n);
  }

  if (n.GetDepth() >= kDepthMax - 1) {
    TT_ITER_INC(impl_depth_returns);
    return SearchResult::MakeRepetition(n.OrHand(), len, 1, 0);
  }

  // 必要があれば TCA による探索延長をしたいので、このタイミングで現局面の pn/dn を取得する。
  auto curr_result = local_expansion.CurrentResult(n);
  // Threshold Controlling Algorithm(TCA).
  // 浅い結果を参照している場合、無限ループになる可能性があるので少しだけ探索を延長する
  if (local_expansion.DoesHaveOldChild()) {
    inc_flag++;
  }

  if (inc_flag > 0) {
    ExtendSearchThreshold(curr_result, thpn, thdn);
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
    if (root_return && (thpn!=orig_thpn || thdn!=orig_thdn)) root_return->tca_changed=true;
#endif
  }

  if (tl_gc_thread && monitor_.ShouldCheckHashfull()) {
    if (tt_.Hashfull() >= kExecuteGcHashfullThreshold) {
      tt_.CollectGarbage(kGcRemovalRatio);
    }
    monitor_.ResetNextHashfullCheck();
  }

  while (!monitor_.ShouldStop() && (curr_result.Pn() < thpn && curr_result.Dn() < thdn)) {
    // local_expansion.BestMove() にしたがい子局面を展開する
    // （curr_result.Pn() > 0 && curr_result.Dn() > 0 なので、BestMove が必ず存在する）
    const auto best_move = local_expansion.BestMove();
    const bool is_first_search = local_expansion.FrontIsFirstVisit();
    if (is_first_search) { TT_EXPANSION_INC(expansion_first_by_tt); } else { TT_EXPANSION_INC(expansion_revisit_by_tt); }
    const auto [child_thpn, child_thdn] = local_expansion.FrontPnDnThresholds(thpn, thdn);

    n.DoMove(best_move);

    // 子局面を展開する。展開した expansion は UndoMove() の直前に忘れずに開放しなければならない。
    expansion_list_[tl_thread_id].Emplace(tt_, n, len - 1, is_first_search, 1);
    auto& child_expansion = expansion_list_[tl_thread_id].back();

    SearchResult child_result;
    if (is_first_search) {
      child_result = child_expansion.CurrentResult(n);
      // 新規局面を展開したので、inc_flag を 1 つ減らしておく
      // オリジナルの TCA では inc_flag は bool 値なので単に false を代入していたが、KomoringHeights では
      // 非負整数へ拡張している。
      if (inc_flag > 0) {
        inc_flag--;
      }

      // 子局面を初展開する場合、child_result を計算した時点で threshold を超過する可能性がある
      // しかし、SearchImpl をコールしてしまうと TCA の探索延長によりすぐに返ってこない可能性がある
      // ゆえに、この時点で Exceed している場合は SearchImpl を呼ばないようにする。
      if (child_result.Pn() >= child_thpn || child_result.Dn() >= child_thdn) {
        TT_EXPANSION_INC(expansion_immediate_return);
        goto CHILD_SEARCH_END;
      }
    }
    child_result = SearchImpl(n, child_thpn, child_thdn, len - 1, inc_flag);

  CHILD_SEARCH_END:
    expansion_list_[tl_thread_id].Pop();
    n.UndoMove();

    local_expansion.UpdateBestChild(child_result);
    curr_result = local_expansion.CurrentResult(n);

    // TCA で延長したしきい値はいったん戻す
    thpn = orig_thpn;
    thdn = orig_thdn;
    if (inc_flag > 0) {
      // TCA 継続中ならしきい値を伸ばす
      ExtendSearchThreshold(curr_result, thpn, thdn);
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
      if (root_return && (thpn!=orig_thpn || thdn!=orig_thdn)) root_return->tca_changed=true;
#endif
    } else if (inc_flag == 0 && orig_inc_flag > 0) {
      // TCA の展開が終わったので、いったん親局面に戻る
#ifdef ENABLE_ROOT_ITERATION_STATS
      observed_tca_break = true;
#endif
      TT_ITER_INC(impl_tca_breaks);
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
      if (root_return) root_return->tca_break=true;
#endif
      break;
    }
  }

  /// `inc_flag` の値は探索前より小さくなっているはず
  inc_flag = std::min(inc_flag, orig_inc_flag);
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
  if (root_return) {
    root_return->thpn=thpn; root_return->thdn=thdn; root_return->returned=true;
  }
#endif
#ifdef ENABLE_ROOT_ITERATION_STATS
  if (curr_result.IsFinal()) { TT_ITER_INC(impl_final_returns); }
  else if (curr_result.Pn() >= thpn || curr_result.Dn() >= thdn) { TT_ITER_INC(impl_threshold_returns); }
  else { TT_ITER_INC(impl_other_returns); }
  if (n.GetDepth() == 0 && option_.threads == 1 && root_iteration_trace_.Active())
    root_iteration_trace_.RootReturn(thpn, thdn, observed_tca_break);
#endif
  return curr_result;
}

UsiInfo KomoringHeights::CurrentInfo() const {
  UsiInfo usi_output = monitor_.GetInfo();
  usi_output.Set(UsiInfoKey::kHashfull, tt_.Hashfull());
  usi_output.Set(UsiInfoKey::kScore, score_.ToString());

  return usi_output;
}

void KomoringHeights::Print(const Node& n) {
  if (option_.silent) {
    return;
  }

  UsiInfo info = CurrentInfo();
  if (!expansion_list_[0].empty()) {
    const LocalExpansion& root = expansion_list_[0].front();
    const SearchResult result = root.FrontResult();
    if (!in_pv_search_) {
      const auto& pv = n.MovesFromStart();
      std::vector<Move> pv_moves(pv.begin(), pv.end());
      pv_list_.Update(root.BestMove(), result, n.GetDepth(), std::move(pv_moves));
    }

    info.Set(UsiInfoKey::kCurrMove, USI::move(root.BestMove()));
    for (const auto& [i, mr] : WithIndex(root.GetAllResults())) {
      const auto& [move, result] = mr;
      if (i >= option_.multi_pv) {
        break;
      }
      const PvList::PvInfo pv_info = pv_list_.GetPvInfo(move);
      Score score = score_maker_.Make(pv_info.result, n.IsRootOrNode());
      score.AddOneIfFinal();
      info.PushPVBack(pv_info.depth, score.ToString(), ToString(pv_info.pv));
    }
  }

  sync_cout << info << sync_endl;
}
}  // namespace komori

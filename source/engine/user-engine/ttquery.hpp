/**
 * @file ttquery.hpp
 */
#ifndef KOMORI_TTQUERY_HPP_
#define KOMORI_TTQUERY_HPP_

#include <optional>
#include <shared_mutex>

#include "mate_len.hpp"
#include "regular_table.hpp"
#include "repetition_table.hpp"
#include "search_result.hpp"
#include "ttentry.hpp"
#include "typedefs.hpp"
#include "tt_stats.hpp"
#include "or_defense_config.hpp"
#ifdef ENABLE_TT_SINGLE_THREAD_LOCK_OPT
#include "tt_read_lock_policy.hpp"
#endif

namespace komori::tt {

/**
 * @brief 連続する複数エントリを束ねてまとめて読み書きするためのクラス。
 *
 * 詰将棋エンジンでは置換表を何回も何回も読み書きする。このクラスでは、置換表の読み書きに必要となる
 * 情報（ハッシュ値など）をキャッシュしておき、高速化することが目的である。
 *
 * ## 実装詳細
 *
 * 置換表から結果を読む際、以下の情報を利用して探索結果を復元する。
 *
 * 1. 同一局面（盤面も持ち駒も一致）
 * 2. 優等局面（盤面が一致していて持ち駒が現局面より多い）
 * 3. 劣等局面（盤面が一致していて持ち駒が現局面より少ない）
 */
class Query {
 public:
  /**
   * @brief Query の構築を行う。
   * @param rep_table   千日手テーブル
   * @param initial_entry_pointer 通常テーブルの探索開始位置への循環ポインタ
   * @param path_key    経路ハッシュ値
   * @param board_key   盤面ハッシュ値
   * @param hand        持ち駒
   * @param depth       探索深さ
   */
  constexpr Query(RepetitionTable& rep_table,
                  CircularEntryPointer initial_entry_pointer,
                  Key path_key,
                  Key board_key,
                  Hand hand,
                  Depth depth)
      : rep_table_{&rep_table},
        initial_entry_pointer_{initial_entry_pointer},
        path_key_{path_key},
        board_key_{board_key},
        hand_{hand},
        depth_{depth},
        cached_entry_{&*initial_entry_pointer} {};

  /**
   * @brief Default constructor(default)
   *
   * 配列で領域を確保したいためデフォルトコンストラクト可能にしておく。デフォルトコンストラクト状態では
   * メンバ関数の呼び出しは完全に禁止である。そのため、使用する前に必ず引数つきのコンストラクタで初期化を行うこと。
   */
  Query() = default;
  /// Copy constructor(delete). 使うことはないと思われるので封じておく。
  Query(const Query&) = delete;
  /// Move constructor(default)
  constexpr Query(Query&&) noexcept = delete;
  /// Copy assign operator(delete)
  Query& operator=(const Query&) = delete;
  /// Move assign operator(default)
  constexpr Query& operator=(Query&&) noexcept = default;
  /// Destructor
  ~Query() noexcept = default;

#if !defined(DISABLE_CHILD_TT_PREFETCH) && (defined(__GNUC__) || defined(__clang__))
  /// Prefetch the initial regular-TT entry without reading or changing it.
  void PrefetchInitialEntry() const noexcept {
    auto ptr = initial_entry_pointer_;
    __builtin_prefetch(ptr.data(), 0, 1);
  }
#endif

  // テンプレート関数のカバレッジは悲しいことになるので取らない
  // LCOV_EXCL_START NOLINTBEGIN

  /**
   * @brief 置換表から結果を集めてきて返す関数。
   * @tparam InitialEvalFunc 初期値関数の型
   * @param does_have_old_child  unproven old child の結果を使った場合 `true` が書かれる変数
   * @param len 探している詰み手数
   * @param eval_func 初期値を計算する関数。エントリが見つからなかったときのみ呼ばれる。
   * @param strict_lookup 厳密な探索を行うかどうか。false のときは、詰みを見つけたらすぐにそれを返す。
   * @return Look Up 結果
   *
   * 置換表を探索して現局面の探索情報を返す関数。
   *
   * 一般に、初期化関数の呼び出しは実行コストがかかる。そのため、初期化関数は `eval_func` に包んで渡す。
   * 必要な時のみ `eval_func` の呼び出しを行うことで、呼び出さないパスの高速化ができる。
   */
  template <typename InitialEvalFunc>
  SearchResult LookUp(bool& does_have_old_child,
                      MateLen len,
                      InitialEvalFunc&& eval_func,
                      bool strict_lookup = false) const {
#ifdef ENABLE_TT_SINGLE_THREAD_LOCK_OPT
    // Select once per query, never once per probed slot. The mode is fixed
    // before search; both primary and strict lookup use the same policy.
    if (read_lock_policy::required) {
      return LookUpImpl<true>(does_have_old_child, len, std::forward<InitialEvalFunc>(eval_func), strict_lookup);
    }
    return LookUpImpl<false>(does_have_old_child, len, std::forward<InitialEvalFunc>(eval_func), strict_lookup);
  }

 private:
  template <bool kReadLockRequired, typename InitialEvalFunc>
  SearchResult LookUpImpl(bool& does_have_old_child, MateLen len, InitialEvalFunc&& eval_func,
                          bool strict_lookup) const {
#endif
    #ifdef ENABLE_TT_STATS
    tt_stats::LookupObservation observation(strict_lookup);
    #endif
    PnDn pn = 1;
    PnDn dn = 1;
    SearchAmount amount = 1;

    bool found_exact = false;
#ifdef ENABLE_TT_LOOKUP_KEY_CACHE
    // This Query key is immutable throughout the primary and strict traversal.
    const Key lookup_board_key = board_key_;
#endif

    for (auto itr = initial_entry_pointer_; TT_SLOT(observation.primary_slots, !itr->IsNull()); ++itr) {
#ifdef ENABLE_TT_SINGLE_THREAD_LOCK_OPT
      read_lock_policy::Guard<kReadLockRequired, Entry> lock(*itr);
#else
      std::shared_lock lock(*itr);
#endif
      // 本来は lock 後にも !itr->Null() のチェックが必要だが、itr->Hand() == kNullHand のとき itr->LookUp() が必ず
      // 失敗するので、このタイミングでのチェックは省略できる。
      if (itr->IsFor(
#ifdef ENABLE_TT_LOOKUP_KEY_CACHE
                      lookup_board_key
#else
                      board_key_
#endif
                      )) {
        if (itr->LookUp(hand_, depth_, len, pn, dn, does_have_old_child
#ifdef ENABLE_TT_STATS
                        , &observation
#endif
                        )) {
#ifdef ENABLE_TT_STATS
          const auto old_amount = amount;
#endif
          amount = std::max(amount, itr->Amount());
#ifdef ENABLE_TT_STATS
          if (amount != old_amount) observation.AmountChanged();
#endif
          if (pn == 0) {
            Hand proof_hand = itr->GetHand();
            MateLen proven_len = itr->ProvenLen();
            if (strict_lookup) {
#ifdef ENABLE_TT_STATS_DETAIL
              observation.strict_executed = true;
#endif
              for (auto itr2 = ++itr; TT_SLOT(observation.strict_slots, !itr2->IsNull()); ++itr2) {
#ifdef ENABLE_TT_SINGLE_THREAD_LOCK_OPT
                read_lock_policy::Guard<kReadLockRequired, Entry> lock2{*itr2};
#else
                std::shared_lock lock2{*itr2};
#endif
                if (itr2->IsFor(
#ifdef ENABLE_TT_LOOKUP_KEY_CACHE
                                lookup_board_key
#else
                                board_key_
#endif
                                ) && itr2->UpdateProvenLen(hand_, proven_len)) {
                  proof_hand = itr2->GetHand();
#ifdef ENABLE_TT_STATS_DETAIL
                  ++observation.strict_len_updates;
#endif
                }
              }
            }
#ifdef ENABLE_TT_STATS
            observation.proven = true;
#endif
            return SearchResult::MakeFinal<true>(proof_hand, proven_len, amount);
          } else if (dn == 0) {
#ifdef ENABLE_TT_STATS
            observation.disproven = true;
#endif
            return SearchResult::MakeFinal<false>(itr->GetHand(), len, amount);
          } else if (itr->GetHand() == hand_) {
            if (itr->IsPossibleRepetition()) {
              if (const auto maybe_depth = rep_table_->Contains(path_key_)) {
#ifdef ENABLE_TT_STATS
                observation.repetition = true;
#endif
                return SearchResult::MakeRepetition(hand_, len, amount, *maybe_depth);
              }
            }

            found_exact = true;
            cached_entry_ = &*itr;
          }
        }
      }
    }

    if (found_exact) {
#ifdef ENABLE_INITIAL_ESTIMATE_STATS
      if (tt_stats::current) ++tt_stats::current->exact_initial_bypasses;
#endif
      return SearchResult::MakeUnknown(pn, dn, len, amount);
    }

#ifdef ENABLE_TT_STATS
    observation.initial = true;
#endif
    const auto [init_pn, init_dn] = std::forward<InitialEvalFunc>(eval_func)();
#ifdef ENABLE_INITIAL_ESTIMATE_STATS
    tt_stats::RecordInitialMerge(pn, dn, init_pn, init_dn, kOrDefenseBase);
#endif
    pn = std::max(pn, init_pn);
    dn = std::max(dn, init_dn);

    return SearchResult::MakeFirstVisit(pn, dn, len, amount);
  }
  // LCOV_EXCL_STOP NOLINTEND
#ifdef ENABLE_TT_SINGLE_THREAD_LOCK_OPT
 public:
#endif

  /**
   * @brief 探索結果 `result` を置換表に書き込む
   * @param result 探索結果
   * @note 実際の処理は `SetProven()`, `SetDisproven()`, `SetRepetition()`, `SetUnknown()` を参照。
   */
  void SetResult(const SearchResult& result) const noexcept {
    TT_DETAIL_INC(update_calls);
    if (result.Pn() == 0) {
      SetFinal<true>(result);
    } else if (result.Dn() == 0) {
      if (result.GetFinalData().IsRepetition()) {
        SetRepetition(result);
      } else {
        SetFinal<false>(result);
      }
    } else {
      SetUnknown(result);
    }
  }

  /**
   * @brief 現局面が `pr` の余る詰みかどうか
   * @param pr 駒
   * @return true `pr` が余る詰みである / false `pr` が余る詰みでない
   */
  bool IsRedundantProven(PieceType pr) const {
    Hand hand = hand_;
    sub_hand(hand, pr);
    for (auto itr = initial_entry_pointer_; !itr->IsNull(); ++itr) {
      std::shared_lock lock(*itr);
      if (itr->IsFor(board_key_)) {
        if (hand_is_equal_or_superior(hand, itr->GetHand()) && itr->ProvenLen() < MateLen::DepthMax()) {
          return true;
        }
      }
    }

    return false;
  }

 private:
  /**
   * @brief 置換表に `hand` に一致するエントリがあればそれを返し、なければ作って返す
   * @param hand 持ち駒
   * @return 見つけた or 作成したエントリ。`lock()` された状態で返るので、参照が完了したら必ず `unlock()` を呼ぶこと。
   */
  Entry* FindOrCreate(Hand hand) const noexcept {
#ifdef ENABLE_TT_STATS_DETAIL
    tt_stats::WriteObservation observation;
#endif
    if (TT_SLOT(observation.slots, !cached_entry_->IsNull())) {
      cached_entry_->lock();
      if (cached_entry_->IsFor(board_key_, hand)) {
        TT_DETAIL_INC(cached_updates);
        TT_DETAIL_INC(existing_updates);
        return cached_entry_;
      }
      cached_entry_->unlock();
    }

    for (auto itr = initial_entry_pointer_;; ++itr) {
#ifdef ENABLE_TT_STATS_DETAIL
      ++observation.slots;
#endif
      itr->lock();
      if (itr->IsNull()) {
        itr->Init(board_key_, hand, depth_);
        TT_DETAIL_INC(new_entries);
        return cached_entry_ = &*itr;
      }

      if (itr->IsFor(board_key_, hand)) {
        TT_DETAIL_INC(existing_updates);
        return cached_entry_ = &*itr;
      }
      itr->unlock();
    }
  }

  /**
   * @brief 詰みまたは不詰の探索結果 `result` を置換表に書き込む
   * @tparam kIsProven true: 詰み／false: 不詰
   * @param result 探索結果（詰み or 不詰）
   */
  template <bool kIsProven>
  void SetFinal(const SearchResult& result) const noexcept {
    const auto hand = result.GetFinalData().hand;
    auto* const entry = FindOrCreate(hand);
    const auto len = result.Len();
    const auto amount = result.Amount();

    if constexpr (kIsProven) {
      entry->UpdateProven(len, amount);
    } else {
      entry->UpdateDisproven(amount);
    }
    entry->unlock();
  }

  /**
   * @brief
   * @param result 探索結果（千日手）
   */
  void SetRepetition(const SearchResult& result) const noexcept {
    auto* const entry = FindOrCreate(hand_);

    entry->SetPossibleRepetition();
    entry->unlock();
    rep_table_->Insert(path_key_, result.GetFinalData().repetition_start);
  }

  /**
   * @brief 探索中の探索結果 `result` を置換表に書き込む関数
   * @param result 探索結果（探索中）
   */
  void SetUnknown(const SearchResult& result) const noexcept {
    const auto pn = result.Pn();
    const auto dn = result.Dn();
    const auto amount = result.Amount();

    auto* const entry = FindOrCreate(hand_);
    entry->UpdateUnknown(depth_, pn, dn, amount);
    entry->unlock();
  }

  RepetitionTable* rep_table_;                  ///< 千日手テーブル。千日手判定に用いる。
  CircularEntryPointer initial_entry_pointer_;  ///< 通常テーブルの探索開始位置への循環ポインタ
  Key path_key_;                                ///< 現局面の経路ハッシュ値
  Key board_key_;                               ///< 現局面の盤面ハッシュ値
  Hand hand_;                                   ///< 現局面の持ち駒
  Depth depth_;                                 ///< 現局面の探索深さ
  mutable Entry* cached_entry_;                 ///< 前回アクセスしたエントリ
};
}  // namespace komori::tt

#endif  // KOMORI_TTQUERY_HPP_

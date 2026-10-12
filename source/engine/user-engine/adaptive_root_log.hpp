#ifndef KOMORI_ADAPTIVE_ROOT_LOG_HPP_
#define KOMORI_ADAPTIVE_ROOT_LOG_HPP_
#include "adaptive_root_threshold.hpp"
#ifdef ENABLE_ADAPTIVE_ROOT_LOG
#include <sstream>
#include <string>
#include <vector>
namespace komori::adaptive_root {
struct LogRow {
  std::uint64_t call{}, iteration{}, nodes_begin{}, nodes_end{};
  std::uint32_t pn_begin{}, dn_begin{}, next_thpn{}, next_thdn{}, stop_flags{};
  Window window;
  Decision decision;
  bool extended{};
  const char* root_state="unknown";
};
class Log {
 public:
  void Reset() { rows_.clear(); rows_.reserve(512); calls_=dropped_=0; }
  std::uint64_t NewCall() { return ++calls_; }
  void Add(const LogRow& r) { if (rows_.size()<4096) rows_.push_back(r); else ++dropped_; }
  std::size_t Size() const { return rows_.size(); }
  std::uint64_t Dropped() const { return dropped_; }
  std::vector<std::string> Lines() const {
    std::vector<std::string> output;
    for (const auto& r:rows_) {
      std::ostringstream s;
      s << "{\"schema\":1,\"phase\":\"search\",\"thread\":0";
#define AR_LOG(n) s << ",\"" #n "\":" << r.n;
      AR_LOG(call) AR_LOG(iteration) AR_LOG(nodes_begin) AR_LOG(nodes_end)
      AR_LOG(pn_begin) AR_LOG(dn_begin) AR_LOG(next_thpn) AR_LOG(next_thdn) AR_LOG(stop_flags) AR_LOG(extended)
#undef AR_LOG
#define AR_WINDOW(n) s << ",\"" #n "\":" << r.window.n;
      AR_WINDOW(nodes) AR_WINDOW(pn) AR_WINDOW(dn) AR_WINDOW(input_thpn) AR_WINDOW(input_thdn)
      AR_WINDOW(effective_thpn) AR_WINDOW(effective_thdn) AR_WINDOW(unknown) AR_WINDOW(natural)
      AR_WINDOW(tca_changed) AR_WINDOW(tca_break)
#undef AR_WINDOW
      s << ",\"axis\":" << static_cast<std::uint32_t>(r.decision.axis)
        << ",\"previous_axis\":" << static_cast<std::uint32_t>(r.decision.previous_axis);
#define AR_DECISION(n) s << ",\"" #n "\":" << r.decision.n;
      AR_DECISION(tenths) AR_DECISION(previous_nodes) AR_DECISION(previous_exists) AR_DECISION(comparable) AR_DECISION(growth)
#undef AR_DECISION
      s << ",\"root_state\":\"" << r.root_state << "\"}";
      output.push_back(s.str());
    }
    return output;
  }
 private:
  std::vector<LogRow> rows_;
  std::uint64_t calls_{}, dropped_{};
};
}  // namespace komori::adaptive_root
#endif
#endif

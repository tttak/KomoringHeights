#ifndef KOMORI_ADAPTIVE_ROOT_THRESHOLD_HPP_
#define KOMORI_ADAPTIVE_ROOT_THRESHOLD_HPP_
#if defined(ENABLE_ADAPTIVE_ROOT_LOG) && !defined(ENABLE_ADAPTIVE_ROOT_THRESHOLD)
#error ENABLE_ADAPTIVE_ROOT_LOG requires ENABLE_ADAPTIVE_ROOT_THRESHOLD
#endif
#ifdef ENABLE_ADAPTIVE_ROOT_THRESHOLD
#include <cstdint>
namespace komori::adaptive_root {
// Only the outer, single-thread, non-PV root invocation supplies this pointer.
struct ReturnInfo {
  std::uint32_t thpn{}, thdn{};
  bool returned{}, tca_changed{}, tca_break{};
};
enum class Axis : std::uint32_t { kNone=0, kPn=1, kDn=2 };
struct Window {
  std::uint64_t nodes{};
  std::uint32_t pn{}, dn{}, input_thpn{}, input_thdn{}, effective_thpn{}, effective_thdn{};
  bool unknown{}, natural{}, tca_changed{}, tca_break{};
  Axis EligibleAxis() const {
    if (!unknown || !natural || tca_changed || tca_break || nodes < 4096 ||
        effective_thpn != input_thpn || effective_thdn != input_thdn) return Axis::kNone;
    const bool p = pn >= effective_thpn, d = dn >= effective_thdn;
    return p == d ? Axis::kNone : p ? Axis::kPn : Axis::kDn;
  }
};
struct Decision {
  int tenths{17};
  Axis axis{Axis::kNone}, previous_axis{Axis::kNone};
  std::uint64_t previous_nodes{};
  bool previous_exists{}, comparable{}, growth{};
};
class Controller {
 public:
  void Reset() { previous_={}; have_previous_=false; }
  Decision Finish(const Window& current) {
    Decision d;
    d.axis=current.EligibleAxis(); d.previous_exists=have_previous_;
    d.previous_axis=have_previous_ ? previous_.EligibleAxis() : Axis::kNone;
    d.previous_nodes=have_previous_ ? previous_.nodes : 0;
    d.comparable=have_previous_ && d.axis!=Axis::kNone && d.axis==d.previous_axis;
    // Division is exact for the >= four-times comparison and cannot overflow.
    d.growth=d.comparable && current.nodes/4 >= previous_.nodes;
    d.tenths=d.growth ? 15 : 17;
    // Always replace the adjacent window, including ineligible/stopped windows.
    previous_=current; have_previous_=true;
    return d;
  }
 private:
  Window previous_;
  bool have_previous_{};
};
}  // namespace komori::adaptive_root
#endif
#endif

#ifndef SENTRY_NAV_BT__NAVIGATION_TRANSITION_GATE_HPP_
#define SENTRY_NAV_BT__NAVIGATION_TRANSITION_GATE_HPP_

#include <cstdint>

namespace sentry_nav_bt::navigation_transition
{

uint64_t allocateToken();
void markCancelPending(uint64_t token);
void clearCancelPending(uint64_t token);
bool cancelPending(
  double timeout_s,
  bool * timed_out,
  uint64_t * pending_token,
  double * elapsed_s);

}  // namespace sentry_nav_bt::navigation_transition

#endif  // SENTRY_NAV_BT__NAVIGATION_TRANSITION_GATE_HPP_

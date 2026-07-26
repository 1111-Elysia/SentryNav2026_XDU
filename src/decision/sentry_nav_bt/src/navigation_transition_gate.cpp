#include "navigation_transition_gate.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>

namespace sentry_nav_bt::navigation_transition
{
namespace
{

// BehaviorTree 中每个 SubTree 都会创建独立的 ReliableNavigateToPose 实例。
// 取消门控必须跨实例共享，否则新目标会在旧目标确认终止前进入 Nav2 pending goal。
struct NavigationTransitionGate
{
  std::mutex mutex;
  bool cancel_pending{false};
  uint64_t pending_token{0};
  uint64_t next_token{0};
  std::chrono::steady_clock::time_point pending_since{};
};

NavigationTransitionGate & gate()
{
  static NavigationTransitionGate instance;
  return instance;
}

}  // namespace

uint64_t allocateToken()
{
  auto & state = gate();
  std::lock_guard<std::mutex> lock(state.mutex);
  return ++state.next_token;
}

void markCancelPending(uint64_t token)
{
  if (token == 0) {
    return;
  }

  auto & state = gate();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.cancel_pending = true;
  state.pending_token = token;
  state.pending_since = std::chrono::steady_clock::now();
}

void clearCancelPending(uint64_t token)
{
  if (token == 0) {
    return;
  }

  auto & state = gate();
  std::lock_guard<std::mutex> lock(state.mutex);
  if (state.cancel_pending && state.pending_token == token) {
    state.cancel_pending = false;
    state.pending_token = 0;
  }
}

bool cancelPending(
  double timeout_s,
  bool * timed_out,
  uint64_t * pending_token,
  double * elapsed_s)
{
  auto & state = gate();
  std::lock_guard<std::mutex> lock(state.mutex);

  if (timed_out) {
    *timed_out = false;
  }
  if (pending_token) {
    *pending_token = state.pending_token;
  }
  if (elapsed_s) {
    *elapsed_s = 0.0;
  }
  if (!state.cancel_pending) {
    return false;
  }

  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - state.pending_since).count();
  if (elapsed_s) {
    *elapsed_s = elapsed;
  }
  if (elapsed < std::max(timeout_s, 0.0)) {
    return true;
  }

  if (timed_out) {
    *timed_out = true;
  }
  state.cancel_pending = false;
  state.pending_token = 0;
  return false;
}

}  // namespace sentry_nav_bt::navigation_transition

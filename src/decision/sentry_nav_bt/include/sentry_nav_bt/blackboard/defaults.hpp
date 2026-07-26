#ifndef SENTRY_NAV_BT_BLACKBOARD_DEFAULTS_HPP_
#define SENTRY_NAV_BT_BLACKBOARD_DEFAULTS_HPP_

#include "behaviortree_cpp/blackboard.h"

namespace sentry_nav_bt::blackboard
{

// 建立所有运行期键，避免条件节点在首条裁判消息前读取缺失键。
void initializeDefaults(const BT::Blackboard::Ptr &blackboard);

}  // namespace sentry_nav_bt::blackboard

#endif  // SENTRY_NAV_BT_BLACKBOARD_DEFAULTS_HPP_

#ifndef SENTRY_NAV_BT_TEST_NAVIGATION_WAYPOINT_LOADER_HPP_
#define SENTRY_NAV_BT_TEST_NAVIGATION_WAYPOINT_LOADER_HPP_

#include <string>

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt_test::navigation
{

// Loads named map-frame poses into blackboard keys prefixed with "waypoint_".
bool loadWaypoints(
    const std::string &json_file_path,
    const BT::Blackboard::Ptr &blackboard,
    const rclcpp::Node::SharedPtr &node);

}  // namespace sentry_nav_bt_test::navigation

#endif  // SENTRY_NAV_BT_TEST_NAVIGATION_WAYPOINT_LOADER_HPP_

#ifndef SENTRY_NAV_BT_TEST_RANDOM_SELECTOR_NODE_HPP_
#define SENTRY_NAV_BT_TEST_RANDOM_SELECTOR_NODE_HPP_

#include <string>
#include <vector>
#include <random>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt_test
{

class RandomSelector : public BT::SyncActionNode
{
public:
  RandomSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  std::random_device rd_;
  std::mt19937 gen_;
  rclcpp::Logger logger_;
  int last_index = -1;
  
  void loadGoals();
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_RANDOM_SELECTOR_NODE_HPP_
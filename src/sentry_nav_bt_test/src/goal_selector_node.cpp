#include "sentry_nav_bt_test/goal_selector_node.hpp"

namespace sentry_nav_bt_test
{

GoalSelector::GoalSelector(
    const std::string &xml_tag_name,
    const BT::NodeConfiguration &conf)
    : BT::SyncActionNode(xml_tag_name, conf),
      logger_(rclcpp::get_logger("GoalSelector"))
{
}

BT::NodeStatus GoalSelector::tick()
{
    // 获取输入的目标点名称
    std::string goal_name;
    if (!getInput("goal_name", goal_name)) {
        RCLCPP_ERROR(logger_, "缺少必要的输入参数 'goal_name'");
        return BT::NodeStatus::FAILURE;
    }

    // 构建在黑板中查找的完整键名
    std::string blackboard_key = "waypoint_" + goal_name;
    
    // 从黑板中获取目标点
    auto blackboard = this->config().blackboard;
    geometry_msgs::msg::PoseStamped goal_pose;
    
    if (!blackboard->get(blackboard_key, goal_pose)) {
        RCLCPP_ERROR(logger_, "无法找到名为 '%s' 的目标点（键名：'%s'）", 
                   goal_name.c_str(), blackboard_key.c_str());
        return BT::NodeStatus::FAILURE;
    }
    
    // 更新时间戳
    goal_pose.header.stamp = rclcpp::Clock().now();
    
    // 设置输出
    setOutput("goal", goal_pose);
    
    const bool goal_changed =
        !has_last_logged_goal_ ||
        goal_name != last_logged_goal_name_ ||
        std::abs(goal_pose.pose.position.x - last_logged_goal_pose_.pose.position.x) > 1e-3 ||
        std::abs(goal_pose.pose.position.y - last_logged_goal_pose_.pose.position.y) > 1e-3;

    if (goal_changed) {
        RCLCPP_INFO(logger_, "已选择目标点 '%s'，位置(%.2f, %.2f)",
                   goal_name.c_str(), goal_pose.pose.position.x, goal_pose.pose.position.y);
        last_logged_goal_name_ = goal_name;
        last_logged_goal_pose_ = goal_pose;
        has_last_logged_goal_ = true;
    }
    
    return BT::NodeStatus::SUCCESS;
}

} // namespace sentry_nav_bt_test

// #include "behaviortree_cpp_v3/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<sentry_nav_bt_test::GoalSelector>("GoalSelector");
// }
#include "sentry_nav_bt/random_selector_node.hpp"

#include <sstream>
#include <utility>

namespace sentry_nav_bt
{

namespace
{

std::string trim(const std::string &value)
{
    const auto begin = value.find_first_not_of(" \t");
    if (begin == std::string::npos) {
        return "";
    }

    const auto end = value.find_last_not_of(" \t");
    return value.substr(begin, end - begin + 1);
}

}  // namespace

RandomSelector::RandomSelector(
    const std::string &xml_tag_name,
    const BT::NodeConfig &conf)
: BT::SyncActionNode(xml_tag_name, conf),
  gen_(rd_()),
  logger_(rclcpp::get_logger("RandomSelector"))
{
}

std::vector<std::string> RandomSelector::parseGoalNames(const std::string &goal_names) const
{
    std::vector<std::string> names;
    std::stringstream ss(goal_names);
    std::string item;

    while (std::getline(ss, item, ',')) {
        const auto name = trim(item);
        if (!name.empty()) {
            names.push_back(name);
        }
    }

    return names;
}

BT::NodeStatus RandomSelector::tick()
{
    auto blackboard = config().blackboard;
    if (!blackboard) {
        RCLCPP_ERROR(logger_, "无法获取黑板");
        return BT::NodeStatus::FAILURE;
    }

    std::string goal_names_raw;
    if (!getInput("goal_names", goal_names_raw)) {
        RCLCPP_ERROR(logger_, "缺少必要参数 'goal_names'");
        return BT::NodeStatus::FAILURE;
    }

    const auto goal_names = parseGoalNames(goal_names_raw);
    if (goal_names.empty()) {
        RCLCPP_ERROR(logger_, "'goal_names' 中没有可用的目标点");
        return BT::NodeStatus::FAILURE;
    }

    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> candidates;
    candidates.reserve(goal_names.size());

    for (const auto &goal_name : goal_names) {
        const std::string waypoint_key = "waypoint_" + goal_name;
        geometry_msgs::msg::PoseStamped goal_pose;
        if (!blackboard->get(waypoint_key, goal_pose)) {
            RCLCPP_ERROR(
                logger_,
                "找不到随机目标点 '%s'（黑板键 '%s'）",
                goal_name.c_str(),
                waypoint_key.c_str());
            return BT::NodeStatus::FAILURE;
        }
        candidates.emplace_back(goal_name, goal_pose);
    }

    bool avoid_repeat = true;
    getInput("avoid_repeat", avoid_repeat);

    std::uniform_int_distribution<size_t> dis(0, candidates.size() - 1);
    size_t index = dis(gen_);

    if (avoid_repeat && candidates.size() > 1) {
        while (candidates[index].first == last_goal_name_) {
            index = dis(gen_);
        }
    }

    auto selected_goal = candidates[index].second;
    selected_goal.header.stamp = rclcpp::Clock().now();
    const auto &selected_name = candidates[index].first;
    last_goal_name_ = selected_name;

    setOutput("goal", selected_goal);
    setOutput("goal_name", selected_name);

    RCLCPP_INFO(
        logger_,
        "随机选择目标点 '%s'，位置(%.2f, %.2f)",
        selected_name.c_str(),
        selected_goal.pose.position.x,
        selected_goal.pose.position.y);

    return BT::NodeStatus::SUCCESS;
}

} // namespace sentry_nav_bt

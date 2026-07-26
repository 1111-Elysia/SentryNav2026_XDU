#include "sentry_nav_bt/navigation/waypoint_loader.hpp"

#include <fstream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nlohmann/json.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace sentry_nav_bt::navigation
{

bool loadWaypoints(
    const std::string &json_file_path,
    const BT::Blackboard::Ptr &blackboard,
    const rclcpp::Node::SharedPtr &node)
{
    try {
        std::ifstream file(json_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(node->get_logger(), "无法打开目标点文件: %s", json_file_path.c_str());
            return false;
        }

        nlohmann::json document;
        file >> document;
        if (!document.is_object() || !document.contains("waypoints") ||
            !document["waypoints"].is_array())
        {
            RCLCPP_ERROR(node->get_logger(), "路径点 JSON 必须包含 waypoints 数组");
            return false;
        }

        int loaded_count = 0;
        for (const auto &waypoint : document["waypoints"]) {
            if (!waypoint.is_object() || !waypoint.contains("name") ||
                !waypoint.contains("x") || !waypoint.contains("y") ||
                !waypoint.contains("yaw"))
            {
                RCLCPP_WARN(node->get_logger(), "跳过格式不正确的目标点");
                continue;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = node->now();
            pose.pose.position.x = waypoint["x"];
            pose.pose.position.y = waypoint["y"];

            tf2::Quaternion orientation;
            orientation.setRPY(0.0, 0.0, waypoint["yaw"].get<double>());
            pose.pose.orientation.x = orientation.x();
            pose.pose.orientation.y = orientation.y();
            pose.pose.orientation.z = orientation.z();
            pose.pose.orientation.w = orientation.w();
            blackboard->set("waypoint_" + waypoint["name"].get<std::string>(), pose);
            ++loaded_count;
        }
        RCLCPP_INFO(node->get_logger(), "成功加载 %d 个目标点", loaded_count);
        return true;
    } catch (const std::exception &error) {
        RCLCPP_ERROR(node->get_logger(), "加载目标点文件时出错: %s", error.what());
        return false;
    }
}

}  // namespace sentry_nav_bt::navigation

#include "sentry_nav_bt_test/random_selector_node.hpp"

namespace sentry_nav_bt_test
{

    RandomSelector::RandomSelector(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(xml_tag_name, conf),
          gen_(rd_()),
          logger_(rclcpp::get_logger("RandomSelector"))
    {
        loadGoals();
    }

    void RandomSelector::loadGoals()
    {
        // 这里为了简单直接硬编码目标点
        // 实际应用中应该从参数文件加载这些目标点

        goals_.clear();

        auto createPose = [](double x, double y, double z, double yaw)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
            return pose;
        };

        // 添加几个示例目标点
        goals_.push_back(createPose(4.0, -5.0, 0.0, 0.0));
        goals_.push_back(createPose(4.0, -7.0, 0.0, 0.0));
        goals_.push_back(createPose(2.0, -3.5, 0.0, 0.0));
        goals_.push_back(createPose(2.0, 0.0, 0.0, 0.0));

        RCLCPP_INFO(logger_, "已加载 %zu 个目标点用于随机导航", goals_.size());
    }

    BT::NodeStatus RandomSelector::tick()
    {
        if (goals_.empty())
        {
            RCLCPP_ERROR(logger_, "没有可用的目标点进行随机选择");
            return BT::NodeStatus::FAILURE;
        }

        // 生成随机索引
        std::uniform_int_distribution<> dis(0, goals_.size() - 1);
        int index =  dis(gen_);
        while(last_index == index){
            index =  dis(gen_);
        }
        last_index = index;

        // 获取随机选择的目标点
        geometry_msgs::msg::PoseStamped selected_goal = goals_[index];
        selected_goal.header.stamp = rclcpp::Clock().now();

        // 设置输出端口
        setOutput("goal", selected_goal);

        RCLCPP_INFO(logger_, "随机选择了目标点 %d，位置(%.2f, %.2f)",
                    index, selected_goal.pose.position.x, selected_goal.pose.position.y);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace sentry_nav_bt_test

// #include "behaviortree_cpp_v3/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<sentry_nav_bt_test::RandomSelector>("RandomSelector");
// }
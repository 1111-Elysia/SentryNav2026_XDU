#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <vector>
#include <mutex>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class PubPoint : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    PubPoint() : Node("pub_point"), index_(0)
    {
        declare_parameter("points", std::vector<double>{});
        declare_parameter("interval", 2.0);  // 定时发布的时间间隔

        interval_sec_ = get_parameter("interval").as_double();

        // 读取 points 数组
        std::vector<double> flat = get_parameter("points").as_double_array();
        if (flat.size() % 3 != 0)
        {
            RCLCPP_ERROR(get_logger(),
                         "参数 points 格式必须是 3*N，如 [x,y,yaw_deg,x,y,yaw_deg...]");
            rclcpp::shutdown();
            return;
        }

        // 转换为 PoseStamped 数组
        for (size_t i = 0; i < flat.size(); i += 3)
        {
            double x = flat[i];
            double y = flat[i + 1];
            double yaw_deg = flat[i + 2];
            double yaw = yaw_deg * M_PI / 180.0;

            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;

            p.pose.orientation.z = std::sin(yaw / 2.0);
            p.pose.orientation.w = std::cos(yaw / 2.0);

            points_.push_back(p);
        }

        // 创建 Nav2 客户端
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(get_logger(), "等待 Nav2 navigate_to_pose Action 服务器...");
        client_->wait_for_action_server();

        RCLCPP_INFO(get_logger(), "服务器就绪，开始定时发送目标点...");

        // 定时器：按间隔定时发布目标点
        create_interval_timer();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> points_;
    size_t index_;
    double interval_sec_;
    std::mutex lock_;
    rclcpp::TimerBase::SharedPtr interval_timer_;

    // -------------------- 定时发送目标点 --------------------
    void send_next_goal()
    {
        std::lock_guard<std::mutex> guard(lock_);

        // 创建目标
        auto goal = NavigateToPose::Goal();
        goal.pose = points_[index_];

        RCLCPP_INFO(get_logger(),
            "发送第 %d 个目标点: x=%.2f y=%.2f",
            (int)index_,
            goal.pose.pose.position.x,
            goal.pose.pose.position.y);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // 直接发送目标，不做到点检测
        client_->async_send_goal(goal);

        // 更新目标点索引
        index_++;
        if (index_ >= points_.size())
        {
            RCLCPP_INFO(get_logger(), "所有路径点已完成，重新开始循环");
            index_ = 0;  // 无限循环回第一个点
        }
    }

    // -------------------- 定时器 --------------------
    void create_interval_timer()
    {
        interval_timer_ = create_wall_timer(
            std::chrono::duration<double>(interval_sec_),
            [this]()
            {
                send_next_goal();  // 定时发送目标点
            }
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubPoint>());
    rclcpp::shutdown();
    return 0;
}

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
        declare_parameter("timeout", 10.0);

        timeout_sec_ = get_parameter("timeout").as_double();

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

        RCLCPP_INFO(get_logger(), "服务器就绪，发送第一个点...");
        send_next_goal();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> points_;
    size_t index_;
    double timeout_sec_;
    std::mutex lock_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr oneshot_timer_;

    // -------------------- 发送下一个点 --------------------
    void send_next_goal()
    {
        std::lock_guard<std::mutex> guard(lock_);

        auto goal = NavigateToPose::Goal();
        goal.pose = points_[index_];

        RCLCPP_INFO(get_logger(),
            "发送第 %d 个目标点: x=%.2f y=%.2f",
            (int)index_,
            goal.pose.pose.position.x,
            goal.pose.pose.position.y);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        options.result_callback =
            [this](const GoalHandleNav::WrappedResult & result)
            {
                timeout_timer_.reset();

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                    RCLCPP_INFO(get_logger(), "到达目标点");
                else
                    RCLCPP_WARN(get_logger(), "Nav2 失败或终止");

                create_oneshot_timer([this]() { next_index_and_send(); });
            };

        client_->async_send_goal(goal, options);

        timeout_timer_ = create_wall_timer(
            std::chrono::duration<double>(timeout_sec_),
            [this]()
            {
                RCLCPP_WARN(get_logger(), "超时 %.1f 秒，切换下一个点", timeout_sec_);
                timeout_timer_.reset();

                create_oneshot_timer([this]() { next_index_and_send(); });
            }
        );
    }

    // -------------------- 切换下一点 --------------------
    void next_index_and_send()
    {
        index_++;

        if (index_ >= points_.size())
        {
            RCLCPP_INFO(get_logger(), "所有路径点已完成，重新开始循环");
            index_ = 0;  // 🔹 无限循环回第一个点
        }

        send_next_goal();
    }

    // -------------------- 一次性定时器 --------------------
    void create_oneshot_timer(std::function<void()> func)
    {
        oneshot_timer_ = create_wall_timer(
            1ms,
            [this, func]()
            {
                func();
                oneshot_timer_.reset();
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

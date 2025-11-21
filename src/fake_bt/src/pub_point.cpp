#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <vector>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class PubPoint : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    PubPoint() : Node("pub_point"), current_index_(0)
    {
        // 参数
        this->declare_parameter("points", std::vector<double>{});
        this->declare_parameter("timeout", 10.0);

        timeout_sec_ = this->get_parameter("timeout").as_double();

        // 读取一维数组 points
        auto flat = this->get_parameter("points").as_double_array();
        if (flat.size() % 3 != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "参数 points 必须是 3*N，如 [x,y,yaw_deg, x,y,yaw_deg...]");
            return;
        }

        // 转成二维 points_(x,y,yaw_rad)
        for (size_t i = 0; i < flat.size(); i += 3) {
            double x = flat[i];
            double y = flat[i + 1];
            double yaw_deg = flat[i + 2];
            double yaw_rad = yaw_deg * M_PI / 180.0;

            points_.push_back({x, y, yaw_rad});
        }

        // 创建 Nav2 Action 客户端
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // 等待服务器
        while (!nav_client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(this->get_logger(), "等待 Nav2 navigate_to_pose Action 服务器...");
        }

        RCLCPP_INFO(this->get_logger(), "Nav2 Action 就绪，开始发送第一个点");

        send_next_goal();
    }

private:
    std::vector<std::vector<double>> points_;
    int current_index_;
    double timeout_sec_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    // 发送下一个目标点
    void send_next_goal()
    {
        if (points_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "points 参数为空！");
            return;
        }

        const auto &pt = points_[current_index_];

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = pt[0];
        goal_msg.pose.pose.position.y = pt[1];

        // yaw 转四元数
        double yaw = pt[2];
        goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

        RCLCPP_INFO(this->get_logger(), "发送目标点 [%d]: x=%.2f y=%.2f yaw_deg=%.1f",
                    current_index_, pt[0], pt[1], pt[2] * 180.0 / M_PI);

        // 发送 action goal
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&PubPoint::result_callback, this, std::placeholders::_1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);

        // 启动超时计时器
        timeout_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timeout_sec_),
            std::bind(&PubPoint::on_timeout, this));
    }

    // Nav2 返回结果
    void result_callback(const GoalHandleNav::WrappedResult &result)
    {
        timeout_timer_->cancel();

        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "到达目标点，切换下一个点");
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Nav2 ABORTED，切换下一个点");
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Nav2 CANCELED，切换下一个点");
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Nav2 失败，切换下一个点");
            break;
        }

        next_index_and_send();
    }

    // 超时处理
    void on_timeout()
    {
        RCLCPP_WARN(this->get_logger(),
                    "超时 %.1f 秒未到点，切换下一个点", timeout_sec_);

        // 取消当前 goal
        nav_client_->async_cancel_all_goals();

        next_index_and_send();
    }

    // 切换点并发送
    void next_index_and_send()
    {
        current_index_++;
        if (current_index_ >= static_cast<int>(points_.size()))
            current_index_ = 0;

        send_next_goal();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubPoint>());
    rclcpp::shutdown();
    return 0;
}

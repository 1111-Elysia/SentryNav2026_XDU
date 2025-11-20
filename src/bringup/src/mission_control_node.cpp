#include <rclcpp/rclcpp.hpp>
#include <sentry_msgs/msg/match_stage.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <cmath>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <thread>
#include <atomic>
#include <future>
#include <chrono>
#include <mutex>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

struct TargetPose {
    double x, y, z;
    double qx, qy, qz, qw;
};

class MissionControlNode : public rclcpp::Node
{
public:
    MissionControlNode() : Node("mission_control_node")
    {
        this->declare_parameter<std::string>("match_stage_topic", "/match_stage");
        this->declare_parameter<std::string>("nav_goal_topic", "/goal_pose");
        this->declare_parameter<std::string>("navigate_action_name", "navigate_to_pose");

        this->declare_parameter<std::vector<double>>("target_poses", std::vector<double>{});
        this->declare_parameter<double>("publish_interval", 10.0);

        match_topic_ = this->get_parameter("match_stage_topic").as_string();
        goal_topic_ = this->get_parameter("nav_goal_topic").as_string();
        navigate_action_name_ = this->get_parameter("navigate_action_name").as_string();
        publish_interval_ = this->get_parameter("publish_interval").as_double();

        loadTargetPoses();

        match_stage_sub_ = this->create_subscription<sentry_msgs::msg::MatchStage>(
            match_topic_, 10,
            std::bind(&MissionControlNode::matchStageCallback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);

        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_action_name_);

        // hurt_armor 订阅与 vw 发布
        hurt_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/hurt_armor", 10,
            std::bind(&MissionControlNode::hurtCallback, this, std::placeholders::_1));

        vw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/vw", 10);

        // 启动 hurt 监控线程
        stop_node_.store(false);
        // 初始发布值为 0（持续发布 0.0 直到有非0 hurt）
        desired_state_.store(0);
        burst_end_time_ = std::chrono::steady_clock::time_point::min();
        hurt_thread_ = std::thread(&MissionControlNode::hurtMonitorThread, this);

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "任务控制节点已启动");
        RCLCPP_INFO(this->get_logger(), "监听话题: %s", match_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "目标发布话题(Topic): %s", goal_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "导航 Action 名称: %s", navigate_action_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "目标点数量: %zu", target_poses_.size());
        RCLCPP_INFO(this->get_logger(), "等待/发布间隔: %.1f 秒", publish_interval_);
        RCLCPP_INFO(this->get_logger(), "/hurt_armor 订阅: %s", "/hurt_armor");
        RCLCPP_INFO(this->get_logger(), "/vw 发布: %s", "/vw");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~MissionControlNode()
    {
        // 停止任务线程
        stop_mission_.store(true);
        if (mission_thread_.joinable()) mission_thread_.join();

        // 停止 hurt 监控线程
        stop_node_.store(true);
        {
            std::lock_guard<std::mutex> lk(burst_mutex_);
            // 唤醒线程（如果在 sleep，可通过 stop_node_ 检测退出）
        }
        if (hurt_thread_.joinable()) hurt_thread_.join();
    }

private:
    void loadTargetPoses()
    {
        auto poses_flat = this->get_parameter("target_poses").as_double_array();

        if (poses_flat.empty()) {
            RCLCPP_WARN(this->get_logger(), "未配置目标点，使用默认目标点");
            target_poses_.push_back({2.0, 1.5, 0.0, 0.0, 0.0, 0.707, 0.707});
            target_poses_.push_back({-2.0, 1.5, 0.0, 0.0, 0.0, -0.707, 0.707});
            return;
        }

        if (poses_flat.size() % 7 == 0) {
            for (size_t i = 0; i < poses_flat.size(); i += 7) {
                TargetPose pose;
                pose.x = poses_flat[i];
                pose.y = poses_flat[i + 1];
                pose.z = poses_flat[i + 2];
                pose.qx = poses_flat[i + 3];
                pose.qy = poses_flat[i + 4];
                pose.qz = poses_flat[i + 5];
                pose.qw = poses_flat[i + 6];
                target_poses_.push_back(pose);
                RCLCPP_INFO(this->get_logger(), "目标点 %zu: [%.2f, %.2f, %.2f] (quaternion)",
                            target_poses_.size(), pose.x, pose.y, pose.z);
            }
            return;
        } else if (poses_flat.size() % 4 == 0) {
            for (size_t i = 0; i < poses_flat.size(); i += 4) {
                double x = poses_flat[i];
                double y = poses_flat[i + 1];
                double z = poses_flat[i + 2];
                double yaw_deg = poses_flat[i + 3];
                double yaw = yaw_deg * M_PI / 180.0;
                double qx = 0.0;
                double qy = 0.0;
                double qz = std::sin(yaw / 2.0);
                double qw = std::cos(yaw / 2.0);
                TargetPose pose;
                pose.x = x; pose.y = y; pose.z = z;
                pose.qx = qx; pose.qy = qy; pose.qz = qz; pose.qw = qw;
                target_poses_.push_back(pose);
                RCLCPP_INFO(this->get_logger(),
                            "目标点 %zu: [%.2f, %.2f, %.2f] yaw=%.1f°",
                            target_poses_.size(), x, y, z, yaw_deg);
            }
            return;
        }

        RCLCPP_ERROR(this->get_logger(), "目标点参数数量错误，应为7的倍数(四元数)或4的倍数(含偏航角度)");
        rclcpp::shutdown();
        return;
    }

    void matchStageCallback(const sentry_msgs::msg::MatchStage::SharedPtr msg)
    {
        bool was_active = is_active_;
        is_active_ = (msg->match_stage == 4);

        if (is_active_ && !was_active) {
            RCLCPP_INFO(this->get_logger(), "检测到 match_stage == 4，开始导航任务");
            current_target_index_ = 0;
            startMissionThread();
        } else if (!is_active_ && was_active) {
            RCLCPP_INFO(this->get_logger(), "match_stage != 4，暂停导航任务");
            stop_mission_.store(true);
            if (mission_thread_.joinable()) mission_thread_.join();
            stop_mission_.store(false);
        }
    }

    void startMissionThread()
    {
        if (mission_thread_.joinable()) return;
        stop_mission_.store(false);
        mission_thread_ = std::thread([this]() {
            if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_WARN(this->get_logger(), "导航 action server (%s) 未就绪", navigate_action_name_.c_str());
            }

            while (rclcpp::ok() && is_active_ && !stop_mission_.load() && !target_poses_.empty()) {
                const auto & target = target_poses_[current_target_index_];
                bool success = sendGoalAndWait(target, publish_interval_);
                if (success) {
                    current_target_index_ = (current_target_index_ + 1) % target_poses_.size();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                } else {
                    break;
                }
            }
        });
    }

    bool sendGoalAndWait(const TargetPose& target, double timeout_seconds)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = target.x;
        pose_msg.pose.position.y = target.y;
        pose_msg.pose.position.z = target.z;
        pose_msg.pose.orientation.x = target.qx;
        pose_msg.pose.orientation.y = target.qy;
        pose_msg.pose.orientation.z = target.qz;
        pose_msg.pose.orientation.w = target.qw;

        goal_pub_->publish(pose_msg);

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "导航 action server 未响应，发布 Topic 但未发送 action goal");
            return false;
        }

        NavigateToPose::Goal goal;
        goal.pose = pose_msg;

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
        auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);

        if (goal_handle_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "发送 goal 超时");
            return false;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "goal 被拒绝或创建失败");
            return false;
        }

        auto result_future = action_client_->async_get_result(goal_handle);
        auto status = result_future.wait_for(std::chrono::duration<double>(timeout_seconds));
        if (status == std::future_status::ready) {
            auto wrapped_result = result_future.get();
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "导航到目标 %zu 成功（在等待时间内）", current_target_index_ + 1);
                return true;
            } else {
                RCLCPP_INFO(this->get_logger(), "导航到目标 %zu 完成，但结果 code=%d", current_target_index_ + 1, static_cast<int>(wrapped_result.code));
                return false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "在 %.1f 秒内未收到导航结果（目标 %zu）", timeout_seconds, current_target_index_ + 1);
            return false;
        }
    }

    // hurt_armor 回调：更新最新值并在非0 时设置/延长 5 秒的发布窗口，持续发布由 desired_state_ 决定的值
    void hurtCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        last_hurt_.store(msg->data);
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(burst_mutex_);
        if (msg->data != 0) {
            // 将结束时间设置为现在 + 5s（如果已有窗口则延长）
            burst_end_time_ = now + std::chrono::seconds(5);
            // 期望发布值切换到 1.0（并持续发布）
            desired_state_.store(1);
        } else {
            // msg==0 时不立即切换为 0，因为需要等到窗口结束后再切换
            // 窗口到期逻辑在 hurtMonitorThread 中处理
        }
    }

    // 监控线程：始终以 ~10Hz 持续发布当前 desired_state（1 -> 1.0，0 -> 0.0）。
    // 当窗口到期（now > burst_end_time_）且 last_hurt_==0 时切换 desired_state 为 0。
    void hurtMonitorThread()
    {
        const std::chrono::milliseconds loop_sleep(100); // 10Hz
        while (rclcpp::ok() && !stop_node_.load()) {
            {
                std::lock_guard<std::mutex> lk(burst_mutex_);
                auto now = std::chrono::steady_clock::now();
                // 如果窗口到期且没有收到新的 hurt，则设置为 0（持续发布 0.0）
                if (now > burst_end_time_ && last_hurt_.load() == 0) {
                    desired_state_.store(0);
                }
            }

            // 持续发布当前 desired_state（无论是否与之前相同）
            int state = desired_state_.load();
            std_msgs::msg::Float64 m;
            m.data = (state == 1) ? 1.0 : 0.0;
            vw_pub_->publish(m);

            std::this_thread::sleep_for(loop_sleep);
        }
    }

private:
    rclcpp::Subscription<sentry_msgs::msg::MatchStage>::SharedPtr match_stage_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    // hurt/vw
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hurt_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vw_pub_;

    std::vector<TargetPose> target_poses_;
    size_t current_target_index_ = 0;
    bool is_active_ = false;

    std::string match_topic_;
    std::string goal_topic_;
    std::string navigate_action_name_;
    double publish_interval_ = 10.0;

    std::thread mission_thread_;
    std::atomic<bool> stop_mission_{false};

    // hurt 监控相关
    std::thread hurt_thread_;
    std::atomic<bool> stop_node_{false};
    std::atomic<int> last_hurt_{0};        // 最近的 hurt_armor 值
    std::atomic<int> desired_state_{0};    // 0 -> 发布 0.0；1 -> 发布 1.0（持续发布直到改变）
    std::mutex burst_mutex_;
    std::chrono::steady_clock::time_point burst_end_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}
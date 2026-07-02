#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <mutex>
#include <string>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class TargetFrameNode : public rclcpp::Node
{
public:
  TargetFrameNode()
  : rclcpp::Node("target_frame"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("target_frame", "target_frame");
    this->declare_parameter<std::string>("target_pose_topic", "/detected_target_pose");
    this->declare_parameter<std::string>("planner_name_topic", "/planner_name");
    this->declare_parameter<bool>("auto_navigate", true);

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 1e-6) publish_rate_hz = 50.0;

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();
    planner_name_topic_ = this->get_parameter("planner_name_topic").as_string();
    auto_navigate_ = this->get_parameter("auto_navigate").as_bool();

    yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target/yaw", 10,
      [this](const std_msgs::msg::Float32::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mutex_);
        yaw_ = m->data;
      });

    pitch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target/pitch", 10,
      [this](const std_msgs::msg::Float32::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mutex_);
        pitch_ = m->data;
      });

    distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target/distance", 10,
      [this](const std_msgs::msg::Float32::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mutex_);
        distance_ = m->data;
      });

    // 监听 planner 切换
    planner_name_sub_ = this->create_subscription<std_msgs::msg::String>(
      planner_name_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr m) {
        auto prev = current_planner_;
        current_planner_ = m->data;
        RCLCPP_INFO(this->get_logger(), "Planner switched → %s", m->data.c_str());

        if (m->data == "ElasticTracker") {
          tryAutoNavigate();
        } else if (prev == "ElasticTracker" && nav_goal_handle_) {
          // 切离 ElasticTracker → 取消追踪
          RCLCPP_INFO(this->get_logger(), "Cancel tracking, planner switched away from ElasticTracker");
          nav_action_client_->async_cancel_goal(nav_goal_handle_);
          nav_goal_handle_.reset();
          nav_goal_active_ = false;
        }
      });

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10);

    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    const double freq = std::max(publish_rate_hz, 1.0);
    const auto period = std::chrono::duration<double>(1.0 / freq);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TargetFrameNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "target_frame node started | tf: %s→%s | pose: %s | auto_nav: %s @ %.1f Hz",
      base_frame_.c_str(), target_frame_.c_str(), target_pose_topic_.c_str(),
      auto_navigate_ ? "ON" : "OFF", freq);
  }

private:
  static double deg2rad(const double deg) { return deg * M_PI / 180.0; }
  static double clampAngleDeg(double deg)
  {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
  }

  void tryAutoNavigate()
  {
    if (!auto_navigate_) return;
    if (current_planner_ != "ElasticTracker") return;
    if (nav_goal_active_) return;
    if (!last_pose_in_map_valid_) return;
    if (!nav_action_client_->action_server_is_ready()) return;

    // 冷却检查
    auto now = this->now();
    if ((now - last_nav_send_time_).seconds() < nav_cooldown_s_) return;
    last_nav_send_time_ = now;

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = last_pose_in_map_;
    goal_msg.behavior_tree = "";

    RCLCPP_INFO(this->get_logger(),
      "Auto NavigateToPose (%.2f, %.2f)",
      last_pose_in_map_.pose.position.x, last_pose_in_map_.pose.position.y);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & handle) {
        nav_goal_active_ = true;
        nav_goal_handle_ = handle;
      };
    opts.result_callback =
      [this](const GoalHandleNav::WrappedResult &) {
        nav_goal_active_ = false;
        nav_goal_handle_.reset();
      };
    nav_action_client_->async_send_goal(goal_msg, opts);
  }

  void onTimer()
  {
    float yaw_deg, pitch_deg, distance;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      yaw_deg  = yaw_;
      pitch_deg = pitch_;
      distance = distance_;
    }

    // 距离 ≤ 0 表示无有效目标，不发布
    if (distance <= 0.0f) {
      last_pose_in_map_valid_ = false;
      return;
    }

    const double pitch_rad = deg2rad(static_cast<double>(pitch_deg));

    // 水平距离
    const double real_distance = static_cast<double>(distance) * std::cos(pitch_rad);

    // yaw: 输入0°=base_link后方 → 加180°换到base_link前向基准，右手定则
    const double corrected_yaw_deg = clampAngleDeg(static_cast<double>(yaw_deg) + 180.0);
    const double corrected_yaw_rad = deg2rad(corrected_yaw_deg);

    // target_frame 在 base_link 下的位置
    const double x_base = real_distance * std::cos(corrected_yaw_rad);
    const double y_base = real_distance * std::sin(corrected_yaw_rad);
    const double z_base = static_cast<double>(distance) * std::sin(pitch_rad);

    // 发布 base_link → target_frame 的 TF
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = this->now();
      tf.header.frame_id = base_frame_;
      tf.child_frame_id = target_frame_;
      tf.transform.translation.x = x_base;
      tf.transform.translation.y = y_base;
      tf.transform.translation.z = z_base;
      tf.transform.rotation.x = 0.0;
      tf.transform.rotation.y = 0.0;
      tf.transform.rotation.z = 0.0;
      tf.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(tf);
    }

    // 转换到 map 系并发布 PoseStamped
    geometry_msgs::msg::PoseStamped pose_in_base;
    pose_in_base.header.frame_id = base_frame_;
    pose_in_base.header.stamp = builtin_interfaces::msg::Time();
    pose_in_base.pose.position.x = x_base;
    pose_in_base.pose.position.y = y_base;
    pose_in_base.pose.position.z = z_base;
    pose_in_base.pose.orientation.x = 0.0;
    pose_in_base.pose.orientation.y = 0.0;
    pose_in_base.pose.orientation.z = 0.0;
    pose_in_base.pose.orientation.w = 1.0;

    try {
      auto pose_in_map = tf_buffer_.transform(pose_in_base, map_frame_);
      pose_in_map.header.stamp = this->now();
      target_pose_pub_->publish(pose_in_map);

      // 缓存最新目标位置
      last_pose_in_map_ = pose_in_map;
      last_pose_in_map_valid_ = true;

      // 首次有目标 + planner 已经是 ElasticTracker → 自动触发
      tryAutoNavigate();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "TF transform failed (%s→%s): %s",
        base_frame_.c_str(), map_frame_.c_str(), ex.what());
    }
  }

private:
  std::string map_frame_;
  std::string base_frame_;
  std::string target_frame_;
  std::string target_pose_topic_;
  std::string planner_name_topic_;
  bool auto_navigate_ = true;

  std::mutex mutex_;
  float yaw_{0.0f};
  float pitch_{0.0f};
  float distance_{0.0f};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_name_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  GoalHandleNav::SharedPtr nav_goal_handle_;
  bool nav_goal_active_ = false;
  rclcpp::Time last_nav_send_time_{0, 0, RCL_ROS_TIME};
  double nav_cooldown_s_ = 1.0;

  std::string current_planner_ = "GridBased";
  geometry_msgs::msg::PoseStamped last_pose_in_map_;
  bool last_pose_in_map_valid_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetFrameNode>());
  rclcpp::shutdown();
  return 0;
}

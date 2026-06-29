#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

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

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 1e-6) publish_rate_hz = 50.0;

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();

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

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10);

    const double freq = std::max(publish_rate_hz, 1.0);
    const auto period = std::chrono::duration<double>(1.0 / freq);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TargetFrameNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "target_frame node started | tf: %s→%s | pose_topic: %s @ %.1f Hz",
      base_frame_.c_str(), target_frame_.c_str(), target_pose_topic_.c_str(), freq);
  }

private:
  static double deg2rad(const double deg) { return deg * M_PI / 180.0; }
  static double clampAngleDeg(double deg)
  {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
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
    pose_in_base.header.stamp = this->now();
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

  std::mutex mutex_;
  float yaw_{0.0f};
  float pitch_{0.0f};
  float distance_{0.0f};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

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

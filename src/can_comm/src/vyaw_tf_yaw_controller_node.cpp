#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <string>

using namespace std::chrono_literals;

class VyawTfYawControllerNode final : public rclcpp::Node
{
public:
  VyawTfYawControllerNode()
  : rclcpp::Node("vyaw_tf_yaw_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    this->declare_parameter<std::string>("target_yaw_topic", "/target_yaw");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // Pose of NLJG in map frame.
    this->declare_parameter<double>("NLJG_pose_x", 0.0);
    this->declare_parameter<double>("NLJG_pose_y", 0.0);

    // Pose of outpost in map frame.
    this->declare_parameter<double>("outpost_pose_x", 0.0);
    this->declare_parameter<double>("outpost_pose_y", 0.0);

    this->declare_parameter<double>("publish_rate_hz", 30.0);

    // 下位机实时 yaw 订阅，用于零飘补偿
    this->declare_parameter<std::string>("diankong_yaw_topic", "/diankong/yaw");
    this->declare_parameter<double>("diankong_yaw_timeout_s", 0.5);

    target_yaw_topic_ = this->get_parameter("target_yaw_topic").as_string();
    std::string diankong_yaw_topic = this->get_parameter("diankong_yaw_topic").as_string();
    diankong_yaw_timeout_s_ = this->get_parameter("diankong_yaw_timeout_s").as_double();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    NLJG_pose_x_ = this->get_parameter("NLJG_pose_x").as_double();
    NLJG_pose_y_ = this->get_parameter("NLJG_pose_y").as_double();

    outpost_pose_x_ = this->get_parameter("outpost_pose_x").as_double();
    outpost_pose_y_ = this->get_parameter("outpost_pose_y").as_double();

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 1e-6) publish_rate_hz = 30.0;

    target_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(target_yaw_topic_, 10);

    yaw_enable_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/yaw_controller",
      10,
      std::bind(&VyawTfYawControllerNode::onYawEnable, this, std::placeholders::_1));

    diankong_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      diankong_yaw_topic, 10,
      [this](const std_msgs::msg::Float32::SharedPtr m) {
        diankong_yaw_deg_ = m->data;
        last_diankong_yaw_time_ = this->now();
        diankong_yaw_received_ = true;
      });

    const double freq = std::max(publish_rate_hz, 1.0);
    const auto period = std::chrono::duration<double>(1.0 / freq);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VyawTfYawControllerNode::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started. target_yaw on %s @ %.1f Hz",
      target_yaw_topic_.c_str(), freq);

    RCLCPP_INFO(
      this->get_logger(),
      "NLJG_pose: x=%.3f, y=%.3f",
      NLJG_pose_x_, NLJG_pose_y_);

    RCLCPP_INFO(
      this->get_logger(),
      "outpost_pose: x=%.3f, y=%.3f",
      outpost_pose_x_, outpost_pose_y_);

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for /yaw_controller msg (0=NLJG, 1=outpost) to start.");
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing diankong_yaw on %s for zero-drift compensation",
      diankong_yaw_topic.c_str());
  }

  ~VyawTfYawControllerNode() override
  {
    if (active_) {
      publishTargetYaw(0.0);
    }
  }

private:
  static double rad2deg(const double rad)
  {
    return rad * 180.0 / M_PI;
  }

  // 归一化到 (-180, 180]
  static double normalizeDeg(const double deg)
  {
    double d = std::fmod(deg + 180.0, 360.0);
    if (d <= 0.0) d += 360.0;
    return d - 180.0;
  }

  void publishTargetYaw(const double target_yaw_deg)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(target_yaw_deg);
    target_yaw_pub_->publish(msg);
  }

  bool lookupCurrentPoseMap(double & x_out, double & y_out, double & yaw_out)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      x_out = tf.transform.translation.x;
      y_out = tf.transform.translation.y;
      tf2::Quaternion q;
      q.setX(tf.transform.rotation.x);
      q.setY(tf.transform.rotation.y);
      q.setZ(tf.transform.rotation.z);
      q.setW(tf.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      yaw_out = yaw;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "TF lookup failed (%s->%s): %s",
        map_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  double computeTargetYawDeg(const double base_x_map, const double base_y_map)
  {
    // 以目标为起点、车体为终点的向量 (车体 - 目标) 在 map 系下的方向角
    // atan2 区间 (-π, π]，俯视逆时针为正
    const double target_yaw_rad = std::atan2(base_y_map - target_y_, base_x_map - target_x_);
    return rad2deg(target_yaw_rad);
  }

  void onYawEnable(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 0) {
      target_x_ = NLJG_pose_x_;
      target_y_ = NLJG_pose_y_;
      active_ = true;
      has_target_yaw_ = false;
      RCLCPP_INFO(this->get_logger(), "Triggered by /yaw_controller=0. Using NLJG_pose.");
    } else if (msg->data == 1) {
      target_x_ = outpost_pose_x_;
      target_y_ = outpost_pose_y_;
      active_ = true;
      has_target_yaw_ = false;
      RCLCPP_INFO(this->get_logger(), "Triggered by /yaw_controller=1. Using outpost_pose.");
    }
  }

  void onTimer()
  {
    if (!active_) {
      return;
    }

    double base_x_map = 0.0;
    double base_y_map = 0.0;
    double current_yaw_rad = 0.0;
    if (!lookupCurrentPoseMap(base_x_map, base_y_map, current_yaw_rad)) {
      return;
    }

    // 首次：用当前机器人位置计算目标角度，锁定不再变化
    if (!has_target_yaw_) {
      target_yaw_deg_ = computeTargetYawDeg(base_x_map, base_y_map);
      has_target_yaw_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Computed target yaw: %.2f deg (locked, keeps publishing)",
        target_yaw_deg_);
    }

    // ── 零飘补偿：下位机 yaw − 导航 TF yaw ，每个周期重算 ──
    double yaw_offset_deg = 0.0;
    const auto now = this->now();
    const bool diankong_fresh =
      diankong_yaw_received_ &&
      (now - last_diankong_yaw_time_).seconds() <= diankong_yaw_timeout_s_;

    if (diankong_fresh) {
      yaw_offset_deg = normalizeDeg(diankong_yaw_deg_ - rad2deg(current_yaw_rad));
    } else if (diankong_yaw_received_) {
      // 下位机 yaw 超时，保持最后已知偏移
      yaw_offset_deg = normalizeDeg(diankong_yaw_deg_ - rad2deg(current_yaw_rad));
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "diankong_yaw 超时(%.2fs)，保持最后偏移",
        diankong_yaw_timeout_s_);
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "尚未收到 diankong_yaw，暂不补偿零飘");
    }

    // 持续发送补偿后的目标角度，直到下次 /yaw_controller 触发
    const double compensated_yaw = normalizeDeg(target_yaw_deg_ + yaw_offset_deg);
    publishTargetYaw(compensated_yaw);
  }

private:
  std::string target_yaw_topic_;
  std::string map_frame_;
  std::string base_frame_;

  double target_x_ = 0.0;
  double target_y_ = 0.0;

  double NLJG_pose_x_ = 0.0;
  double NLJG_pose_y_ = 0.0;

  double outpost_pose_x_ = 0.0;
  double outpost_pose_y_ = 0.0;

  bool active_ = false;
  bool has_target_yaw_ = false;
  double target_yaw_deg_ = 0.0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_yaw_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr yaw_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr diankong_yaw_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float diankong_yaw_deg_ = 0.0f;
  bool diankong_yaw_received_ = false;
  rclcpp::Time last_diankong_yaw_time_{0, 0, RCL_ROS_TIME};
  double diankong_yaw_timeout_s_ = 0.5;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VyawTfYawControllerNode>());
  rclcpp::shutdown();
  return 0;
}

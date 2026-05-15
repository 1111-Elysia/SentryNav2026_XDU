#include <rclcpp/rclcpp.hpp>

#include <rm_referee_msgs/msg/hurt_data.hpp>
#include <sentry_msgs/msg/vw.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

class HurtSpinVwNode : public rclcpp::Node
{
public:
  HurtSpinVwNode() : Node("hurt_spin_vw_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    this->declare_parameter<std::string>("vw_out_topic", "/vw");
    this->declare_parameter<std::string>("hurt_data_topic", "/rm_referee/hurt_data");

    this->declare_parameter<int>("hurt_reason_value", 0);
    this->declare_parameter<double>("spin_vw", 1.0);
    this->declare_parameter<double>("spin_duration_s", 5.0);
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    // 坡度检测参数
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("max_slope_angle_deg", 10.0);

    const auto vw_out_topic = this->get_parameter("vw_out_topic").as_string();
    const auto hurt_data_topic = this->get_parameter("hurt_data_topic").as_string();

    hurt_reason_value_ = this->get_parameter("hurt_reason_value").as_int();
    spin_vw_ = static_cast<float>(this->get_parameter("spin_vw").as_double());
    spin_duration_s_ = this->get_parameter("spin_duration_s").as_double();

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    max_slope_angle_deg_ = std::abs(this->get_parameter("max_slope_angle_deg").as_double());

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 1e-6) publish_rate_hz = 50.0;

    vw_pub_ = this->create_publisher<sentry_msgs::msg::Vw>(vw_out_topic, 10);

    hurt_sub_ = this->create_subscription<rm_referee_msgs::msg::HurtData>(
      hurt_data_topic,
      rclcpp::SensorDataQoS(),
      [this](const rm_referee_msgs::msg::HurtData::SharedPtr msg) {
        if (!msg) return;
        if (msg->hp_deduction_reason != static_cast<uint8_t>(hurt_reason_value_)) {
          return;
        }

        const auto now = this->now();
        {
          std::lock_guard<std::mutex> lk(mutex_);
          hurt_active_ = true;
          hurt_start_time_ = now;
        }

        // 立刻发布一次，减少响应延迟；后续由定时器持续发布
        publishVw(spin_vw_);
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HurtSpinVwNode::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "HurtSpinVwNode started | vw_out=%s hurt=%s | normal_vw=0 | slope_limit=%.1f deg",
      vw_out_topic.c_str(),
      hurt_data_topic.c_str(),
      max_slope_angle_deg_);
  }

private:
  void publishVw(float vw)
  {
    sentry_msgs::msg::Vw msg;
    msg.vw = vw;
    vw_pub_->publish(msg);
  }

  bool isOnSlope()
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);

      tf2::Quaternion q;
      q.setX(tf.transform.rotation.x);
      q.setY(tf.transform.rotation.y);
      q.setZ(tf.transform.rotation.z);
      q.setW(tf.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // 综合 roll 和 pitch 作为坡度角（取绝对值较大的那个）
      const double slope_rad = std::max(std::abs(roll), std::abs(pitch));
      const double slope_deg = slope_rad * 180.0 / M_PI;

      return slope_deg > max_slope_angle_deg_;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF lookup failed (%s->%s): %s",
        map_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      // TF 不可用时保守处理：视为非斜坡（允许旋转）
      return false;
    }
  }

  void onTimer()
  {
    const auto now = this->now();

    float out_vw = 0.0f;
    {
      std::lock_guard<std::mutex> lk(mutex_);

      if (hurt_active_) {
        const double dt = (now - hurt_start_time_).seconds();
        if (dt < spin_duration_s_) {
          out_vw = spin_vw_;
        } else {
          hurt_active_ = false;
          out_vw = 0.0f;
        }
      }
    }

    // 坡度检测：有坡度时强制 vw=0，避免侧翻
    if (out_vw != 0.0f && isOnSlope()) {
      out_vw = 0.0f;
    }

    // 非受击时 vw 恒为 0；受击时在 spin_duration_s 内输出 spin_vw（无坡度时）
    publishVw(out_vw);
  }

private:
  rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
  rclcpp::Subscription<rm_referee_msgs::msg::HurtData>::SharedPtr hurt_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;

  int hurt_reason_value_{0};
  float spin_vw_{1.0f};
  double spin_duration_s_{5.0};

  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  double max_slope_angle_deg_{10.0};

  bool hurt_active_{false};
  rclcpp::Time hurt_start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HurtSpinVwNode>());
  rclcpp::shutdown();
  return 0;
}

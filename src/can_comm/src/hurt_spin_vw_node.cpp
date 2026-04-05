#include <rclcpp/rclcpp.hpp>

#include <sentry_msgs/msg/vw.hpp>
#include <rm_referee_msgs/msg/hurt_data.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

class HurtSpinVwNode : public rclcpp::Node
{
public:
  HurtSpinVwNode() : Node("hurt_spin_vw_node")
  {
    this->declare_parameter<std::string>("vw_in_topic", "/vw");
    this->declare_parameter<std::string>("vw_out_topic", "/vw_to_can");
    this->declare_parameter<std::string>("hurt_data_topic", "/rm_referee/hurt_data");

    this->declare_parameter<int>("hurt_reason_value", 0);
    this->declare_parameter<double>("spin_vw", 1.0);
    this->declare_parameter<double>("spin_duration_s", 5.0);
    this->declare_parameter<double>("post_hurt_default_vw", 0.3);
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    const auto vw_in_topic = this->get_parameter("vw_in_topic").as_string();
    const auto vw_out_topic = this->get_parameter("vw_out_topic").as_string();
    const auto hurt_data_topic = this->get_parameter("hurt_data_topic").as_string();

    hurt_reason_value_ = this->get_parameter("hurt_reason_value").as_int();
    spin_vw_ = static_cast<float>(this->get_parameter("spin_vw").as_double());
    spin_duration_s_ = this->get_parameter("spin_duration_s").as_double();
    post_hurt_default_vw_ = static_cast<float>(this->get_parameter("post_hurt_default_vw").as_double());

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 1e-6) publish_rate_hz = 50.0;

    vw_pub_ = this->create_publisher<sentry_msgs::msg::Vw>(vw_out_topic, 10);

    vw_in_sub_ = this->create_subscription<sentry_msgs::msg::Vw>(
      vw_in_topic,
      10,
      [this](const sentry_msgs::msg::Vw::SharedPtr msg) {
        if (!msg) return;

        const auto now = this->now();
        bool publish_now = false;
        float out_vw = 0.0f;

        {
          std::lock_guard<std::mutex> lk(mutex_);
          vw_input_received_once_ = true;
          last_input_vw_ = msg->vw;
          last_input_time_ = now;

          // 新的输入 vw 到达后，退出“受击结束后默认值覆盖”模式
          if (post_hurt_override_active_) {
            post_hurt_override_active_ = false;
          }

          // 受击强制旋转期间不放行输入；否则直接转发
          if (!hurt_active_ && !post_hurt_override_active_) {
            publish_now = true;
            out_vw = last_input_vw_;
          }
        }

        if (publish_now) {
          publishVw(out_vw);
        }
      });

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

    RCLCPP_INFO(this->get_logger(), "HurtSpinVwNode started | vw_in=%s vw_out=%s hurt=%s",
      vw_in_topic.c_str(), vw_out_topic.c_str(), hurt_data_topic.c_str());
  }

private:
  void publishVw(float vw)
  {
    sentry_msgs::msg::Vw msg;
    msg.vw = vw;
    vw_pub_->publish(msg);
  }

  void onTimer()
  {
    const auto now = this->now();

    bool publish = false;
    float out_vw = 0.0f;

    {
      std::lock_guard<std::mutex> lk(mutex_);

      if (hurt_active_) {
        const double dt = (now - hurt_start_time_).seconds();
        if (dt < spin_duration_s_) {
          publish = true;
          out_vw = spin_vw_;
        } else {
          // 受击结束：仅当历史上收到过至少一次输入 vw 时，启用 0.3 默认覆盖
          hurt_active_ = false;
          post_hurt_override_active_ = vw_input_received_once_;
        }
      }

      if (!hurt_active_ && post_hurt_override_active_) {
        publish = true;
        out_vw = post_hurt_default_vw_;
      }
    }

    if (publish) {
      publishVw(out_vw);
    }
  }

private:
  rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
  rclcpp::Subscription<sentry_msgs::msg::Vw>::SharedPtr vw_in_sub_;
  rclcpp::Subscription<rm_referee_msgs::msg::HurtData>::SharedPtr hurt_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  int hurt_reason_value_{0};
  float spin_vw_{1.0f};
  double spin_duration_s_{5.0};
  float post_hurt_default_vw_{0.3f};

  bool vw_input_received_once_{false};
  float last_input_vw_{0.0f};
  rclcpp::Time last_input_time_;

  bool hurt_active_{false};
  rclcpp::Time hurt_start_time_;

  bool post_hurt_override_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HurtSpinVwNode>());
  rclcpp::shutdown();
  return 0;
}

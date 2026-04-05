#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sentry_msgs/msg/vw.hpp>

#include <algorithm>
#include <cmath>
#include <string>

class VxVyToVwNode : public rclcpp::Node
{
public:
  VxVyToVwNode() : Node("vxvy_to_vw_node")
  {
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("vw_topic", "/vw");

    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    const auto vw_topic = this->get_parameter("vw_topic").as_string();

    vw_pub_ = this->create_publisher<sentry_msgs::msg::Vw>(vw_topic, 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!msg) return;

        const double x = msg->linear.x;
        const double y = msg->linear.y;

        // w = sqrt(1 - (x^2 + y^2)/2)
        double inside = 1.0 - (x * x + y * y) / 2.0;
        inside = std::max(0.0, inside); // avoid NaN when cmd_vel is large

        const float w = static_cast<float>(std::sqrt(inside));

        sentry_msgs::msg::Vw out;
        out.vw = w;
        vw_pub_->publish(out);
      });

    RCLCPP_INFO(this->get_logger(), "VxVyToVwNode started | cmd_vel=%s vw=%s",
      cmd_vel_topic.c_str(), vw_topic.c_str());
  }

private:
  rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VxVyToVwNode>());
  rclcpp::shutdown();
  return 0;
}

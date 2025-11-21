#include "rclcpp/rclcpp.hpp"
#include "sentry_msgs/msg/hurt_armor.hpp"
#include "sentry_msgs/msg/vw.hpp"

using namespace std::chrono_literals;

class PubVw : public rclcpp::Node
{
public:
    PubVw() : Node("pub_vw"), active_(false)
    {
        // 订阅 hurt_armor（sentry_msgs::msg::HurtArmor）
        hurt_sub_ = this->create_subscription<sentry_msgs::msg::HurtArmor>(
            "hurt_armor", 10,
            std::bind(&PubVw::hurt_callback, this, std::placeholders::_1));

        // 发布 vw（sentry_msgs::msg::Vw）
        vw_pub_ = this->create_publisher<sentry_msgs::msg::Vw>("vw", 10);

        // 定时器以固定频率发布（持续发布 1.0 或 0.0）
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PubVw::on_timer, this));
    }

private:
    rclcpp::Subscription<sentry_msgs::msg::HurtArmor>::SharedPtr hurt_sub_;
    rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool active_;

    // 回调只更新状态并打印日志，不再在此发送一次性 0.0
    void hurt_callback(const sentry_msgs::msg::HurtArmor::SharedPtr msg)
    {
        bool was_active = active_;
        active_ = (msg->hurt_armor != 0);
        if (active_ && !was_active) {
            RCLCPP_INFO(this->get_logger(), "hurt_armor != 0, 开始持续发布 vw=1.0");
        } else if (!active_ && was_active) {
            RCLCPP_INFO(this->get_logger(), "hurt_armor == 0, 切换为持续发布 vw=0.0");
        }
    }

    // 定时器持续发布 vw：active_ 为 true 则 1.0，否则 0.0
    void on_timer()
    {
        auto out = std::make_shared<sentry_msgs::msg::Vw>();
        out->vw = active_ ? 1.0f : 0.0f;
        vw_pub_->publish(*out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubVw>());
    rclcpp::shutdown();
    return 0;
}


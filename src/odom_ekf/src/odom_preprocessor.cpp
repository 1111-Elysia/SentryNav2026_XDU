#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

struct OdomConfig
{
    std::string input_topic;
    std::string output_topic;
    std::string new_frame_id;
    std::string new_child_frame_id;
    std::vector<double> pose_cov;
    std::vector<double> twist_cov;
};

class OdomPreprocessor : public rclcpp::Node
{
public:
    OdomPreprocessor() : Node("odom_preprocessor")
    {
        // 声明里程计列表参数
        this->declare_parameter<std::vector<std::string>>("odom_list", {"odom0"});

        std::vector<std::string> odom_names;
        this->get_parameter("odom_list", odom_names);

        for (const auto &name : odom_names)
        {
            OdomConfig cfg;

            // 声明每个里程计的参数
            this->declare_parameter<std::string>(name + ".input_topic", "/odom");
            this->declare_parameter<std::string>(name + ".output_topic", "/odom_processed");
            this->declare_parameter<std::string>(name + ".new_frame_id", "odom");
            this->declare_parameter<std::string>(name + ".new_child_frame_id", "base_link");
            this->declare_parameter<std::vector<double>>(name + ".pose_covariance", std::vector<double>(36, 0.0));
            this->declare_parameter<std::vector<double>>(name + ".twist_covariance", std::vector<double>(36, 0.0));

            // 获取参数
            this->get_parameter(name + ".input_topic", cfg.input_topic);
            this->get_parameter(name + ".output_topic", cfg.output_topic);
            this->get_parameter(name + ".new_frame_id", cfg.new_frame_id);
            this->get_parameter(name + ".new_child_frame_id", cfg.new_child_frame_id);
            this->get_parameter(name + ".pose_covariance", cfg.pose_cov);
            this->get_parameter(name + ".twist_covariance", cfg.twist_cov);

            // 协方差检查
            if (cfg.pose_cov.size() != 36)
            {
                RCLCPP_WARN(this->get_logger(), "%s pose_covariance size !=36, using default", name.c_str());
                cfg.pose_cov.assign(36, 0.0);
                for (int i = 0; i < 36; i += 7)
                    cfg.pose_cov[i] = 1e-3;
            }
            if (cfg.twist_cov.size() != 36)
            {
                RCLCPP_WARN(this->get_logger(), "%s twist_covariance size !=36, using default", name.c_str());
                cfg.twist_cov.assign(36, 0.0);
                for (int i = 0; i < 36; i += 7)
                    cfg.twist_cov[i] = 1e-3;
            }

            // 创建 publisher
            auto pub = this->create_publisher<nav_msgs::msg::Odometry>(cfg.output_topic, 10);

            // [修改] 创建 subscription，lambda 捕获 this, cfg 和 pub
            auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
                cfg.input_topic, 10,
                [this, cfg, pub](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    nav_msgs::msg::Odometry new_msg = *msg;
                    
                    // [关键修改] 使用当前节点时间重写时间戳，防止 EKF 丢弃"过时"数据
                    new_msg.header.stamp = this->now(); 
                    
                    new_msg.header.frame_id = cfg.new_frame_id;
                    new_msg.child_frame_id = cfg.new_child_frame_id;
                    std::copy(cfg.pose_cov.begin(), cfg.pose_cov.end(), new_msg.pose.covariance.begin());
                    std::copy(cfg.twist_cov.begin(), cfg.twist_cov.end(), new_msg.twist.covariance.begin());
                    pub->publish(new_msg);
                });

            subs_.push_back(sub);
        }

        RCLCPP_INFO(this->get_logger(), "Odom Preprocessor started for %zu odoms", odom_names.size());
    }

private:
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
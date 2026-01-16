#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>
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

// [新增] IMU 配置结构体
struct ImuConfig
{
    std::string input_topic;
    std::string output_topic;
    std::vector<double> orientation_cov;
    std::vector<double> angular_velocity_cov;
    std::vector<double> linear_acceleration_cov;
};

class OdomPreprocessor : public rclcpp::Node
{
public:
    OdomPreprocessor() : Node("odom_preprocessor")
    {
        // === 1. 处理里程计列表 ===
        this->declare_parameter<std::vector<std::string>>("odom_list", {"odom0"});
        std::vector<std::string> odom_names;
        this->get_parameter("odom_list", odom_names);

        for (const auto &name : odom_names)
        {
            OdomConfig cfg;
            this->declare_parameter<std::string>(name + ".input_topic", "/odom");
            this->declare_parameter<std::string>(name + ".output_topic", "/odom_processed");
            this->declare_parameter<std::string>(name + ".new_frame_id", "odom");
            this->declare_parameter<std::string>(name + ".new_child_frame_id", "base_link");
            this->declare_parameter<std::vector<double>>(name + ".pose_covariance", std::vector<double>(36, 0.0));
            this->declare_parameter<std::vector<double>>(name + ".twist_covariance", std::vector<double>(36, 0.0));

            this->get_parameter(name + ".input_topic", cfg.input_topic);
            this->get_parameter(name + ".output_topic", cfg.output_topic);
            this->get_parameter(name + ".new_frame_id", cfg.new_frame_id);
            this->get_parameter(name + ".new_child_frame_id", cfg.new_child_frame_id);
            this->get_parameter(name + ".pose_covariance", cfg.pose_cov);
            this->get_parameter(name + ".twist_covariance", cfg.twist_cov);

            if (cfg.pose_cov.size() != 36) { cfg.pose_cov.assign(36, 0.0); for(int i=0; i<36; i+=7) cfg.pose_cov[i] = 1e-3; }
            if (cfg.twist_cov.size() != 36) { cfg.twist_cov.assign(36, 0.0); for(int i=0; i<36; i+=7) cfg.twist_cov[i] = 1e-3; }

            auto pub = this->create_publisher<nav_msgs::msg::Odometry>(cfg.output_topic, 10);
            auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
                cfg.input_topic, 10,
                [this, cfg, pub](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    nav_msgs::msg::Odometry new_msg = *msg;
                    new_msg.header.stamp = this->now();
                    new_msg.header.frame_id = cfg.new_frame_id;
                    new_msg.child_frame_id = cfg.new_child_frame_id;
                    std::copy(cfg.pose_cov.begin(), cfg.pose_cov.end(), new_msg.pose.covariance.begin());
                    std::copy(cfg.twist_cov.begin(), cfg.twist_cov.end(), new_msg.twist.covariance.begin());
                    pub->publish(new_msg);
                });
            odom_subs_.push_back(sub);
        }
        
        // === 2. 处理 IMU 协方差填充 ===
        ImuConfig imu_cfg;
        this->declare_parameter<std::string>("imu_input_topic", "/livox/imu");
        this->declare_parameter<std::string>("imu_output_topic", "/imu_processed");
        this->declare_parameter<std::vector<double>>("imu_orientation_covariance", std::vector<double>(9, 0.0));
        this->declare_parameter<std::vector<double>>("imu_angular_velocity_covariance", std::vector<double>(9, 0.0));
        this->declare_parameter<std::vector<double>>("imu_linear_acceleration_covariance", std::vector<double>(9, 0.0));

        this->get_parameter("imu_input_topic", imu_cfg.input_topic);
        this->get_parameter("imu_output_topic", imu_cfg.output_topic);
        this->get_parameter("imu_orientation_covariance", imu_cfg.orientation_cov);
        this->get_parameter("imu_angular_velocity_covariance", imu_cfg.angular_velocity_cov);
        this->get_parameter("imu_linear_acceleration_covariance", imu_cfg.linear_acceleration_cov);

        // 默认值检查 (如果 yaml 没配或者格式不对，使用默认 1e-3)
        if (imu_cfg.orientation_cov.size() != 9) {
            imu_cfg.orientation_cov = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
        }
        if (imu_cfg.angular_velocity_cov.size() != 9) {
            imu_cfg.angular_velocity_cov = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
        }
        if (imu_cfg.linear_acceleration_cov.size() != 9) {
            imu_cfg.linear_acceleration_cov = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
        }

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_cfg.output_topic, 10);
        
        // 捕获 imu_cfg
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_cfg.input_topic, 10,
            [this, imu_cfg](const sensor_msgs::msg::Imu::SharedPtr msg) {
                sensor_msgs::msg::Imu new_msg = *msg;
                new_msg.header.stamp = this->now(); 

                // 如果原始 msg 里的协方差是 0 (表示未知)，则覆盖它
                // 或者无论如何都强制覆盖，取决于需求。这里逻辑是：如果第一个元素是0，就覆盖。
                
                if (new_msg.orientation_covariance[0] == 0.0) {
                     std::copy(imu_cfg.orientation_cov.begin(), imu_cfg.orientation_cov.end(), new_msg.orientation_covariance.begin());
                }
                
                if (new_msg.angular_velocity_covariance[0] == 0.0) {
                     std::copy(imu_cfg.angular_velocity_cov.begin(), imu_cfg.angular_velocity_cov.end(), new_msg.angular_velocity_covariance.begin());
                }
                
                if (new_msg.linear_acceleration_covariance[0] == 0.0) {
                     std::copy(imu_cfg.linear_acceleration_cov.begin(), imu_cfg.linear_acceleration_cov.end(), new_msg.linear_acceleration_covariance.begin());
                }

                imu_pub_->publish(new_msg);
            });

        RCLCPP_INFO(this->get_logger(), "Odom Preprocessor started for %zu odoms + 1 IMU", odom_names.size());
    }

private:
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
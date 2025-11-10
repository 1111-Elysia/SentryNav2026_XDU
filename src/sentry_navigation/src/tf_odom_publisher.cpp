#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher() : Node("tf_odom_publisher")
    {
        // 初始化 TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 初始化静态 TF 广播器
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // 创建 odom 话题发布器
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // 订阅 /Odometry 话题
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&TfOdomPublisher::odometryCallback, this, std::placeholders::_1));
        
        // 订阅 /tf 话题（用于监听 map->odom 变换）
        tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            std::bind(&TfOdomPublisher::tfCallback, this, std::placeholders::_1));
        
        // 发布静态 TF: base_link -> livox_frame
        publishStaticTransform();
        
        RCLCPP_INFO(this->get_logger(), "TF Odom Publisher initialized");
        RCLCPP_INFO(this->get_logger(), "  Subscribing to /Odometry, publishing to /odom");
        RCLCPP_INFO(this->get_logger(), "  Published static TF: base_link -> livox_frame");
    }

private:
    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "livox_frame";
        
        // 单位变换（如果雷达相对于base_link有偏移，请修改这些值）
        static_transform.transform.translation.x = 0.1;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(static_transform);
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 直接转发 /Odometry 到 /odom
        odom_publisher_->publish(*msg);
    }

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        // 遍历所有变换，查找 map -> odom
        for (const auto& transform : msg->transforms)
        {
            if (transform.header.frame_id == "map" && transform.child_frame_id == "odom")
            {
                auto current_time = this->now();
                
                // 发布 odom -> base_link 的单位变换
                geometry_msgs::msg::TransformStamped odom_to_baselink;
                odom_to_baselink.header.stamp = current_time;
                odom_to_baselink.header.frame_id = "odom";
                odom_to_baselink.child_frame_id = "base_link";
                
                // 零偏移和单位旋转
                odom_to_baselink.transform.translation.x = 0.0;
                odom_to_baselink.transform.translation.y = 0.0;
                odom_to_baselink.transform.translation.z = 0.0;
                odom_to_baselink.transform.rotation.x = 0.0;
                odom_to_baselink.transform.rotation.y = 0.0;
                odom_to_baselink.transform.rotation.z = 0.0;
                odom_to_baselink.transform.rotation.w = 1.0;
                
                tf_broadcaster_->sendTransform(odom_to_baselink);
                
                break;
            }
        }
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

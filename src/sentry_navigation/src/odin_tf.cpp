#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rm_referee_msgs/msg/game_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <cmath>

class OdinTfNode : public rclcpp::Node
{
public:
    OdinTfNode()
        : Node("odin_tf")
    {
        this->declare_parameter<std::string>("odin_frame", "odin1_base_link");
        this->declare_parameter<double>("publish_rate", 100.0);
        this->declare_parameter<double>("base_link_to_odin1_base_link_x", 0.22);
        this->declare_parameter<double>("base_link_to_odin1_base_link_y", 0.0);
        this->declare_parameter<double>("base_link_to_odin1_base_link_z", 0.0);
        this->declare_parameter<double>("base_link_to_odin1_base_link_roll", 0.0);
        this->declare_parameter<double>("base_link_to_odin1_base_link_pitch", 45.0);
        this->declare_parameter<double>("base_link_to_odin1_base_link_yaw", 0.0);

        this->get_parameter("odin_frame", odin_frame_);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        game_status_sub_ = this->create_subscription<rm_referee_msgs::msg::GameStatus>(
            "/rm_referee/game_status", 10, std::bind(&OdinTfNode::game_status_callback, this, std::placeholders::_1));

        scan_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/scan_mode", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&OdinTfNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Odin TF node has been started.");
    }

private:
    void game_status_callback(const rm_referee_msgs::msg::GameStatus::SharedPtr msg)
    {
        if (msg->game_progress == 2)
        {
            std_msgs::msg::Bool scan_mode_msg;
            scan_mode_msg.data = true;
            scan_mode_pub_->publish(scan_mode_msg);
            RCLCPP_INFO(this->get_logger(), "Game progress is 2, publishing /scan_mode = true");
        }
        else if (msg->game_progress == 4)
        {
            // Check if the transform exists in the buffer (non-blocking)
            if (!tf_buffer_->canTransform("map", "odom", tf2::TimePointZero))
            {
                geometry_msgs::msg::TransformStamped static_transform_stamped;
                static_transform_stamped.header.stamp = this->get_clock()->now();
                static_transform_stamped.header.frame_id = "map";
                static_transform_stamped.child_frame_id = "odom";
                static_transform_stamped.transform.translation.x = 0.0;
                static_transform_stamped.transform.translation.y = 0.0;
                static_transform_stamped.transform.translation.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                q.setW(1);
                static_transform_stamped.transform.rotation = tf2::toMsg(q);
                static_tf_broadcaster_->sendTransform(static_transform_stamped);
                RCLCPP_WARN(this->get_logger(), "Game progress is 4 and map->odom transform not available. Publishing static transform.");
            }
        }
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped map_to_odin_transform;
        try
        {
            map_to_odin_transform = tf_buffer_->lookupTransform("map", odin_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_DEBUG(this->get_logger(), "Could not get transform from map to %s: %s", odin_frame_.c_str(), ex.what());
            return;
        }

        tf2::Transform T_map_odin;
        tf2::fromMsg(map_to_odin_transform.transform, T_map_odin);

        double x = this->get_parameter("base_link_to_odin1_base_link_x").as_double();
        double y = this->get_parameter("base_link_to_odin1_base_link_y").as_double();
        double z = this->get_parameter("base_link_to_odin1_base_link_z").as_double();
        double roll = this->get_parameter("base_link_to_odin1_base_link_roll").as_double() * M_PI / 180.0;
        double pitch = this->get_parameter("base_link_to_odin1_base_link_pitch").as_double() * M_PI / 180.0;
        double yaw = this->get_parameter("base_link_to_odin1_base_link_yaw").as_double() * M_PI / 180.0;

        tf2::Transform T_base_odin;
        T_base_odin.setOrigin(tf2::Vector3(x, y, z));
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        T_base_odin.setRotation(q);

        tf2::Transform T_map_base = T_map_odin * T_base_odin.inverse();

        try
        {
            geometry_msgs::msg::TransformStamped map_to_odom_transform = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
            tf2::Transform T_map_odom;
            tf2::fromMsg(map_to_odom_transform.transform, T_map_odom);
            
            tf2::Transform T_odom_base = T_map_odom.inverse() * T_map_base;

            geometry_msgs::msg::TransformStamped odom_to_base_transform;
            odom_to_base_transform.header.stamp = this->get_clock()->now();
            odom_to_base_transform.header.frame_id = "odom";
            odom_to_base_transform.child_frame_id = "base_link";
            odom_to_base_transform.transform = tf2::toMsg(T_odom_base);
            tf_broadcaster_->sendTransform(odom_to_base_transform);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_DEBUG(this->get_logger(), "Could not get transform from map to odom: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Subscription<rm_referee_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_mode_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string odin_frame_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdinTfNode>());
    rclcpp::shutdown();
    return 0;
}

/**
 * @file field_to_odom_tf_node.cpp
 * @brief Publish rm_field -> odom static transform selected by robot_id.
 */

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_referee_msgs/msg/robot_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cstdint>
#include <memory>
#include <string>

class FieldToOdomTfNode : public rclcpp::Node {
 public:
  FieldToOdomTfNode() : Node("field_to_odom_tf_node") {
    declare_parameter<std::string>("field_frame", "rm_field");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<double>("red.field_to_odom_x", 0.0);
    declare_parameter<double>("red.field_to_odom_y", 0.0);
    declare_parameter<double>("red.field_to_odom_z", 0.0);
    declare_parameter<double>("red.field_to_odom_roll", 0.0);
    declare_parameter<double>("red.field_to_odom_pitch", 0.0);
    declare_parameter<double>("red.field_to_odom_yaw", 0.0);
    declare_parameter<double>("blue.field_to_odom_x", 0.0);
    declare_parameter<double>("blue.field_to_odom_y", 0.0);
    declare_parameter<double>("blue.field_to_odom_z", 0.0);
    declare_parameter<double>("blue.field_to_odom_roll", 0.0);
    declare_parameter<double>("blue.field_to_odom_pitch", 0.0);
    declare_parameter<double>("blue.field_to_odom_yaw", 0.0);

    field_frame_ = get_parameter("field_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    robot_status_sub_ = create_subscription<rm_referee_msgs::msg::RobotStatus>(
        "/rm_referee/robot_status",
        rclcpp::SensorDataQoS(),
        [this](const rm_referee_msgs::msg::RobotStatus::SharedPtr msg) {
          robot_status_callback(msg);
        });

    RCLCPP_INFO(get_logger(),
                "等待 /rm_referee/robot_status 发布 %s -> %s",
                field_frame_.c_str(), odom_frame_.c_str());
  }

 private:
  struct TransformConfig {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
  };

  void robot_status_callback(const rm_referee_msgs::msg::RobotStatus::SharedPtr msg) {
    const uint8_t robot_id = msg->robot_id;
    if (robot_id != 7 && robot_id != 107) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "robot_id=%u 非哨兵机器人，等待哨兵机器人状态",
                       static_cast<unsigned>(robot_id));
      return;
    }

    if (published_robot_id_ == robot_id) {
      return;
    }

    const bool is_red = robot_id == 7;
    const TransformConfig config = load_config(is_red ? "red" : "blue");
    publish_transform(config, is_red ? "red" : "blue", robot_id);
    published_robot_id_ = robot_id;
  }

  TransformConfig load_config(const std::string &prefix) const {
    TransformConfig config;
    config.x = get_parameter(prefix + ".field_to_odom_x").as_double();
    config.y = get_parameter(prefix + ".field_to_odom_y").as_double();
    config.z = get_parameter(prefix + ".field_to_odom_z").as_double();
    config.roll = get_parameter(prefix + ".field_to_odom_roll").as_double();
    config.pitch = get_parameter(prefix + ".field_to_odom_pitch").as_double();
    config.yaw = get_parameter(prefix + ".field_to_odom_yaw").as_double();
    return config;
  }

  void publish_transform(const TransformConfig &config,
                         const std::string &side,
                         uint8_t robot_id) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = field_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = config.x;
    transform.transform.translation.y = config.y;
    transform.transform.translation.z = config.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(config.roll, config.pitch, config.yaw);
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    broadcaster_->sendTransform(transform);
    RCLCPP_INFO(get_logger(),
                "发布 %s -> %s %s 哨兵机器人 (robot_id=%u): "
                "x=%.3f, y=%.3f, z=%.3f, roll=%.6f, pitch=%.6f, yaw=%.6f rad",
                field_frame_.c_str(), odom_frame_.c_str(), side.c_str(),
                static_cast<unsigned>(robot_id), config.x, config.y, config.z,
                config.roll, config.pitch, config.yaw);
  }

  rclcpp::Subscription<rm_referee_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::string field_frame_{"rm_field"};
  std::string odom_frame_{"odom"};
  uint8_t published_robot_id_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FieldToOdomTfNode>());
  rclcpp::shutdown();
  return 0;
}

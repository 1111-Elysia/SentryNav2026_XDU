/**
 * @file teammate_frame_converter.cpp
 * @brief 监听 /rm_referee/ground_robot_position（裁判系统发布的队友坐标，官方地图系），
 *        将其从官方地图系转换到纯里程计系 (odom)，并发布转换后的坐标供导航等模块使用。
 *
 * 转换关系（与 ground_pos_relay_node 中 get_self_official_position 互为逆运算）：
 *   官方坐标 = origin + R(origin_yaw) * odom坐标
 *   ⇒ odom坐标 = R(origin_yaw)^T * (官方坐标 - origin)
 *
 *   其中 R(θ) = [cosθ  -sinθ; sinθ  cosθ]
 *   R^T(θ)   = [cosθ   sinθ; -sinθ  cosθ]
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <rm_referee_msgs/msg/ground_robot_position.hpp>
#include <rm_referee_msgs/msg/robot_status.hpp>

#include <cmath>
#include <string>
#include <utility>

class TeammateFrameConverter : public rclcpp::Node {
 public:
  TeammateFrameConverter() : Node("teammate_frame_converter") {
    // ---- 参数声明 ----
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("red_origin_x", 0.0);
    declare_parameter("red_origin_y", 0.0);
    declare_parameter("red_origin_yaw", 0.0);
    declare_parameter("blue_origin_x", 0.0);
    declare_parameter("blue_origin_y", 0.0);
    declare_parameter("blue_origin_yaw", 0.0);

    odom_frame_id_ = get_parameter("odom_frame_id").as_string();

    red_origin_x_ = get_parameter("red_origin_x").as_double();
    red_origin_y_ = get_parameter("red_origin_y").as_double();
    red_origin_yaw_ = get_parameter("red_origin_yaw").as_double();
    blue_origin_x_ = get_parameter("blue_origin_x").as_double();
    blue_origin_y_ = get_parameter("blue_origin_y").as_double();
    blue_origin_yaw_ = get_parameter("blue_origin_yaw").as_double();

    // ---- 订阅 robot_status 获取本机 ID，用于判断红蓝阵营 ----
    robot_status_sub_ = create_subscription<rm_referee_msgs::msg::RobotStatus>(
        "/rm_referee/robot_status",
        rclcpp::SensorDataQoS(),
        [this](const rm_referee_msgs::msg::RobotStatus::SharedPtr msg) {
          robot_id_ = msg->robot_id;
          RCLCPP_INFO_ONCE(get_logger(), "Got robot_id: %d", robot_id_);
        });

    // ---- 订阅官方地图系中的队友坐标 ----
    ground_pos_sub_ = create_subscription<rm_referee_msgs::msg::GroundRobotPosition>(
        "/rm_referee/ground_robot_position",
        rclcpp::SensorDataQoS(),
        [this](const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg) {
          ground_pos_callback(msg);
        });

    // ---- 发布转换到 odom 系后的队友坐标 ----
    ground_pos_odom_pub_ = create_publisher<rm_referee_msgs::msg::GroundRobotPosition>(
        "/ground_pos_relay/teammate_pos_odom",
        rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
                "TeammateFrameConverter started. "
                "Subscribing /rm_referee/ground_robot_position (official frame) -> "
                "publishing /ground_pos_relay/teammate_pos_odom (odom frame).");
  }

 private:
  /**
   * @brief 将一对官方地图系坐标转换为 odom 系坐标
   * @param official_x 官方地图系 x
   * @param official_y 官方地图系 y
   * @param origin_x   阵营原点 x
   * @param origin_y   阵营原点 y
   * @param cos_yaw    cos(origin_yaw)
   * @param sin_yaw    sin(origin_yaw)
   * @return (odom_x, odom_y)
   */
  static std::pair<float, float> officialToOdom(
      float official_x, float official_y,
      double origin_x, double origin_y,
      double cos_yaw, double sin_yaw) {
    double dx = static_cast<double>(official_x) - origin_x;
    double dy = static_cast<double>(official_y) - origin_y;
    float odom_x = static_cast<float>(cos_yaw * dx + sin_yaw * dy);
    float odom_y = static_cast<float>(-sin_yaw * dx + cos_yaw * dy);
    return {odom_x, odom_y};
  }

  void ground_pos_callback(const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg) {
    // 等待 robot_id 有效
    if (robot_id_ != 7 && robot_id_ != 107) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "robot_id=%d, not 7(red) or 107(blue), skipping conversion",
                       static_cast<int>(robot_id_));
      return;
    }

    // 根据阵营选择对应的原点参数
    double origin_x, origin_y, origin_yaw;
    if (robot_id_ == 7) {  // 红方
      origin_x = red_origin_x_;
      origin_y = red_origin_y_;
      origin_yaw = red_origin_yaw_;
    } else {  // 蓝方
      origin_x = blue_origin_x_;
      origin_y = blue_origin_y_;
      origin_yaw = blue_origin_yaw_;
    }

    const double cos_yaw = std::cos(origin_yaw);
    const double sin_yaw = std::sin(origin_yaw);

    // 转换所有队友坐标
    auto out_msg = std::make_shared<rm_referee_msgs::msg::GroundRobotPosition>();
    out_msg->header = msg->header;
    out_msg->header.frame_id = odom_frame_id_;

    // 英雄
    std::tie(out_msg->hero_x, out_msg->hero_y) =
        officialToOdom(msg->hero_x, msg->hero_y, origin_x, origin_y, cos_yaw, sin_yaw);

    // 工程
    std::tie(out_msg->engineer_x, out_msg->engineer_y) =
        officialToOdom(msg->engineer_x, msg->engineer_y, origin_x, origin_y, cos_yaw, sin_yaw);

    // 步兵 3
    std::tie(out_msg->standard_3_x, out_msg->standard_3_y) =
        officialToOdom(msg->standard_3_x, msg->standard_3_y, origin_x, origin_y, cos_yaw, sin_yaw);

    // 步兵 4
    std::tie(out_msg->standard_4_x, out_msg->standard_4_y) =
        officialToOdom(msg->standard_4_x, msg->standard_4_y, origin_x, origin_y, cos_yaw, sin_yaw);

    // 哨兵（reserved 字段存放哨兵官方坐标）
    std::tie(out_msg->reserved, out_msg->reserved_2) =
        officialToOdom(msg->reserved, msg->reserved_2, origin_x, origin_y, cos_yaw, sin_yaw);

    ground_pos_odom_pub_->publish(*out_msg);

    RCLCPP_DEBUG(get_logger(),
                 "Converted hero=(%.2f,%.2f)->(%.2f,%.2f)  engineer=(%.2f,%.2f)->(%.2f,%.2f)",
                 msg->hero_x, msg->hero_y, out_msg->hero_x, out_msg->hero_y,
                 msg->engineer_x, msg->engineer_y, out_msg->engineer_x, out_msg->engineer_y);
  }

  // ---- 订阅 ----
  rclcpp::Subscription<rm_referee_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<rm_referee_msgs::msg::GroundRobotPosition>::SharedPtr ground_pos_sub_;

  // ---- 发布 ----
  rclcpp::Publisher<rm_referee_msgs::msg::GroundRobotPosition>::SharedPtr ground_pos_odom_pub_;

  // ---- 状态 ----
  uint8_t robot_id_{7};

  // ---- 参数 ----
  std::string odom_frame_id_;
  double red_origin_x_{0.0};
  double red_origin_y_{0.0};
  double red_origin_yaw_{0.0};
  double blue_origin_x_{0.0};
  double blue_origin_y_{0.0};
  double blue_origin_yaw_{0.0};
};

// ============================================================================
// main
// ============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeammateFrameConverter>());
  rclcpp::shutdown();
  return 0;
}

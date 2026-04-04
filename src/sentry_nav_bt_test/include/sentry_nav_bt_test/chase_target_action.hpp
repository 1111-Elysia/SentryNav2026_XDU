#ifndef SENTRY_NAV_BT_TEST_CHASE_TARGET_ACTION_HPP_
#define SENTRY_NAV_BT_TEST_CHASE_TARGET_ACTION_HPP_

#include <atomic>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sentry_nav_bt_test
{


class ChaseTargetAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  enum class ChaseStatus : uint8_t
  {
    IDLE = 0,
    CHASING = 1,
    HOLD = 2,
    LOST = 3,
    ERROR = 4
  };

  ChaseTargetAction(const std::string &name,
                    const BT::NodeConfiguration &config,
                    const rclcpp::Node::SharedPtr &node);

  ~ChaseTargetAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      // -------------------- input --------------------
      BT::InputPort<std::string>("aim_topic", "/autoaim/target_bl",
                                 "自瞄目标话题（base_link 下 x,y）"),
      BT::InputPort<std::string>("world_frame", "map",
                                 "导航目标所在坐标系（map 或 odom，需与 Nav2 配置一致）"),
      BT::InputPort<std::string>("base_frame", "base_link",
                                 "机器人底盘坐标系"),
      BT::InputPort<std::string>("nav_action", "navigate_to_pose",
                                 "Nav2 action 名称"),
      BT::InputPort<double>("standoff", 1.2,
                            "站位距离：目标点沿方向退 standoff 米"),
      BT::InputPort<double>("stop_dist", 1.0,
                            "进入 HOLD 的距离阈值（目标距离<=stop_dist 则停止发点并保持近距离观察）"),
      BT::InputPort<double>("start_dist", 1.6,
                            "从 HOLD 恢复追击的距离阈值（滞回，建议 > stop_dist）"),
      BT::InputPort<double>("lost_timeout", 0.6,
                            "丢失目标超时（秒）：超过该时间未收到自瞄点则 LOST 并 cancel"),
      BT::InputPort<double>("update_thresh", 0.6,
                            "目标更新阈值（米）：新旧 world 目标距离 < 阈值则不更新"),
      BT::InputPort<double>("max_rate", 3.0,
                            "最大发送频率（Hz）：限制 goal 更新频率以减少重规划"),
      BT::InputPort<double>("ema_alpha", 0.35,
                            "EMA 滤波系数（0~1），越大越跟手"),
      BT::InputPort<double>("max_jump", 2.5,
                            "输入跳变剔除阈值（米）：base_link 下相邻测量跳变过大则丢弃"),

      // -------------------- output --------------------
      BT::OutputPort<int>("chase_status", "追击状态输出：0=IDLE 1=CHASING 2=HOLD(近距离保持) 3=LOST 4=ERROR"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("last_goal", "最近一次发布的 world 目标点（可用于调试/可视化）")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ---- internal helpers ----
  void ensureInitialized_();
  void ensureSubscriptions_(const std::string &topic);
  void ensureTf_();
  void ensureNavClient_(const std::string &action_name);

  void onAimPoint_(const geometry_msgs::msg::Point::SharedPtr msg);

  geometry_msgs::msg::PoseStamped makeStandOffGoalBL_(const geometry_msgs::msg::Point &p_bl,
                                                      const rclcpp::Time &stamp,
                                                      double standoff) const;

  bool shouldUpdateGoal_(const geometry_msgs::msg::PoseStamped &goal_world,
                         const rclcpp::Time &now,
                         double update_thresh,
                         double max_rate_hz) const;

  void sendNavGoal_(const geometry_msgs::msg::PoseStamped &goal_world);
  void cancelAllGoals_();

  void setStatus_(ChaseStatus s);

private:
  // ROS node (shared from runner)
  rclcpp::Node::SharedPtr node_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Nav2 action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Subscription to aim topic
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_aim_;

  // ---- state / cache ----
  mutable std::mutex mtx_;

  // runtime config (cached from ports on first start; can be refreshed if desired)
  std::string aim_topic_;
  std::string world_frame_;
  std::string base_frame_;
  std::string nav_action_;

  double standoff_{1.2};
  double stop_dist_{1.0};
  double start_dist_{1.6};
  double lost_timeout_{0.6};
  double update_thresh_{0.6};
  double max_rate_{3.0};
  double ema_alpha_{0.35};
  double max_jump_{2.5};

  // measurement cache
  rclcpp::Time last_recv_time_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::Point> last_meas_raw_;
  std::optional<geometry_msgs::msg::Point> meas_filt_;

  // goal cache (world frame)
  rclcpp::Time last_goal_send_time_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_world_;

  // status
  std::atomic<uint8_t> status_{static_cast<uint8_t>(ChaseStatus::IDLE)};

  std::atomic<int> abort_count_{0};

  // init guard
  bool initialized_{false};
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_CHASE_TARGET_ACTION_HPP_

// sentry_eval_bt/src/bt_nodes.cpp

#include <memory>
#include <chrono>
#include <vector>
#include <cmath>
#include <mutex>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp" 

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

//-------------------------------------------------------
// 全局上下文
//-------------------------------------------------------
struct BtRosContext
{
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client;


  // 可视化发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vis_pub;

  // 速度发布者 
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  // TF 监听器
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  std::vector<geometry_msgs::msg::PoseStamped> points;
  size_t current_index{0};

  std::mutex nav_mutex;

  static BtRosContext &instance()
  {
    static BtRosContext ctx;
    return ctx;
  }
};

//=======================================================
// InitPoints：初始化路点
//=======================================================
class InitPoints : public BT::SyncActionNode
{
public:
  InitPoints(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("points_param")};
  }

  BT::NodeStatus tick() override
  {
    auto &ctx = BtRosContext::instance();
    if (!ctx.node) throw BT::RuntimeError("InitPoints: ROS node not set");

    static bool initialized = false;
    if (initialized) return BT::NodeStatus::SUCCESS;

    std::string points_param_name;
    getInput("points_param", points_param_name);

    if (!ctx.node->has_parameter(points_param_name)) {
      ctx.node->declare_parameter(points_param_name, std::vector<double>{});
    }
    auto flat = ctx.node->get_parameter(points_param_name).as_double_array();

    if (flat.size() % 3 != 0 || flat.empty()) {
      RCLCPP_ERROR(ctx.node->get_logger(), "InitPoints: points 必须是 3*N");
      return BT::NodeStatus::FAILURE;
    }

    ctx.points.clear();
    for (size_t i = 0; i < flat.size(); i += 3) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = flat[i];
      p.pose.position.y = flat[i + 1];
      double yaw = flat[i + 2] * M_PI / 180.0;
      p.pose.orientation.z = std::sin(yaw / 2.0);
      p.pose.orientation.w = std::cos(yaw / 2.0);
      ctx.points.push_back(p);
    }

    ctx.current_index = 0;
    RCLCPP_INFO(ctx.node->get_logger(), "InitPoints: loaded %zu points", ctx.points.size());
    initialized = true;
    return BT::NodeStatus::SUCCESS;
  }
};

//=======================================================
// NextPoint：获取当前目标点
//=======================================================
class NextPoint : public BT::SyncActionNode
{
public:
  NextPoint(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<size_t>("idx"),
      BT::OutputPort<bool>("is_last")
    };
  }

  BT::NodeStatus tick() override
  {
    auto &ctx = BtRosContext::instance();
    if (ctx.points.empty()) return BT::NodeStatus::FAILURE;

    if (ctx.current_index >= ctx.points.size()) {
      RCLCPP_INFO(ctx.node->get_logger(), "NextPoint: 所有点已跑完，任务结束！");
      ctx.current_index = 0;
      return BT::NodeStatus::FAILURE;
    }
    size_t idx = ctx.current_index;
    auto goal = ctx.points[idx];
    goal.header.stamp = ctx.node->now();

    setOutput("goal", goal);
    setOutput("idx", idx);
    setOutput("is_last", (idx == ctx.points.size() - 1));
    ctx.current_index++;
    return BT::NodeStatus::SUCCESS;
  }
};

//=======================================================
// CheckDistance：检查距离 
//=======================================================
class CheckDistance : public BT::StatefulActionNode
{
public:
  CheckDistance(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<double>("threshold", 0.5, "Distance threshold"),
        BT::InputPort<double>("final_threshold", 0.3, "Final point threshold"),
        BT::InputPort<bool>("is_last"),
        BT::InputPort<size_t>("idx")
      };
  }

  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }

  BT::NodeStatus onRunning() override
  {
    auto &ctx = BtRosContext::instance();
    if (!ctx.tf_buffer) return BT::NodeStatus::FAILURE;

    // 1. 确保 vel_pub 存在 (为了终点刹车)
    if (!ctx.vel_pub && ctx.node) {
        ctx.vel_pub = ctx.node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);
    }

    geometry_msgs::msg::PoseStamped goal;

    double xml_threshold = 0.5;
    if (!getInput("goal", goal)) return BT::NodeStatus::FAILURE;
    getInput("threshold", xml_threshold);

    double final_xml_threshold = 0.3;
    getInput("final_threshold", final_xml_threshold);

    try {
      geometry_msgs::msg::TransformStamped t;
      t = ctx.tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

      double dx = goal.pose.position.x - t.transform.translation.x;
      double dy = goal.pose.position.y - t.transform.translation.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      // --- 关键逻辑：区分普通点和终点 ---
      bool is_last_point = false;
      getInput("is_last", is_last_point);
      double effective_threshold = xml_threshold;
      std::string mode_str = "巡逻模式";

      size_t cur_idx = 0;
      getInput("idx", cur_idx);

      if (is_last_point) {
        effective_threshold = final_xml_threshold; 
        mode_str = "终点锁定";
      }
      
      if (dist < effective_threshold) {
        
        if (is_last_point) {
            RCLCPP_WARN(ctx.node->get_logger(), "🛑 到达终点 (%.2fm) -> 触发强制急刹！", dist);

            // 向底盘发 0 速度
            if (ctx.vel_pub) {
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.linear.y = 0.0;
                stop_msg.angular.z = 0.0;
                ctx.vel_pub->publish(stop_msg);
                ctx.vel_pub->publish(stop_msg);
                ctx.vel_pub->publish(stop_msg);//连发三次
            }
        } 
        else {
            RCLCPP_INFO(ctx.node->get_logger(),
                    "\n>>> [到达 P%zu] %s | 距离 %.2fm (阈值 %.2f) <<<\n",
                    cur_idx, mode_str.c_str(), dist, effective_threshold);
        }

        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }
    catch (const tf2::TransformException &ex) {
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override {}
};

//=======================================================
// SendNav2Goal：发送目标 
//=======================================================
class SendNav2Goal : public BT::StatefulActionNode
{
public:
  SendNav2Goal(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }

  enum class InternalState { IDLE, SENDING, ACCEPTED, REJECTED };

  BT::NodeStatus onStart() override
  {
    internal_state_ = InternalState::IDLE;
    retry_count_ = 0;

    nav_result_ready_ = false;
    nav_succeeded_ = false;

    active_goal_id_ = ++seq_;
    goal_handle_.reset();

    auto &ctx = BtRosContext::instance();
    RCLCPP_INFO(ctx.node->get_logger(), "🚩 [Goal %lu] onStart", active_goal_id_);

    auto clock_type = BtRosContext::instance().node->get_clock()->get_clock_type();
    last_attempt_time_ = rclcpp::Time(0, 0, clock_type);
    last_vis_time_ = rclcpp::Time(0, 0, clock_type);

    if (!getInput("goal", current_goal_)) {
      RCLCPP_ERROR(BtRosContext::instance().node->get_logger(), "❌ 无法获取 goal 输入");
      return BT::NodeStatus::FAILURE;
    }
    if (current_goal_.header.frame_id.empty()) {
      current_goal_.header.frame_id = "map";
    }

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto &ctx = BtRosContext::instance();
    auto now = ctx.node->now();


      if (nav_result_ready_) {
        if (nav_succeeded_) {
          return BT::NodeStatus::SUCCESS;
        } else {
          return BT::NodeStatus::FAILURE;
        }
    }
    if (!ctx.client->action_server_is_ready()) {
      if ((now - last_attempt_time_).seconds() > 2.0) {
        RCLCPP_WARN(ctx.node->get_logger(), "⚠️ Nav2 Action Server 未连接，正在等待...");
        last_attempt_time_ = now;
      }
      return BT::NodeStatus::RUNNING;
    }

    switch (internal_state_)
    {
    // ----------------------------------------------------------------
    // 状态：空闲 或 上次被拒 -> 准备发送
    // ----------------------------------------------------------------
    case InternalState::IDLE:
    case InternalState::REJECTED:
    {
      const uint64_t this_goal_id = active_goal_id_;
      if ((now - last_attempt_time_).seconds() < 0.2) {
          return BT::NodeStatus::RUNNING;
      }

      RCLCPP_INFO(ctx.node->get_logger(),
                  "🚀 [尝试发送] -> (%.2f, %.2f) ...",
                  current_goal_.pose.position.x, current_goal_.pose.position.y);

      // 可视化
      current_goal_.header.stamp = now;
      if (ctx.vis_pub) ctx.vis_pub->publish(current_goal_);

      NavigateToPose::Goal goal_msg;
      goal_msg.pose = current_goal_;

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

      // 回调函数：处理 Nav2 的反馈
      send_goal_options.goal_response_callback =
          [this, logger = ctx.node->get_logger(), goal_id = this_goal_id]
          (const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle)
      {
          if (goal_id != active_goal_id_) {
            RCLCPP_WARN(logger, "⚠️ [Goal %lu] 过期的 goal_response，忽略", goal_id);
            return;
          }

          if (!handle) {
            RCLCPP_ERROR(logger, "❌ [Goal %lu] 被 Nav2 拒绝", goal_id);
            internal_state_ = InternalState::REJECTED;
          } else {
            goal_handle_ = handle;
            RCLCPP_INFO(logger, "✅ [Goal %lu] Nav2 接收成功", goal_id);
            internal_state_ = InternalState::ACCEPTED;
          }
      };

      send_goal_options.result_callback =
          [this, logger = ctx.node->get_logger(), goal_id = this_goal_id]
          (const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
      {
          if (goal_id != active_goal_id_) {
            RCLCPP_WARN(logger,
                        "⚠️ [Goal %lu] 收到迟到结果, 当前活动 goal=%lu, 忽略",
                        goal_id, active_goal_id_);
            return;
          }

          nav_result_ready_ = true;

          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              nav_succeeded_ = true;
              RCLCPP_INFO(logger, "✅ [Goal %lu] 到达成功", goal_id);
              break;

            case rclcpp_action::ResultCode::ABORTED:
              nav_succeeded_ = true;
              RCLCPP_WARN(logger, "⚠️ [Goal %lu] 被中止 -> 跳过当前点", goal_id);
              break;

            case rclcpp_action::ResultCode::CANCELED:
              nav_succeeded_ = true;
              if (goal_id == canceled_by_bt_goal_id_) {
                RCLCPP_INFO(logger, "ℹ️ [Goal %lu] 因 BT 切点取消", goal_id);
              } else {
                RCLCPP_WARN(logger, "⚠️ [Goal %lu] 被外部取消 -> 继续下一个点", goal_id);
              }
              break;

            default:
              nav_succeeded_ = false;
              RCLCPP_WARN(logger, "⚠️ [Goal %lu] 未知结果码", goal_id);
              break;
          }
      };

      // 发送请求
      ctx.client->async_send_goal(goal_msg, send_goal_options);
      
      // 更新状态和时间
      internal_state_ = InternalState::SENDING;
      last_attempt_time_ = now;
      break;
    }

    // ----------------------------------------------------------------
    // 状态：正在发送 (等待回调)
    // ----------------------------------------------------------------
    case InternalState::SENDING:
    {
      // 超时看门狗 (Watchdog)
      // 如果发送后 1.0 秒内没有收到 callback (既没变 ACCEPTED 也没变 REJECTED)
      // 说明消息丢了或者卡住了。直接强制重置回 IDLE，触发下一次 tick 重新发。
      double wait_time = (now - last_attempt_time_).seconds();
      if (wait_time > 1.0) {
          RCLCPP_WARN(ctx.node->get_logger(), "⏰ [超时] 等待 Nav2 响应 %.1fs 无果，强制重发！", wait_time);
          internal_state_ = InternalState::IDLE; 
      }
      break;
    }

    // ----------------------------------------------------------------
    // 状态：已接受
    // ----------------------------------------------------------------
    case InternalState::ACCEPTED:
      if ((now - last_vis_time_).seconds() > 0.5) {
        if (ctx.vis_pub) {
          current_goal_.header.stamp = now;
          ctx.vis_pub->publish(current_goal_);
        }
        last_vis_time_ = now;
      }
      break;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override 
  {
      auto &ctx = BtRosContext::instance();

      if (ctx.client && goal_handle_) {
        canceled_by_bt_goal_id_ = active_goal_id_;
        RCLCPP_WARN(ctx.node->get_logger(), "✂️ [Goal %lu] 取消当前 goal", active_goal_id_);
        ctx.client->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
      }
      internal_state_ = InternalState::IDLE;
      retry_count_ = 0;
  }

private:
  geometry_msgs::msg::PoseStamped current_goal_;
  InternalState internal_state_{InternalState::IDLE};
  rclcpp::Time last_attempt_time_;
  rclcpp::Time last_vis_time_;
  int retry_count_{0};

  bool nav_result_ready_{false};
  bool nav_succeeded_{false};
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle_;
  uint64_t canceled_by_bt_goal_id_{0};
  uint64_t active_goal_id_{0};
  uint64_t seq_{0};
};




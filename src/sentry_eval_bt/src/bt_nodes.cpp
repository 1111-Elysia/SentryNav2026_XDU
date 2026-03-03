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
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")};
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

    auto goal = ctx.points[ctx.current_index];
    goal.header.stamp = ctx.node->now();

    setOutput("goal", goal);
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
        BT::InputPort<double>("threshold", 0.5, "Distance threshold")};
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

    try {
      geometry_msgs::msg::TransformStamped t;
      t = ctx.tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

      double dx = goal.pose.position.x - t.transform.translation.x;
      double dy = goal.pose.position.y - t.transform.translation.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      // --- 关键逻辑：区分普通点和终点 ---
      bool is_last_point = (ctx.current_index >= ctx.points.size());
      double effective_threshold = xml_threshold;
      std::string mode_str = "巡逻模式";

      if (is_last_point) {
        effective_threshold = 0.3; 
        mode_str = "终点锁定";
      }
      
      if (dist < effective_threshold) {
        size_t cur_idx = (ctx.current_index > 0) ? ctx.current_index - 1 : 0;
        
        // 🚨【新增逻辑】如果是最后一个点，执行“混合急刹”
        if (is_last_point) {
            RCLCPP_WARN(ctx.node->get_logger(), "🛑 到达终点 (%.2fm) -> 触发强制急刹！", dist);

            // A. Soft Stop: 告诉 Nav2 别算了，订单取消
            if (ctx.client) {
                ctx.client->async_cancel_all_goals();
            }

            // B. Hard Stop: 向 /cmd_vel 连发 0 速度，按死车轮
            if (ctx.vel_pub) {
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.linear.y = 0.0;
                stop_msg.angular.z = 0.0;
                // 连发3次，确保底层收到
                ctx.vel_pub->publish(stop_msg);
                ctx.vel_pub->publish(stop_msg);
                ctx.vel_pub->publish(stop_msg);
            }
        } 
        else {
            // 普通点：只打印日志，平滑切过
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

    auto clock_type = BtRosContext::instance().node->get_clock()->get_clock_type();
    // 初始化为一个很早的时间，确保第一次立刻触发
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

    // 0. 基础检查：Nav2 动作服务器是否在线
    if (!ctx.client->action_server_is_ready()) {
      // 降低日志频率：每 2 秒报一次错，避免刷屏，但保持 RUNNING 等待
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
          [this, logger = ctx.node->get_logger()](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle)
      {
        if (!handle) {
          RCLCPP_ERROR(logger, "❌ [被拒] 目标被 Nav2 拒绝，准备重发...");
          internal_state_ = InternalState::REJECTED; // 触发下一次循环的重发
        } else {
          RCLCPP_INFO(logger, "✅ [已接受] Nav2 接收成功！");
          internal_state_ = InternalState::ACCEPTED; // 锁定状态，不再重发
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
      
      // 只有在已经发送或者接受状态下，才需要去取消
      // 避免 IDLE 状态下乱发取消指令
      if (internal_state_ == InternalState::SENDING || internal_state_ == InternalState::ACCEPTED) {
          if (ctx.client) {
              RCLCPP_WARN(ctx.node->get_logger(), "✂️ [切点] 距离达标，强制 Cancel 上一个 Nav2 目标！");
              ctx.client->async_cancel_all_goals();
          }
      }
      
      // 重置状态，确保下一次使用该节点（发下一个点）时是清清白白的
      internal_state_ = InternalState::IDLE;
      retry_count_ = 0;
  }

private:
  geometry_msgs::msg::PoseStamped current_goal_;
  InternalState internal_state_{InternalState::IDLE};
  rclcpp::Time last_attempt_time_;
  rclcpp::Time last_vis_time_;
  int retry_count_{0};
};
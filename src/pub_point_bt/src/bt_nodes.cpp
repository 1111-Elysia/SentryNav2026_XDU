// pub_point_bt/src/bt_nodes.cpp

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
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

//-------------------------------------------------------
// 全局上下文：保存 Node、Nav2 client、TF监听、waypoints
//-------------------------------------------------------
struct BtRosContext
{
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client;
  
  // 可视化发布者 (让 RViz 能看到目标)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vis_pub;

  // TF 监听器（用于读取当前坐标）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  std::vector<geometry_msgs::msg::PoseStamped> points;
  size_t current_index{0};

  enum class NavStatus { IDLE, SENDING, RUNNING, SUCCEEDED, FAILED, CANCELED };
  NavStatus nav_status{NavStatus::IDLE};
  std::mutex nav_mutex;

  static BtRosContext& instance()
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
  InitPoints(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("points_param") };
  }

  BT::NodeStatus tick() override
  {
    auto& ctx = BtRosContext::instance();
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
// NextPoint：获取当前目标点并输出
//=======================================================
class NextPoint : public BT::SyncActionNode
{
public:
  NextPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal") };
  }

  BT::NodeStatus tick() override
  {
    auto& ctx = BtRosContext::instance();
    if (ctx.points.empty()) return BT::NodeStatus::FAILURE;

    // 循环逻辑
    if (ctx.current_index >= ctx.points.size()) {
      RCLCPP_INFO(ctx.node->get_logger(), "NextPoint: 所有点已跑完，任务结束！");
      ctx.current_index = 0; 
      
      return BT::NodeStatus::FAILURE;  // 终止树的运行
     }

    auto goal = ctx.points[ctx.current_index];
    goal.header.stamp = ctx.node->now();

    setOutput("goal", goal);
    
    ctx.current_index++; // 准备下一次调用的索引
    return BT::NodeStatus::SUCCESS;
  }
};

//=======================================================
// CheckDistance：检查当前位置与目标点的距离
//=======================================================
class CheckDistance : public BT::StatefulActionNode
{
public:
  CheckDistance(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("threshold", 0.5, "Distance threshold")
    };
  }

  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }

  BT::NodeStatus onRunning() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.tf_buffer) return BT::NodeStatus::FAILURE;

    geometry_msgs::msg::PoseStamped goal;
    double threshold = 0.5;

    if (!getInput("goal", goal)) return BT::NodeStatus::FAILURE;
    getInput("threshold", threshold);

    try {
      geometry_msgs::msg::TransformStamped t;
      t = ctx.tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

      double dx = goal.pose.position.x - t.transform.translation.x;
      double dy = goal.pose.position.y - t.transform.translation.y;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist < threshold) {
        // 日志打印逻辑
        size_t cur_idx = (ctx.current_index > 0) ? ctx.current_index - 1 : 0;
        std::string next_str = "结束/重置";
        if (ctx.current_index < ctx.points.size()) {
            char buf[32]; 
            auto p = ctx.points[ctx.current_index];
            snprintf(buf, 32, "(%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
            next_str = std::string(buf);
        }

        RCLCPP_INFO(ctx.node->get_logger(), 
            "\n>>> [到达] 距离目标 %.2fm (阈值 %.2f) <<<\n"
            "    |-- 当前目标: P%zu\n"
            "    |-- 切换下一站: %s", 
            dist, threshold, cur_idx + 1, next_str.c_str());

        return BT::NodeStatus::SUCCESS; 
      }
      return BT::NodeStatus::RUNNING;

    } catch (const tf2::TransformException & ex) {
      // 这里的 warn 可以稍微降频，防止刷屏
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override {}
};

//=======================================================
// SendNav2Goal：发送目标点 (增强可视化版)
//=======================================================
class SendNav2Goal : public BT::StatefulActionNode
{
public:
  SendNav2Goal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
  }

  // -------------------------------------------------------
  // 1. onStart: 只负责向 Nav2 发送一次核心指令
  // -------------------------------------------------------
  BT::NodeStatus onStart() override
  {
    auto& ctx = BtRosContext::instance();
    
    // 安全检查
    if (!ctx.client->action_server_is_ready()) {
        RCLCPP_ERROR(ctx.node->get_logger(), "❌ SendNav2Goal: Nav2 Server 未连接！");
        return BT::NodeStatus::FAILURE;
    }

    // 获取并缓存目标点 (存入成员变量 current_goal_)
    if (!getInput("goal", current_goal_)) return BT::NodeStatus::FAILURE;

    if (current_goal_.header.frame_id.empty()) {
        current_goal_.header.frame_id = "map"; 
    }
    current_goal_.header.stamp = ctx.node->now();

    // 第一次可视化
    if (ctx.vis_pub) {
        ctx.vis_pub->publish(current_goal_);
    }

    RCLCPP_INFO(ctx.node->get_logger(), 
        "🏁 [发送目标] -> (%.2f, %.2f)", 
        current_goal_.pose.position.x, current_goal_.pose.position.y);

    {
        std::lock_guard<std::mutex> lk(ctx.nav_mutex);
        ctx.nav_status = BtRosContext::NavStatus::SENDING;
    }

    // 发送 Nav2 指令
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = current_goal_;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        [ctx_node = ctx.node](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & handle) {
            if (!handle) RCLCPP_ERROR(ctx_node->get_logger(), "❌ 目标被拒！");
            else RCLCPP_INFO(ctx_node->get_logger(), "✅ Nav2已接收");
        };
    
    // 不需要 result callback，因为会被 CheckDistance 打断
    send_goal_options.result_callback = [](auto){};

    ctx.client->async_send_goal(goal_msg, send_goal_options);
    
    // 记录时间，用于 onRunning 的频率控制
    last_pub_time_ = ctx.node->now();

    return BT::NodeStatus::RUNNING;
  }

  // -------------------------------------------------------
  // 2. onRunning: 持续刷新 RViz 显示 (这就是区别所在！)
  // -------------------------------------------------------
  BT::NodeStatus onRunning() override
  {
    auto& ctx = BtRosContext::instance();
    
    // 获取当前时间
    auto now = ctx.node->now();

    // 每隔 0.5 秒 (500ms) 执行一次
    if ((now - last_pub_time_).seconds() > 0.5) {
        if (ctx.vis_pub) {
            // 更新时间戳并发布，确保 RViz 知道这还是新的消息
            current_goal_.header.stamp = now; 
            ctx.vis_pub->publish(current_goal_);
        }
        last_pub_time_ = now;
    }

    return BT::NodeStatus::RUNNING;
  }

  // -------------------------------------------------------
  // 3. onHalted: 切换点时什么都不做 (保持平滑)
  // -------------------------------------------------------
  void onHalted() override
  {
    // 不取消目标
  }

private:
  // 私有成员变量，用于在 onStart 和 onRunning 之间共享数据
  geometry_msgs::msg::PoseStamped current_goal_; 
  rclcpp::Time last_pub_time_;
};
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
      
      return BT::NodeStatus::FAILURE;  // 终止树的运行   循环跑的话注释掉就行
     }

    auto goal = ctx.points[ctx.current_index];
    goal.header.stamp = ctx.node->now();

    setOutput("goal", goal);
    
    // RCLCPP_INFO(ctx.node->get_logger(), "NextPoint: 准备前往第 %zu 个点 (x=%.2f, y=%.2f)", 
    //             ctx.current_index + 1, goal.pose.position.x, goal.pose.position.y);

    ctx.current_index++; // 准备下一次调用的索引
    return BT::NodeStatus::SUCCESS;
  }
};

//=======================================================
// CheckDistance：检查当前位置与目标点的距离
// 返回 RUNNING 表示"还没到，继续跑"
// 返回 SUCCESS 表示"到了/快到了，切下一个点"
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
      BT::InputPort<double>("threshold", 0.5, "Distance threshold to switch next point")
    };
  }

  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.tf_buffer) return BT::NodeStatus::FAILURE;

    geometry_msgs::msg::PoseStamped goal;
    double threshold = 0.5;

    if (!getInput("goal", goal)) return BT::NodeStatus::FAILURE;
    getInput("threshold", threshold);

    try {
      // 1. 获取机器人当前在 map 下的坐标
      // 注意：这里假设机器人底盘 frame 是 "base_link"，地图是 "map"
      geometry_msgs::msg::TransformStamped t;
      t = ctx.tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

      double current_x = t.transform.translation.x;
      double current_y = t.transform.translation.y;

      // 2. 计算欧氏距离
      double dx = goal.pose.position.x - current_x;
      double dy = goal.pose.position.y - current_y;
      double dist = std::sqrt(dx*dx + dy*dy);

      // 3. 判断是否到达
      if (dist < threshold) {
        size_t cur_idx = 0;
        if (ctx.current_index > 0) cur_idx = ctx.current_index - 1;

        // 获取当前点的坐标字符串
        std::string cur_str = pointToString(ctx.points[cur_idx]);
        
        // 获取下一个点的坐标字符串 (检查越界)
        std::string next_str = "结束/重置";
        if (ctx.current_index < ctx.points.size()) {
            next_str = pointToString(ctx.points[ctx.current_index]);
        } else {
             // 如果是循环模式，下一个可能是第0个
             if (!ctx.points.empty()) next_str = pointToString(ctx.points[0]) + " (循环)";
        }

        RCLCPP_INFO(ctx.node->get_logger(), 
            "\n>>> [到达] 距离目标 %.2fm (阈值 %.2f) <<<\n"
            "    |-- 当前目标: P%zu %s\n"
            "    |-- 切换下一站: %s", 
            dist, threshold, cur_idx + 1, cur_str.c_str(), next_str.c_str());
        // ----------------------

        return BT::NodeStatus::SUCCESS; 
      }

      return BT::NodeStatus::RUNNING;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(ctx.node->get_logger(), "CheckDistance: Could not transform: %s", ex.what());
      return BT::NodeStatus::RUNNING; // 拿不到坐标就继续等
    }
  }

  void onHalted() override {}

private:
  // 辅助函数：把 Pose 转成 "(x, y)" 字符串
  std::string pointToString(const geometry_msgs::msg::PoseStamped& p) {
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "(%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
      return std::string(buffer);
  }
};

//=======================================================
// SendNav2Goal：发送目标点（保持原样，但逻辑配合 Parallel）
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

  BT::NodeStatus onStart() override
  {
    auto& ctx = BtRosContext::instance();
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("goal", goal_pose)) return BT::NodeStatus::FAILURE;

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;
    goal_msg.pose.header.stamp = ctx.node->now();

    {
        std::lock_guard<std::mutex> lk(ctx.nav_mutex);
        ctx.nav_status = BtRosContext::NavStatus::SENDING;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    // 我们不需要 result_callback 来决定何时结束，因为 CheckDistance 会抢断我们
    // 但为了程序健壮性，还是留着回调
    send_goal_options.result_callback = [this](auto) {
        // 实际上这个 callback 只有在完全停车后才会触发，
        // 在我们的 S 型过弯逻辑中，这个可能永远不会触发（因为提前被切断了）
    };

    ctx.client->async_send_goal(goal_msg, send_goal_options);
    
    // 立即返回 RUNNING，让行为树继续运行 CheckDistance
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // 一直保持 RUNNING，直到被 Parallel 节点 Halt（中断）
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    // 【关键】当 CheckDistance 成功时，这个节点会被 Halt。
    // 这里我们**什么都不做**。
    // 不要 cancel_goal！
    // 这样，当我们发下一个点时，Nav2 会自动把路径从旧点平滑过渡到新点。
    // 如果这里 cancel 了，机器人会有一个急刹车动作。
  }
};
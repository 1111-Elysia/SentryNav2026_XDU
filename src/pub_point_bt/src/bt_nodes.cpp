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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

//-------------------------------------------------------
// 全局上下文：保存 Node、Nav2 client、waypoints、interval、当前索引、导航状态
//-------------------------------------------------------
struct BtRosContext
{
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client;

  //发布 /initialpose 初始位置
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub;

  std::vector<geometry_msgs::msg::PoseStamped> points;
  double interval_sec{2.0};
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
// InitPoints：从参数读取 points 与 interval，仅执行一次
//=======================================================
class InitPoints : public BT::SyncActionNode
{
public:
  InitPoints(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("points_param"),
      BT::InputPort<std::string>("interval_param")
    };
  }

  BT::NodeStatus tick() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.node) {
      throw BT::RuntimeError("InitPoints: ROS node not set");
    }

    // 只初始化一次，避免 parameter 已声明错误
    static bool initialized = false;
    if (initialized) {
      return BT::NodeStatus::SUCCESS;
    }

    std::string points_param_name;
    std::string interval_param_name;
    getInput("points_param", points_param_name);
    getInput("interval_param", interval_param_name);

    // points 参数：如果未声明则声明
    if (!ctx.node->has_parameter(points_param_name)) {
      ctx.node->declare_parameter(points_param_name, std::vector<double>{});
    }
    auto flat = ctx.node->get_parameter(points_param_name).as_double_array();

    if (flat.size() % 3 != 0 || flat.empty()) {
      RCLCPP_ERROR(ctx.node->get_logger(),
                   "InitPoints: points 必须是 3*N 且非空");
      return BT::NodeStatus::FAILURE;
    }

    // interval 参数
    if (!ctx.node->has_parameter(interval_param_name)) {
      ctx.node->declare_parameter(interval_param_name, 2.0);
    }
    ctx.interval_sec =
        ctx.node->get_parameter(interval_param_name).as_double();

    // 转换为 PoseStamped
    ctx.points.clear();
    ctx.points.reserve(flat.size() / 3);

    for (size_t i = 0; i < flat.size(); i += 3) {
      double x = flat[i];
      double y = flat[i + 1];
      double yaw_deg = flat[i + 2];
      double yaw = yaw_deg * M_PI / 180.0;

      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation.z = std::sin(yaw / 2.0);
      p.pose.orientation.w = std::cos(yaw / 2.0);

      ctx.points.push_back(p);
    }

    ctx.current_index = 0;

    RCLCPP_INFO(ctx.node->get_logger(),
                "InitPoints: loaded %zu points, interval=%.2f",
                ctx.points.size(), ctx.interval_sec);

    initialized = true;
    return BT::NodeStatus::SUCCESS;
  }
};

//=======================================================
// NextPoint：从 points[current_index] 取一个点，输出到端口 goal，并 index++
//=======================================================
class NextPoint : public BT::SyncActionNode
{
public:
  NextPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus tick() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.node) {
      throw BT::RuntimeError("NextPoint: ROS node not set");
    }

    if (ctx.points.empty()) {
      RCLCPP_ERROR(ctx.node->get_logger(), "NextPoint: points empty");
      return BT::NodeStatus::FAILURE;
    }

    if (ctx.current_index >= ctx.points.size()) {
      ctx.current_index = 0;
    }

    auto goal = ctx.points[ctx.current_index];
    goal.header.stamp = ctx.node->now();

    RCLCPP_INFO(ctx.node->get_logger(),
                "NextPoint: index=%zu, x=%.2f, y=%.2f",
                ctx.current_index,
                goal.pose.position.x,
                goal.pose.position.y);

    setOutput("goal", goal);

    ctx.current_index++;
    if (ctx.current_index >= ctx.points.size()) {
      ctx.current_index = 0;
      RCLCPP_INFO(ctx.node->get_logger(),
                  "NextPoint: wrap to index 0");
    }

    return BT::NodeStatus::SUCCESS;
  }
};

//=======================================================
// SendNav2Goal：发送 Nav2 目标并等待结果（到点检测）
//=======================================================
class SendNav2Goal : public BT::StatefulActionNode
{
public:
  SendNav2Goal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus onStart() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.node) {
      throw BT::RuntimeError("SendNav2Goal: ROS node not set");
    }
    if (!ctx.client) {
      RCLCPP_ERROR(ctx.node->get_logger(),
                   "SendNav2Goal: Nav2 client not initialized");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("goal", goal_pose)) {
      RCLCPP_ERROR(ctx.node->get_logger(),
                   "SendNav2Goal: no goal on blackboard");
      return BT::NodeStatus::FAILURE;
    }

    NavigateToPose::Goal goal_msg;
    goal_pose.header.stamp = ctx.node->now();
    goal_msg.pose = goal_pose;

    {
      std::lock_guard<std::mutex> lk(ctx.nav_mutex);
      ctx.nav_status = BtRosContext::NavStatus::SENDING;
    }

    auto node = ctx.node;  // lambda 里使用

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

    // goal 是否被接受
    options.goal_response_callback =
      [node](std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> handle)
      {
        auto& c = BtRosContext::instance();
        std::lock_guard<std::mutex> lk(c.nav_mutex);
        if (!handle) {
          RCLCPP_ERROR(node->get_logger(),
                       "SendNav2Goal: goal rejected");
          c.nav_status = BtRosContext::NavStatus::FAILED;
        } else {
          RCLCPP_INFO(node->get_logger(),
                      "SendNav2Goal: goal accepted");
          c.nav_status = BtRosContext::NavStatus::RUNNING;
        }
      };

    // 反馈（可以用来打印距离等，这里先忽略）
    options.feedback_callback =
      [node](std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>,
             const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        (void)feedback;
        // RCLCPP_DEBUG(node->get_logger(), "feedback ...");
      };

    // 结果：成功 / 失败 / 取消
    options.result_callback =
      [node](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
      {
        auto& c = BtRosContext::instance();
        std::lock_guard<std::mutex> lk(c.nav_mutex);

        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(),
                        "SendNav2Goal: goal reached successfully");
            c.nav_status = BtRosContext::NavStatus::SUCCEEDED;
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(),
                         "SendNav2Goal: goal aborted");
            c.nav_status = BtRosContext::NavStatus::FAILED;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node->get_logger(),
                        "SendNav2Goal: goal canceled");
            c.nav_status = BtRosContext::NavStatus::CANCELED;
            break;
          default:
            RCLCPP_ERROR(node->get_logger(),
                         "SendNav2Goal: unknown result code");
            c.nav_status = BtRosContext::NavStatus::FAILED;
            break;
        }
      };

    ctx.client->async_send_goal(goal_msg, options);

    // 开始等待结果
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto& ctx = BtRosContext::instance();
    std::lock_guard<std::mutex> lk(ctx.nav_mutex);

    switch (ctx.nav_status)
    {
      case BtRosContext::NavStatus::SENDING:
      case BtRosContext::NavStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BtRosContext::NavStatus::SUCCEEDED:
        ctx.nav_status = BtRosContext::NavStatus::IDLE;
        return BT::NodeStatus::SUCCESS;

      case BtRosContext::NavStatus::FAILED:
      case BtRosContext::NavStatus::CANCELED:
        ctx.nav_status = BtRosContext::NavStatus::IDLE;
        return BT::NodeStatus::FAILURE;

      case BtRosContext::NavStatus::IDLE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  void onHalted() override
  {
    // 有需要的话可以在这里调用 cancel_goal，这里先留空
  }
};

//=======================================================
// PublishInitialPose：向 /initialpose 发布一次初始位姿
//=======================================================
class PublishInitialPose : public BT::SyncActionNode
{
public:
  PublishInitialPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  // 输入端口：x, y, yaw_deg（角度）
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("yaw_deg")
    };
  }

  BT::NodeStatus tick() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.node) {
      throw BT::RuntimeError("PublishInitialPose: ROS node not set");
    }
    if (!ctx.initialpose_pub) {
      RCLCPP_ERROR(ctx.node->get_logger(),
                   "PublishInitialPose: initialpose publisher not initialized");
      return BT::NodeStatus::FAILURE;
    }

    double x = 0.0, y = 0.0, yaw_deg = 0.0;
    // 从端口读取（读不到就用默认 0）
    getInput("x", x);
    getInput("y", y);
    getInput("yaw_deg", yaw_deg);

    double yaw = yaw_deg * M_PI / 180.0;

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = ctx.node->now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;

    // 只绕 Z 轴的四元数
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

    // 协方差：36 个元素
    // 这里给一个比较常见的 AMCL 初始不确定性：
    // x, y 方差 0.25 (0.5m)，yaw 方差 ~ (15°)^2
    msg.pose.covariance.fill(0.0);
    msg.pose.covariance[0]  = 0.25; // x
    msg.pose.covariance[7]  = 0.25; // y
    double yaw_var = (15.0 * M_PI / 180.0);
    yaw_var *= yaw_var;
    msg.pose.covariance[35] = yaw_var; // yaw

    ctx.initialpose_pub->publish(msg);

    RCLCPP_INFO(ctx.node->get_logger(),
                "PublishInitialPose: x=%.2f, y=%.2f, yaw=%.1f deg",
                x, y, yaw_deg);

    return BT::NodeStatus::SUCCESS;
  }
};

//=======================================================
// ColorToGoal：根据颜色选择不同的目标位姿（只支持 red / blue）
//=======================================================
class ColorToGoal : public BT::SyncActionNode
{
public:
  ColorToGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  // 输入：color (std::string)
  // 输出：goal (PoseStamped)
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("color"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus tick() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.node) {
      throw BT::RuntimeError("ColorToGoal: ROS node not set");
    }

    // 1) 从输入端口拿颜色字符串
    std::string color;
    if (!getInput("color", color))
    {
      RCLCPP_ERROR(ctx.node->get_logger(),
                   "ColorToGoal: input port [color] not set");
      return BT::NodeStatus::FAILURE;
    }

    // 2) 根据颜色选择 (x, y, yaw_deg)
    double x = 0.0;
    double y = 0.0;
    double yaw_deg = 0.0;

    if (color == "red")
    {
      // 举例：红色目标点
      x = 1.0;
      y = 0.0;
      yaw_deg = 0.0;      // 朝 +X
    }
    else if (color == "blue")
    {
      // 举例：蓝色目标点
      x = 0.0;
      y = 1.0;
      yaw_deg = 90.0;     // 朝 +Y
    }
    else
    {
      RCLCPP_ERROR(ctx.node->get_logger(),
                   "ColorToGoal: unknown color [%s]", color.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // 3) 欧拉角（yaw，单位：度） → 弧度 → 四元数
    double yaw = yaw_deg * M_PI / 180.0;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = ctx.node->now();
    goal.header.frame_id = "map";

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    // 只绕 Z 轴旋转的四元数
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.orientation.w = std::cos(yaw / 2.0);

    // 4) 写到输出端口 goal
    setOutput("goal", goal);

    RCLCPP_INFO(ctx.node->get_logger(),
                "ColorToGoal: color=%s -> (x=%.2f, y=%.2f, yaw=%.1f deg)",
                color.c_str(), x, y, yaw_deg);

    return BT::NodeStatus::SUCCESS;
  }
};


//=======================================================
// WaitSeconds：按 interval_sec 等待
//=======================================================
class WaitSeconds : public BT::StatefulActionNode
{
public:
  WaitSeconds(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};  // 使用全局 ctx.interval_sec，不用端口
  }

  BT::NodeStatus onStart() override
  {
    auto& ctx = BtRosContext::instance();
    if (!ctx.node) {
      throw BT::RuntimeError("WaitSeconds: ROS node not set");
    }

    double sec = ctx.interval_sec;
    if (sec <= 0.0) sec = 0.1;

    wait_duration_ = std::chrono::duration<double>(sec);
    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(ctx.node->get_logger(),
                "WaitSeconds: wait %.2f seconds", sec);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto now = std::chrono::steady_clock::now();
    if (now - start_time_ >= wait_duration_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double> wait_duration_{0.0};
};

// 不要 BT_REGISTER_NODES，这里只定义类，runner 里手动注册

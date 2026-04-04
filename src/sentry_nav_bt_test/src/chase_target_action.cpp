#include "sentry_nav_bt_test/chase_target_action.hpp"

#include <cmath>
#include <algorithm>

namespace sentry_nav_bt_test
{

static inline double hypot2d(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

ChaseTargetAction::ChaseTargetAction(const std::string &name,
                                     const BT::NodeConfiguration &config,
                                     const rclcpp::Node::SharedPtr &node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  if (!node_) {
    throw BT::RuntimeError("ChaseTargetAction: node is null");
  }
  setStatus_(ChaseStatus::IDLE);
}

BT::NodeStatus ChaseTargetAction::onStart()
{
  // 读一次 ports（也可以每 tick 读，但多数参数不频繁变化）
  (void)getInput("aim_topic", aim_topic_);        // 默认：/autoaim/target_bl
  (void)getInput("world_frame", world_frame_);    // 默认：map
  (void)getInput("base_frame", base_frame_);      // 默认：base_link
  (void)getInput("nav_action", nav_action_);      // 默认：navigate_to_pose

  (void)getInput("standoff", standoff_);
  (void)getInput("stop_dist", stop_dist_);
  (void)getInput("start_dist", start_dist_);
  (void)getInput("lost_timeout", lost_timeout_);
  (void)getInput("update_thresh", update_thresh_);
  (void)getInput("max_rate", max_rate_);
  (void)getInput("ema_alpha", ema_alpha_);
  (void)getInput("max_jump", max_jump_);

  if (start_dist_ < stop_dist_) {
    RCLCPP_WARN(node_->get_logger(),
                "[ChaseTarget] start_dist(%.2f) < stop_dist(%.2f)，建议 start_dist > stop_dist 以形成滞回",
                start_dist_, stop_dist_);
  }

  ensureInitialized_();

  // 重置状态（不清订阅/TF/client，只清运行缓存）
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_recv_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    last_meas_raw_.reset();
    meas_filt_.reset();
    last_goal_send_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    last_goal_world_.reset();
  }

  abort_count_.store(0);
  setStatus_(ChaseStatus::CHASING);
  setOutput("chase_status", static_cast<int>(status_.load()));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ChaseTargetAction::onRunning()
{
  // 为了允许后期在 XML 动态调参，这里每 tick 也再读一遍
  (void)getInput("standoff", standoff_);
  (void)getInput("stop_dist", stop_dist_);
  (void)getInput("start_dist", start_dist_);
  (void)getInput("lost_timeout", lost_timeout_);
  (void)getInput("update_thresh", update_thresh_);
  (void)getInput("max_rate", max_rate_);
  (void)getInput("ema_alpha", ema_alpha_);
  (void)getInput("max_jump", max_jump_);

  std::string aim_topic_new = aim_topic_;
  std::string world_frame_new = world_frame_;
  std::string base_frame_new = base_frame_;
  std::string nav_action_new = nav_action_;
  (void)getInput("aim_topic", aim_topic_new);
  (void)getInput("world_frame", world_frame_new);
  (void)getInput("base_frame", base_frame_new);
  (void)getInput("nav_action", nav_action_new);

  if (aim_topic_new != aim_topic_) {
    ensureSubscriptions_(aim_topic_new);
    RCLCPP_INFO(node_->get_logger(), "[ChaseTarget] aim_topic changed -> %s", aim_topic_new.c_str());
  }
  if (world_frame_new != world_frame_) {
    world_frame_ = world_frame_new;
  }
  if (base_frame_new != base_frame_) {
    base_frame_ = base_frame_new;
  }
  if (nav_action_new != nav_action_) {
    ensureNavClient_(nav_action_new);
    RCLCPP_INFO(node_->get_logger(), "[ChaseTarget] nav_action changed -> %s", nav_action_new.c_str());
  }

  ensureInitialized_();

  if (abort_count_.load() >= 3)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "[ChaseTarget] Nav2 aborted %d times, exit chase",
      abort_count_.load());

    cancelAllGoals_();
    setStatus_(ChaseStatus::ERROR);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::FAILURE;
  }

  const auto now = node_->now();

  // 1) 获取滤波后的测量点（base_link 下）
  std::optional<geometry_msgs::msg::Point> meas;
  rclcpp::Time last_recv(0, 0, node_->get_clock()->get_clock_type());
  std::optional<geometry_msgs::msg::PoseStamped> last_goal;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    meas = meas_filt_;
    last_recv = last_recv_time_;
    last_goal = last_goal_world_;
  }

  // 2) 丢失目标超时：cancel & FAILURE（让 BT 切换策略）
  if (last_recv.nanoseconds() == 0)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[ChaseTarget] no aim point received yet, waiting...");

    setStatus_(ChaseStatus::IDLE);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::RUNNING;
  }

  if ((now - last_recv).seconds() > lost_timeout_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[ChaseTarget] target lost: last aim point age=%.3f s > lost_timeout=%.3f s",
      (now - last_recv).seconds(), lost_timeout_);

    cancelAllGoals_();
    setStatus_(ChaseStatus::LOST);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::FAILURE;
  }

  if (!meas)
  {
    // 已收到过时间戳但还没滤波结果（很少见），继续等
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::RUNNING;
  }

  const double x = meas->x;
  const double y = meas->y;
  const double d = hypot2d(x, y);
  const auto current_status = static_cast<ChaseStatus>(status_.load());
  const double resume_dist = std::max(start_dist_, stop_dist_);

  // 3) 近距离进入 HOLD：停止继续发导航点，但保持 RUNNING 以便持续监视目标。
  //    当目标重新离开到 start_dist 外时，才恢复追击，形成滞回。
  if (d <= stop_dist_)
  {
    if (current_status != ChaseStatus::HOLD) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[ChaseTarget] enter HOLD: target distance=%.3f <= stop_dist=%.3f",
        d, stop_dist_);
      cancelAllGoals_();
    }
    setStatus_(ChaseStatus::HOLD);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::RUNNING;
  }

  if (current_status == ChaseStatus::HOLD && d < resume_dist)
  {
    setStatus_(ChaseStatus::HOLD);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::RUNNING;
  }

  if (current_status == ChaseStatus::HOLD)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ChaseTarget] leave HOLD: target distance=%.3f >= resume_dist=%.3f, resume chase",
      d, resume_dist);
  }

  // 4) 生成 base_link 下站位目标点（不直接去装甲板点）
  geometry_msgs::msg::PoseStamped goal_bl = makeStandOffGoalBL_(*meas, now, standoff_);

  // 如果 standoff 过大导致 goal 很接近原点，就不再发点（进入 HOLD）
  const double gd = hypot2d(goal_bl.pose.position.x, goal_bl.pose.position.y);
  if (gd < 0.05)
  {
    if (current_status != ChaseStatus::HOLD) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[ChaseTarget] enter HOLD: stand-off goal too close, gd=%.3f",
        gd);
      cancelAllGoals_();
    }
    setStatus_(ChaseStatus::HOLD);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::RUNNING;
  }

  // 5) TF：base_link -> world(map/odom)
  geometry_msgs::msg::PoseStamped goal_world;
  try
  {
    // transform timeout 50ms
    goal_world = tf_buffer_->transform(goal_bl, world_frame_, tf2::durationFromSec(0.05));
  }
  catch (const tf2::TransformException &ex)
  {
    // TF 暂时不可用：保持 RUNNING 等下一帧
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[ChaseTarget] TF transform failed: %s", ex.what());
    setStatus_(ChaseStatus::ERROR);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::RUNNING;
  }

  // 6) 目标更新阈值 + 频率限制
  if (!shouldUpdateGoal_(goal_world, now, update_thresh_, max_rate_))
  {
    setStatus_(ChaseStatus::CHASING);
    setOutput("chase_status", static_cast<int>(status_.load()));
    return BT::NodeStatus::RUNNING;
  }

  // 7) 发送 Nav2 goal
  sendNavGoal_(goal_world);

  // 8) 更新缓存/输出
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_goal_world_ = goal_world;
    last_goal_send_time_ = now;
  }

  setOutput("last_goal", goal_world);
  setStatus_(ChaseStatus::CHASING);
  setOutput("chase_status", static_cast<int>(status_.load()));
  return BT::NodeStatus::RUNNING;
}

void ChaseTargetAction::onHalted()
{
  // BT 切换分支/停止追击时调用
  cancelAllGoals_();
  setStatus_(ChaseStatus::IDLE);
  setOutput("chase_status", static_cast<int>(status_.load()));
}

// -------------------- helpers --------------------

void ChaseTargetAction::ensureInitialized_()
{
  if (initialized_) return;

  ensureTf_();
  ensureNavClient_(nav_action_);

  // 默认订阅话题（可在 XML 里通过 aim_topic 改）
  if (aim_topic_.empty()) {
    aim_topic_ = "/autoaim/target_bl"; // NOTE: 默认话题名，后期改 XML 即可
  }
  ensureSubscriptions_(aim_topic_);
  initialized_ = true;
}

void ChaseTargetAction::ensureTf_()
{
  if (tf_buffer_ && tf_listener_) return;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ChaseTargetAction::ensureNavClient_(const std::string &action_name)
{
  if (nav_client_ && nav_action_ == action_name) return;

  if (action_name.empty()) {
    nav_action_ = "navigate_to_pose";
  } else {
    nav_action_ = action_name;
  }

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, nav_action_);
}

void ChaseTargetAction::ensureSubscriptions_(const std::string &topic)
{
  if (!sub_aim_ || aim_topic_ != topic)
  {
    aim_topic_ = topic;

    // 使用 best_effort 更适合高频自瞄点
    auto qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    qos.best_effort();
    qos.durability_volatile();

    sub_aim_ = node_->create_subscription<geometry_msgs::msg::Point>(
      aim_topic_, qos,
      std::bind(&ChaseTargetAction::onAimPoint_, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "[ChaseTarget] subscribed aim_topic: %s", aim_topic_.c_str());
  }
}

void ChaseTargetAction::onAimPoint_(const geometry_msgs::msg::Point::SharedPtr msg)
{
  const auto now = node_->now();
  std::lock_guard<std::mutex> lk(mtx_);

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    500,
    "[ChaseTarget] recv aim point: x=%.3f y=%.3f z=%.3f",
    msg->x, msg->y, msg->z);

  last_recv_time_ = now;

  // 跳变剔除
  if (last_meas_raw_)
  {
    const double dx = msg->x - last_meas_raw_->x;
    const double dy = msg->y - last_meas_raw_->y;
    const double jump = hypot2d(dx, dy);
    if (jump > max_jump_)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[ChaseTarget] drop aim point: jump=%.3f > max_jump=%.3f",
        jump, max_jump_);
      // 丢弃本次测量
      return;
    }
  }
  last_meas_raw_ = *msg;

  // EMA 滤波
  if (!meas_filt_)
  {
    meas_filt_ = *msg;
  }
  else
  {
    meas_filt_->x = ema_alpha_ * msg->x + (1.0 - ema_alpha_) * meas_filt_->x;
    meas_filt_->y = ema_alpha_ * msg->y + (1.0 - ema_alpha_) * meas_filt_->y;
    meas_filt_->z = 0.0;
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    500,
    "[ChaseTarget] filt aim point: x=%.3f y=%.3f z=%.3f",
    meas_filt_->x, meas_filt_->y, meas_filt_->z);
}

geometry_msgs::msg::PoseStamped
ChaseTargetAction::makeStandOffGoalBL_(const geometry_msgs::msg::Point &p_bl,
                                       const rclcpp::Time &stamp,
                                       double standoff) const
{
  geometry_msgs::msg::PoseStamped goal_bl;
  goal_bl.header.stamp = stamp;
  goal_bl.header.frame_id = base_frame_;
  goal_bl.pose.orientation.w = 1.0;

  const double d = hypot2d(p_bl.x, p_bl.y);
  if (d < 1e-6)
  {
    goal_bl.pose.position.x = 0.0;
    goal_bl.pose.position.y = 0.0;
    return goal_bl;
  }

  // 站位半径：d_goal = d - standoff
  const double d_goal = std::max(0.0, d - standoff);

  // 沿方向缩放
  const double scale = d_goal / d;
  goal_bl.pose.position.x = p_bl.x * scale;
  goal_bl.pose.position.y = p_bl.y * scale;
  goal_bl.pose.position.z = 0.0;
  return goal_bl;
}

bool ChaseTargetAction::shouldUpdateGoal_(const geometry_msgs::msg::PoseStamped &goal_world,
                                         const rclcpp::Time &now,
                                         double update_thresh,
                                         double max_rate_hz) const
{
  // 频率限制
  if (max_rate_hz > 0.0)
  {
    const double min_period = 1.0 / max_rate_hz;

    rclcpp::Time last_send(0, 0, node_->get_clock()->get_clock_type());
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_send = last_goal_send_time_;
    }

    if (last_send.nanoseconds() != 0 && (now - last_send).seconds() < min_period)
    {
      return false;
    }
  }

  // 更新阈值（world frame）
  std::optional<geometry_msgs::msg::PoseStamped> last_goal;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_goal = last_goal_world_;
  }

  if (last_goal)
  {
    const double dx = goal_world.pose.position.x - last_goal->pose.position.x;
    const double dy = goal_world.pose.position.y - last_goal->pose.position.y;
    const double diff = hypot2d(dx, dy);
    if (diff < update_thresh)
    {
      return false;
    }
  }

  return true;
}

void ChaseTargetAction::sendNavGoal_(const geometry_msgs::msg::PoseStamped &goal_world)
{
  if (!nav_client_) {
    setStatus_(ChaseStatus::ERROR);
    return;
  }

  if (!nav_client_->action_server_is_ready())
  {
    // 非阻塞等待：避免卡住 BT tick
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[ChaseTarget] Nav2 action server not ready: %s", nav_action_.c_str());
    setStatus_(ChaseStatus::ERROR);
    return;
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    500,
    "[ChaseTarget] send nav goal: %s x=%.3f y=%.3f",
    goal_world.header.frame_id.c_str(),
    goal_world.pose.position.x,
    goal_world.pose.position.y);

  
  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_world;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // 回调中尽量只做轻量状态更新（并发安全）
  options.goal_response_callback =
    [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr handle)
    {
      if (!handle)
      {
        RCLCPP_ERROR(node_->get_logger(), "[ChaseTarget] Nav2 rejected goal");
        setStatus_(ChaseStatus::ERROR);
      }
      else
      {
        abort_count_.store(0);
      }
      };

  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
    {
      if (result.code == rclcpp_action::ResultCode::ABORTED)
      {
        int cnt = abort_count_.fetch_add(1) + 1;
        RCLCPP_WARN(
          node_->get_logger(),
          "[ChaseTarget] Nav2 goal aborted (%d)",
          cnt);
        setStatus_(ChaseStatus::ERROR);
      }
    };

  nav_client_->async_send_goal(goal_msg, options);
}

void ChaseTargetAction::cancelAllGoals_()
{
  if (!nav_client_) return;

  if (nav_client_->action_server_is_ready())
  {
    nav_client_->async_cancel_all_goals();
  }

  std::lock_guard<std::mutex> lk(mtx_);
  last_goal_world_.reset();
  last_goal_send_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

void ChaseTargetAction::setStatus_(ChaseStatus s)
{
  status_.store(static_cast<uint8_t>(s));
}

}  // namespace sentry_nav_bt_test

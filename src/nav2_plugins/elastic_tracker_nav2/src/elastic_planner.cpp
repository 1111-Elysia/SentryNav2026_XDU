/*
    Elastic-Tracker 2D Nav2 GlobalPlanner.
    A*-based spring tracking with internal 10Hz replanning timer.
*/
#include "elastic_planner/elastic_planner.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include <chrono>
#include <cmath>
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace elastic_planner {

// ==================== Lifecycle ====================

void ElasticPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  planner_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  nav2_util::declare_parameter_if_not_declared(node, name + ".tracking_dist", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".tracking_dur", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".tracking_dt", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, name + ".use_tracking", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name + ".target_timeout", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ekf_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ekf_alpha", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ekf_beta", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ekf_reset_dt", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".target_topic", rclcpp::ParameterValue("/detected_target_pose"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name + ".max_planning_time", rclcpp::ParameterValue(4.5));
  nav2_util::declare_parameter_if_not_declared(node, name + ".cost_weight", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, name + ".minco_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name + ".minco_sample_dt", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name + ".minco_nominal_speed", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".minco_min_piece_duration", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, name + ".corridor_width", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lbfgs_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lbfgs_max_iterations", rclcpp::ParameterValue(50));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lbfgs_g_epsilon", rclcpp::ParameterValue(1e-4));
  nav2_util::declare_parameter_if_not_declared(node, name + ".finite_diff_eps", rclcpp::ParameterValue(1e-4));
  nav2_util::declare_parameter_if_not_declared(node, name + ".corridor_weight", rclcpp::ParameterValue(50.0));

  node->get_parameter(name + ".tracking_dist", tracking_dist_);
  node->get_parameter(name + ".tracking_dur", tracking_dur_);
  node->get_parameter(name + ".tracking_dt", tracking_dt_);
  node->get_parameter(name + ".use_tracking", use_tracking_);
  node->get_parameter(name + ".target_timeout", target_timeout_);
  node->get_parameter(name + ".ekf_enabled", ekf_enabled_);
  node->get_parameter(name + ".ekf_alpha", ekf_alpha_);
  node->get_parameter(name + ".ekf_beta", ekf_beta_);
  node->get_parameter(name + ".ekf_reset_dt", ekf_reset_dt_);
  std::string target_topic;
  node->get_parameter(name + ".target_topic", target_topic);
  smoother_params_.get(node, name);
  min_turning_radius_ = 0.05;
  node->get_parameter(name + ".minimum_turning_radius", min_turning_radius_);
  node->get_parameter(name + ".max_planning_time", max_planning_time_);
  env_ = std::make_unique<env_2d::Env2D>();
  prediction_ = std::make_unique<prediction_2d::Predict2D>();
  prediction_->setParams(tracking_dt_, tracking_dur_, 1.0, 4.0);

  double cost_weight;
  node->get_parameter(name + ".cost_weight", cost_weight);
  env_->setCostWeight(cost_weight);

  node->get_parameter(name + ".minco_enabled", minco_enabled_);
  node->get_parameter(name + ".minco_sample_dt", minco_config_.sample_dt);
  node->get_parameter(name + ".minco_nominal_speed", minco_config_.nominal_speed);
  node->get_parameter(name + ".minco_min_piece_duration",minco_config_.min_piece_duration);
  node->get_parameter(name + ".corridor_width", corridor_width_);
  node->get_parameter(name + ".lbfgs_enabled", minco_config_.lbfgs_enabled);
  node->get_parameter(name + ".lbfgs_max_iterations", minco_config_.lbfgs_max_iterations);
  node->get_parameter(name + ".lbfgs_g_epsilon", minco_config_.lbfgs_g_epsilon);
  node->get_parameter(name + ".finite_diff_eps", minco_config_.finite_diff_eps);
  node->get_parameter(name + ".corridor_weight", minco_config_.corridor_weight);

  smoother_ = std::make_unique<nav2_smac_planner::Smoother>(smoother_params_);
  smoother_->initialize(min_turning_radius_);
  minco_optimizer_ = std::make_unique<elastic_tracker::MincoOptimizer>(minco_config_);

  RCLCPP_INFO(logger_, "[%s] Configured: tracking_dist=%.1fm dur=%.1fs smtr=(w_smooth=%.1f w_data=%.1f)",
              planner_name_.c_str(), tracking_dist_, tracking_dur_,
              smoother_params_.w_smooth_, smoother_params_.w_data_);
}

void ElasticPlanner::activate() {
  auto node = node_.lock();
  if (!node) return;
  std::string target_topic;
  node->get_parameter(planner_name_ + ".target_topic", target_topic);
  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&ElasticPlanner::targetPoseCallback, this, std::placeholders::_1));
  if (use_tracking_) {
    plan_timer_ = rclcpp::create_timer(
      node, node->get_clock(), rclcpp::Duration::from_seconds(1.0 / replan_hz_),
      std::bind(&ElasticPlanner::planTimerCallback, this));
    RCLCPP_INFO(logger_, "[%s] Timer %.0f Hz, sub %s", planner_name_.c_str(), replan_hz_, target_topic.c_str());
  }
}

void ElasticPlanner::deactivate() {
  plan_timer_.reset();
  target_sub_.reset();
  start_received_ = false;
}

void ElasticPlanner::cleanup() {
  env_.reset();
  prediction_.reset();
}

// ==================== Target Pose Callback ====================

void ElasticPlanner::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (ekf_enabled_) updateEKF(*msg);
  else { latest_target_ = *msg; target_vel_.setZero(); }
  last_target_time_ = rclcpp::Clock().now();
  target_received_ = true;
}

void ElasticPlanner::updateEKF(const geometry_msgs::msg::PoseStamped &msg) {
  rclcpp::Time now = msg.header.stamp;
  if (last_ekf_time_.nanoseconds() == 0) {
    last_ekf_time_ = now;
    ekf_pos_ = Eigen::Vector2d(msg.pose.position.x, msg.pose.position.y);
    ekf_vel_.setZero();
    return;
  }
  double dt = (now - last_ekf_time_).seconds();
  if (dt <= 0.0 || dt > ekf_reset_dt_) {
    // Large gap → reset state to avoid stale prediction
    last_ekf_time_ = now;
    ekf_pos_ = Eigen::Vector2d(msg.pose.position.x, msg.pose.position.y);
    ekf_vel_.setZero();
    return;
  }
  // Clamp dt to avoid divergence from irregular message intervals
  if (dt > 0.5) dt = 0.5;

  Eigen::Vector2d z(msg.pose.position.x, msg.pose.position.y);
  Eigen::Vector2d pred = ekf_pos_ + ekf_vel_ * dt;
  Eigen::Vector2d inn = z - pred;
  ekf_pos_ = pred + ekf_alpha_ * inn;
  ekf_vel_ = ekf_vel_ + ekf_beta_ * inn / dt;
  last_ekf_time_ = now;
  latest_target_ = msg;
  latest_target_.pose.position.x = ekf_pos_.x();
  latest_target_.pose.position.y = ekf_pos_.y();
  target_vel_ = ekf_vel_;
}

// ==================== createPlan ====================

nav_msgs::msg::Path ElasticPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal)
{
  last_start_ = start;
  start_received_ = true;

  if (use_tracking_ && target_received_) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!cached_path_.poses.empty()) return cached_path_;
    // No cached plan yet
    nav_msgs::msg::Path hold;
    hold.header.frame_id = start.header.frame_id;
    hold.header.stamp = rclcpp::Clock().now();
    geometry_msgs::msg::PoseStamped p;
    p.header = hold.header;
    p.pose = start.pose;
    hold.poses.push_back(p);
    return hold;
  }

  // Static mode
  auto *costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = start.header.frame_id;
    return empty;
  }
  env_->setCostmap(costmap);
  Eigen::Vector2d sp(start.pose.position.x, start.pose.position.y);
  Eigen::Vector2d gp(goal.pose.position.x, goal.pose.position.y);
  nav_msgs::msg::Path path;
  planStatic(sp, gp, path);
  return path;
}

// ==================== Internal Replanning Timer ====================

void ElasticPlanner::planTimerCallback() {
  if (!use_tracking_ || !target_received_ || !start_received_) return;

  auto clock = rclcpp::Clock();
  // Target timeout → hold position
  if ((clock.now() - last_target_time_).seconds() > target_timeout_) {
    nav_msgs::msg::Path hold;
    hold.header.frame_id = last_start_.header.frame_id;
    hold.header.stamp = clock.now();
    geometry_msgs::msg::PoseStamped p;
    p.header = hold.header;
    p.pose = last_start_.pose;
    hold.poses.push_back(p);
    { std::lock_guard<std::mutex> lk(path_mutex_); cached_path_ = hold; }
    last_plan_time_ = clock.now();
    return;
  }

  // Rate limiting
  auto now = clock.now();
  if (last_plan_time_.nanoseconds() > 0 &&
      (now - last_plan_time_).seconds() < 1.0 / replan_hz_ * 0.9) return;

  auto *costmap = costmap_ros_->getCostmap();
  if (!costmap || costmap->getSizeInCellsX() == 0 || costmap->getSizeInCellsY() == 0) return;
  env_->setCostmap(costmap);
  prediction_->setCostmap(costmap);

  // Robot pose via TF (50ms timeout)
  Eigen::Vector2d start_pos;
  try {
    auto t = tf_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));
    start_pos = {t.transform.translation.x, t.transform.translation.y};
  } catch (const tf2::TransformException &) {
    start_pos = {last_start_.pose.position.x, last_start_.pose.position.y};
  }

  Eigen::Vector2d target_pos;
  {
    std::lock_guard<std::mutex> lk(target_mutex_);
    target_pos = {latest_target_.pose.position.x, latest_target_.pose.position.y};
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = last_start_.header.frame_id;
  path.header.stamp = now;

  auto t0 = std::chrono::steady_clock::now();
  bool ok = planTracked(start_pos, target_pos, path);
  auto t_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  RCLCPP_INFO(logger_, "[%s] PLAN %.1fms dist=%.2fm (%.1f,%.1f)→(%.1f,%.1f) %zu %s",
    planner_name_.c_str(), t_ms, (target_pos - start_pos).norm(),
    start_pos.x(), start_pos.y(), target_pos.x(), target_pos.y(),
    ok ? path.poses.size() : 0, ok ? "OK" : "FAIL");

  { std::lock_guard<std::mutex> lk(path_mutex_); if (ok) cached_path_ = path; }
  last_plan_time_ = now;
}

// ==================== Path Smoothing ====================

void ElasticPlanner::smoothPath(
    nav_msgs::msg::Path &path,
    const nav2_costmap_2d::Costmap2D *costmap)
{
  if (!costmap || !smoother_ || path.poses.size() < 2) return;

  // Theta* 可能只输出 2 个点，先插值保证 Smoother 有素材
  if (path.poses.size() < 3) {
    nav_msgs::msg::Path dense;
    dense.header = path.header;
    const auto &p0 = path.poses[0].pose.position;
    const auto &p1 = path.poses[1].pose.position;
    double dx = p1.x - p0.x, dy = p1.y - p0.y;
    double len = std::sqrt(dx * dx + dy * dy);
    int n = std::max(3, static_cast<int>(len / 0.1));
    for (int i = 0; i <= n; i++) {
      double t = static_cast<double>(i) / n;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p0.x + dx * t;
      pose.pose.position.y = p0.y + dy * t;
      pose.pose.orientation.w = 1.0;
      dense.poses.push_back(pose);
    }
    path = dense;
  }

  smoother_->smooth(path, costmap, max_planning_time_);
}

// ==================== Planning ====================

bool ElasticPlanner::planTracked(const Eigen::Vector2d &start_pos,
                                 const Eigen::Vector2d &goal_pos,
                                 nav_msgs::msg::Path &path)
{
  // 1. Predict target trajectory
  std::vector<Eigen::Vector2d> predicted;
  {
    std::lock_guard<std::mutex> lk(target_mutex_);
    if (!prediction_->predict(goal_pos, target_vel_, predicted))
      predicted.push_back(goal_pos);
  }

  // 2. Spring goal: stay tracking_dist_ behind target
  Eigen::Vector2d dir = start_pos - predicted.back();
  double d = dir.norm();
  if (d < 0.05) d = 0.05;
  Eigen::Vector2d track_goal = predicted.back() + dir / d * tracking_dist_;

  // 3. Deadband: hold if at equilibrium
  if ((track_goal - start_pos).norm() < 0.15 && target_vel_.norm() < 0.2) {
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = start_pos.x();
    p.pose.position.y = start_pos.y();
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
    return true;
  }

  // 4. A* (50ms timeout, straight-line fallback)
  std::vector<Eigen::Vector2d> raw_path;
  if (!env_->astar_search(start_pos, track_goal, raw_path) || raw_path.size() < 2) {
    raw_path = {start_pos, track_goal};
    Eigen::Vector2d mid = (start_pos + track_goal) * 0.5;
    if ((mid - start_pos).norm() > 0.1) raw_path.insert(raw_path.begin() + 1, mid);
  }

  // minco optimization
  if (minco_enabled_ && minco_optimizer_){
    std::vector<elastic_tracker::Corridor> corridors;
    const auto *costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
    if (!corridor_generator_.generate(raw_path, costmap, corridor_width_, corridors)) {
      RCLCPP_WARN(
        logger_,
        "[%s] Corridor生成失败，MINCO暂时使用空走廊",
        planner_name_.c_str());
      corridors.clear();
    }

    Eigen::Vector2d goal_vel = Eigen::Vector2d::Zero();
    {
      std::lock_guard<std::mutex> lk(target_mutex_);
      goal_vel = target_vel_;
    }

    nav_msgs::msg::Path minco_path;
    if (minco_optimizer_->generatePath(raw_path, Eigen::Vector2d::Zero(), goal_vel, "map", minco_path, corridors)){
      path = minco_path;
      path.header.stamp = rclcpp::Clock().now();
      RCLCPP_INFO(logger_, "[%s] MINCO路径生成成功：raw=%zu, sampled=%zu", planner_name_.c_str(),raw_path.size(),path.poses.size());
      return true;
    }
    RCLCPP_WARN(logger_, "[%s] MINCO路径生成失败，回退至A*到点路径", planner_name_.c_str());
  }

  // 5. Convert to nav_msgs::Path
  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Clock().now();
  for (size_t j = 0; j < raw_path.size(); j++) {
    const auto &pt = raw_path[j];
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    double dx = (j+1 < raw_path.size()) ? raw_path[j+1].x() - pt.x() : pt.x() - raw_path[j-1].x();
    double dy = (j+1 < raw_path.size()) ? raw_path[j+1].y() - pt.y() : pt.y() - raw_path[j-1].y();
    double yaw = std::atan2(dy, dx);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(pose);
  }

  // 6. Smooth path (SmacPlanner2D Smoother)
  smoothPath(path, costmap_ros_->getCostmap());

  return true;
}

bool ElasticPlanner::planStatic(const Eigen::Vector2d &start_pos,
                                const Eigen::Vector2d &goal_pos,
                                nav_msgs::msg::Path &path)
{
  std::vector<Eigen::Vector2d> raw_path;
  if (!env_->astar_search(start_pos, goal_pos, raw_path) || raw_path.size() < 2)
    return false;

  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Clock().now();
  for (size_t j = 0; j < raw_path.size(); j++) {
    const auto &pt = raw_path[j];
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    double yaw = 0.0;
    if (j+1 < raw_path.size()) {
      yaw = std::atan2(raw_path[j+1].y() - pt.y(), raw_path[j+1].x() - pt.x());
    } else if (j > 0) {
      yaw = std::atan2(pt.y() - raw_path[j-1].y(), pt.x() - raw_path[j-1].x());
    }
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(pose);
  }

  smoothPath(path, costmap_ros_->getCostmap());

  return true;
}

}  // namespace elastic_planner

PLUGINLIB_EXPORT_CLASS(elastic_planner::ElasticPlanner, nav2_core::GlobalPlanner)

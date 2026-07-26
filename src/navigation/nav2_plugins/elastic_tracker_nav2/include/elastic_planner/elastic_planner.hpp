/*
    Elastic-Tracker 2D Nav2 GlobalPlanner Plugin
    MINCO trajectory optimization for dynamic target tracking.

    Adapted from Elastic-Tracker (Jialin Ji, Neng Pan, Fei Gao, ICRA 2022)
    for ROS2 Nav2 integration.

    Planning pipeline:
      createPlan(start, goal) →
        Predict target trajectory (acceleration-space A*)
        → A* path search with tracking constraints
        → Generate 2D Safe Flight Corridors (convex polygons)
        → MINCO trajectory optimization with L-BFGS
        → Sample polynomial trajectory to nav_msgs::Path
*/

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_smac_planner/smoother.hpp"
#include "env/env_2d.hpp"
#include "env/polygon_zone.hpp"
#include "prediction/prediction_2d.hpp"
#include "elastic_tracker/corridor_2d.hpp"
#include "elastic_tracker/minco_optimizer.hpp"

namespace elastic_planner {

class ElasticPlanner : public nav2_core::GlobalPlanner {
 public:
  ElasticPlanner() = default;
  ~ElasticPlanner() override = default;

  // -------- Nav2 GlobalPlanner interface --------
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) override;

 private:
  // -------- ROS2 --------
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("ElasticPlanner")};
  std::string planner_name_;

  // Target pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  geometry_msgs::msg::PoseStamped latest_target_;
  Eigen::Vector2d target_vel_{0.0, 0.0};
  std::mutex target_mutex_;
  bool target_received_ = false;

  // Internal replanning timer (tracking mode)
  rclcpp::TimerBase::SharedPtr plan_timer_;
  nav_msgs::msg::Path cached_path_;
  std::mutex path_mutex_;
  geometry_msgs::msg::PoseStamped last_start_;
  rclcpp::Time last_plan_time_;
  bool start_received_ = false;
  double replan_hz_ = 10.0;

  // -------- Planning engines --------
  std::unique_ptr<env_2d::Env2D> env_;
  std::unique_ptr<prediction_2d::Predict2D> prediction_;

  // -------- Parameters --------
  double tracking_dist_ = 1.0;
  double tracking_dur_ = 3.0;
  double tracking_dt_ = 0.2;
  bool use_tracking_ = true;
  double target_timeout_ = 1.0;
  double min_turning_radius_ = 0.05;
  double max_planning_time_ = 4.5;
  nav2_smac_planner::SmootherParams smoother_params_;
  std::unique_ptr<nav2_smac_planner::Smoother> smoother_;
  bool minco_enabled_{true};
  elastic_tracker::MincoOptimizerConfig minco_config_;
  std::unique_ptr<elastic_tracker::MincoOptimizer> minco_optimizer_;
  elastic_tracker::CorridorGenerator corridor_generator_;
  double corridor_width_{0.8};
  PolygonZone tracking_zone_;

  bool planning_in_progress_ = false;  // guards against heartbeat-killing reentry

  // EKF for target velocity estimation
  bool ekf_enabled_ = true;
  double ekf_alpha_ = 0.1;      // position smoothing gain (lower = smoother)
  double ekf_beta_  = 0.05;     // velocity smoothing gain
  double ekf_reset_dt_ = 3.0;   // max dt before resetting EKF (s)
  Eigen::Vector2d ekf_pos_{0.0, 0.0};
  Eigen::Vector2d ekf_vel_{0.0, 0.0};
  rclcpp::Time last_ekf_time_;
  rclcpp::Time last_target_time_;

  // -------- Internal methods --------
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void updateEKF(const geometry_msgs::msg::PoseStamped &msg);
  void planTimerCallback();
  bool planTracked(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &goal_pos, nav_msgs::msg::Path &path);
  bool planStatic(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &goal_pos, nav_msgs::msg::Path &path);
  void smoothPath(nav_msgs::msg::Path &path, const nav2_costmap_2d::Costmap2D *costmap);
};

}  // namespace elastic_planner

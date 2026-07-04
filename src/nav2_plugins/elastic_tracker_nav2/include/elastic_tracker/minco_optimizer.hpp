#pragma once 

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "nav_msgs/msg/path.hpp"

namespace elastic_tracker{

struct MincoOptimizerConfig{
    double sample_dt{0.05};
    double nominal_speed{1.0};
    double min_piece_duration{0.15};
};

class MincoOptimizer{

public:
    explicit MincoOptimizer(const MincoOptimizerConfig &config);

    bool generatePath (const std::vector<Eigen::Vector2d> &raw_path, const Eigen::Vector2d &start_vel,
        const Eigen::Vector2d &goal_vel, const std::string &frame_id, nav_msgs::msg::Path &path) const;

    private:
        MincoOptimizerConfig config_;

        bool buildWaypointsAndTimes(const std::vector<Eigen::Vector2d> &raw_path, Eigen::Matrix3Xd &inner_points, Eigen::VectorXd &times) const;
};

}

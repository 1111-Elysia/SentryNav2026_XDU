#pragma once 

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "nav_msgs/msg/path.hpp"
#include "elastic_tracker/corridor_2d.hpp"
#include "gcopter/trajectory.hpp"

namespace elastic_tracker{

struct MincoOptimizerConfig{
    double sample_dt{0.05};
    double nominal_speed{1.0};
    double min_piece_duration{0.15};
    bool lbfgs_enabled{true};
    int lbfgs_max_iterations{50};
    double lbfgs_g_epsilon{1e-4};
    double finite_diff_eps{1e-4};
    double corridor_weight{50.0};
};

class MincoOptimizer{

public:
    explicit MincoOptimizer(const MincoOptimizerConfig &config);

    bool generatePath (const std::vector<Eigen::Vector2d> &raw_path, const Eigen::Vector2d &start_vel,
        const Eigen::Vector2d &goal_vel, const std::string &frame_id, nav_msgs::msg::Path &path, const std::vector<Corridor> &corridors) const;

    private:
        MincoOptimizerConfig config_;

        bool buildWaypointsAndTimes(const std::vector<Eigen::Vector2d> &raw_path, Eigen::Matrix3Xd &inner_points, Eigen::VectorXd &times) const;

        bool optimizerInnerPoints(Eigen::Matrix3Xd &inner_points, const Eigen::VectorXd &times, 
            const Eigen::Matrix3d &head_state, const Eigen::Matrix3d &tail_state, const std::vector<Corridor> &corridors) const;

        double evaluateCost(const Eigen::VectorXd &x, const Eigen::VectorXd &times, const Eigen::Matrix3d &head_state,
            const Eigen::Matrix3d &tail_state, const std::vector<Corridor> &corridors) const;

        Eigen::Matrix3Xd unpackInnerPoints(const Eigen::VectorXd &x) const;

        Eigen::VectorXd packInnerPoints(const Eigen::Matrix3Xd &inner_points) const;

        double computeCorridorPenalty(const Trajectory<5> &trajectory, const std::vector<Corridor> &corridor) const;
};

}

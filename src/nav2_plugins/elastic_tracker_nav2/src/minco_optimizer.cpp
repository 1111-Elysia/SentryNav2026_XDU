#include "elastic_tracker/minco_optimizer.hpp"
#include <algorithm>
#include <cmath>
#include <string>

#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"

namespace elastic_tracker{

    MincoOptimizer::MincoOptimizer(const MincoOptimizerConfig &config):config_(config){}

    bool MincoOptimizer::buildWaypointsAndTimes(const std::vector<Eigen::Vector2d> &raw_path, Eigen::Matrix3Xd &inner_points, Eigen::VectorXd &times) const{
        if (raw_path.size() < 2){
            return false;
        }
        const int piece_num = static_cast<int>(raw_path.size()) - 1;
        times.resize(piece_num);

        const double nominal_speed = std::max(config_.nominal_speed, 0.1);
        const double min_piece_duration = std::max(config_.min_piece_duration, 0.01);

        for (int i = 0; i < piece_num; i++){
            const double length = (raw_path[i + 1] - raw_path[i]).norm();
            times(i) = std::max(length / nominal_speed, min_piece_duration);
        }

        const int inner_count = static_cast<int>(raw_path.size()) - 2;
        inner_points.resize(3, std::max(inner_count, 0));

        for(int i= 0; i < inner_count; i++){
            inner_points(0, i) = raw_path[i + 1].x();
            inner_points(1, i) = raw_path[i + 1].y();
            inner_points(2, i) = 0.0;
        }
        return true;
    }

    bool MincoOptimizer::generatePath(const std::vector<Eigen::Vector2d> &raw_path, const Eigen::Vector2d &start_vel,
        const Eigen::Vector2d &goal_vel, const std::string &frame_id, nav_msgs::msg::Path &path) const{
        
        path.poses.clear();
        path.header.frame_id = frame_id;

        if(raw_path.size() < 2){
            return false;
        }

        Eigen::Matrix3Xd inner_points;
        Eigen::VectorXd times;
        if (!buildWaypointsAndTimes(raw_path, inner_points, times)){
            return false;
        }

        const int piece_num = static_cast<int>(times.size());
        Eigen::Matrix3d head_state = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d tail_state = Eigen::Matrix3d::Zero();

        head_state.col(0) << raw_path.front().x(), raw_path.front().y(), 0.0;
        head_state.col(1) << start_vel.x(), start_vel.y(), 0.0;
        head_state.col(2) << 0.0, 0.0, 0.0;

        tail_state.col(0) << raw_path.back().x(), raw_path.back().y(), 0.0;
        tail_state.col(1) << goal_vel.x(), goal_vel.y(), 0.0;
        tail_state.col(2) << 0.0, 0.0, 0.0;

        minco::MINCO_S3NU minco;
        minco.setConditions(head_state, tail_state, piece_num);
        minco.setParameters(inner_points, times);

        Trajectory<5> trajectory;
        minco.getTrajectory(trajectory);
        if (trajectory.getPieceNum() <= 0){
            return false;
        }

        const double sample_dt = std::max(config_.sample_dt, 0.01);
        for (int i = 0; i < trajectory.getPieceNum(); i++){
            const auto &piece = trajectory[i];
            const double duration = piece.getDuration();
            const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));

            for (int j = 0; j <= sample_count; j++){
                if (i > 0 && j == 0){
                    continue;
                }

                const double t = duration * static_cast<double>(j) / static_cast<double>(sample_count);
                const Eigen::Vector3d pos = piece.getPos(t);
                const Eigen::Vector3d vel = piece.getVel(t);

                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = pos.x();
                pose.pose.position.y = pos.y();
                pose.pose.position.z = 0.0;

                const double yaw = std::atan2(vel.y(), vel.x());
                pose.pose.orientation.z = std::sin(0.5 * yaw);
                pose.pose.orientation.w = std::cos(0.5 * yaw);
                path.poses.push_back(pose);
            }
        }
        return path.poses.size() >= 2;
        }
    
}

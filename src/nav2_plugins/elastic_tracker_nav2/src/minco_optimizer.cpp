#include "elastic_tracker/minco_optimizer.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/lbfgs.hpp"

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
        const Eigen::Vector2d &goal_vel, const std::string &frame_id, nav_msgs::msg::Path &path, const std::vector<Corridor> &corridors) const{
        (void)corridors;

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
        if(!optimizerInnerPoints(inner_points, times, head_state, tail_state, corridors)){
            return false;
        }

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

    Eigen::VectorXd MincoOptimizer::packInnerPoints(const Eigen::Matrix3Xd &inner_points) const{
        Eigen::VectorXd x(inner_points.cols() * 2);

        for (int i = 0; i < inner_points.cols(); i++){
            x(2 * i) = inner_points(0, i);
            x(2 * i + 1) = inner_points(1, i);
        }

        return x;
    }

    Eigen::Matrix3Xd MincoOptimizer::unpackInnerPoints(const Eigen::VectorXd &x) const{
        const int point_count = static_cast<int>(x.size() / 2);
        Eigen::Matrix3Xd inner_points(3, point_count);

        for(int i = 0; i < point_count; i++){
            inner_points(0, i) = x(2 * i);
            inner_points(1, i) = x(2 * i + 1);
            inner_points(2, i) = 0.0;
        }

        return inner_points;
    }

    // ──────────────────────────────────────────────────────
    //  Value-only evaluate (debug / quick checks only)
    // ──────────────────────────────────────────────────────
    double MincoOptimizer::evaluateCost(const Eigen::VectorXd &x, const Eigen::VectorXd &times, const Eigen::Matrix3d &head_state,
        const Eigen::Matrix3d &tail_state, const std::vector<Corridor> &corridors) const{

        const Eigen::Matrix3Xd inner_points = unpackInnerPoints(x);
        const int piece_num = static_cast<int>(times.size());

        minco::MINCO_S3NU minco;
        minco.setConditions(head_state, tail_state, piece_num);
        minco.setParameters(inner_points, times);

        double energy = 0.0;
        minco.getEnergy(energy);

        Trajectory<5> trajectory;
        minco.getTrajectory(trajectory);

        const double corridor_penalty = computeCorridorPenalty(trajectory, corridors);
        return energy + config_.corridor_weight * corridor_penalty;
    }

    // ──────────────────────────────────────────────────────
    //  Corridor penalty (value only)
    // ──────────────────────────────────────────────────────
    double MincoOptimizer::computeCorridorPenalty(const Trajectory<5> &trajectory, const std::vector<Corridor> &corridors) const{
        if (corridors.empty() || trajectory.getPieceNum() <= 0){
            return 0.0;
        }

        double penalty = 0.0;
        const double sample_dt = std::max(config_.sample_dt, 0.01);

        for(int i = 0; i < trajectory.getPieceNum(); i++){
            const auto &piece = trajectory[i];
            const double duration = piece.getDuration();
            const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
            const Corridor &corridor = corridors[std::min<int>(i, corridors.size() - 1)];

            for (int j = 0; j <= sample_count; j++){
                const double t = duration * static_cast<double>(j) / static_cast<double>(sample_count);
                const Eigen::Vector3d pos3 = piece.getPos(t);
                const Eigen::Vector2d p(pos3.x(), pos3.y());

                const Eigen::Matrix<double, 4, 1> violation = corridor.A * p - corridor.b;

                for(int k = 0; k < 4; k++){
                    if (violation(k) > 0.0){
                        penalty += violation(k)*violation(k)*violation(k);
                    }
                }
            }
        }
        return penalty;
    }

    // ──────────────────────────────────────────────────────────────────
    //  Analytical corridor penalty gradient w.r.t. MINCO coefficients b.
    //
    //  For each piece i at sample time t (normalised to [0, duration] × by
    //  cumulative piece offset):
    //
    //    p(t) = c5·t⁵ + c4·t⁴ + c3·t³ + c2·t² + c1·t + c0
    //
    //    ∂p/∂c_{5-k} = t^k   (k∈[0,5], coeff row 6*i+5-k)
    //
    //    violation v = A·p − b   (4 half-plane constraints)
    //    ∂(v³)/∂c = 3·v²·Aᵀ·∂p/∂c
    //
    //  Accumulated over all samples, then passed to MINCO::propogateGrad.
    // ──────────────────────────────────────────────────────────────────
    void MincoOptimizer::computeCorridorPenaltyGradByCoeffs(
        const minco::MINCO_S3NU &minco, const Trajectory<5> &trajectory,
        const std::vector<Corridor> &corridors,
        Eigen::MatrixX3d &gdC_corridor) const
    {
        const int piece_num = trajectory.getPieceNum();
        gdC_corridor.resize(6 * piece_num, 3);
        gdC_corridor.setZero();

        if (corridors.empty() || piece_num <= 0) return;

        const double sample_dt = std::max(config_.sample_dt, 0.01);

        for (int i = 0; i < piece_num; i++){
            const auto &piece = trajectory[i];
            const double duration = piece.getDuration();
            const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
            const Corridor &corridor = corridors[std::min<int>(i, corridors.size() - 1)];

            for (int j = 0; j <= sample_count; j++){
                const double tau = duration * static_cast<double>(j) / static_cast<double>(sample_count);
                const Eigen::Vector3d pos3 = piece.getPos(tau);
                const Eigen::Vector2d p(pos3.x(), pos3.y());

                const Eigen::Matrix<double, 4, 1> v = corridor.A * p - corridor.b;

                // ∇_p penalty = Σ 3·v_k²·Aₖᵀ   (only for active half-planes)
                Eigen::Vector2d dp_dp = Eigen::Vector2d::Zero();
                for (int k = 0; k < 4; k++){
                    if (v(k) > 0.0){
                        dp_dp += 3.0 * v(k) * v(k) * corridor.A.row(k).transpose();
                    }
                }
                if (dp_dp.squaredNorm() < 1e-18) continue;

                // Map ∇_p penalty → ∇_{coeffs} penalty for this piece.
                // Row 6*i+k stores c_{5-k} → ∂p/∂c_{5-k} = tau^{k}
                double tk[6];
                tk[0] = 1.0;           // k=0: ∂p/∂c5 = τ⁰ = 1    -> row 6*i
                tk[1] = tau;           // k=1: ∂p/∂c4 = τ¹         -> row 6*i+1
                tk[2] = tk[1] * tau;   // k=2: ∂p/∂c3 = τ²         -> row 6*i+2
                tk[3] = tk[2] * tau;   // k=3: ∂p/∂c2 = τ³         -> row 6*i+3
                tk[4] = tk[3] * tau;   // k=4: ∂p/∂c1 = τ⁴         -> row 6*i+4
                tk[5] = tk[4] * tau;   // k=5: ∂p/∂c0 = τ⁵         -> row 6*i+5

                for (int k = 0; k < 6; k++){
                    gdC_corridor(6 * i + k, 0) += tk[k] * dp_dp.x();
                    gdC_corridor(6 * i + k, 1) += tk[k] * dp_dp.y();
                }
            }
        }
    }

    // ──────────────────────────────────────────────────────────────────
    //  Unified cost + analytical gradient via MINCO adjoint propagation.
    //
    //  Energy gradient:   getEnergyPartialGradByCoeffs → propogateGrad
    //  Corridor gradient: computeCorridorPenaltyGradByCoeffs → propogateGrad
    //
    //  Both are summed, then mapped back to the flat inner-point vector.
    // ──────────────────────────────────────────────────────────────────
    double MincoOptimizer::evaluateCostWithGrad(
        const Eigen::VectorXd &x, const Eigen::VectorXd &times,
        const Eigen::Matrix3d &head_state, const Eigen::Matrix3d &tail_state,
        const std::vector<Corridor> &corridors,
        Eigen::VectorXd &grad) const
    {
        const Eigen::Matrix3Xd inner_points = unpackInnerPoints(x);
        const int piece_num = static_cast<int>(times.size());

        minco::MINCO_S3NU minco;
        minco.setConditions(head_state, tail_state, piece_num);
        minco.setParameters(inner_points, times);

        // ── Energy ────────────────────────────────────────────────
        double energy = 0.0;
        minco.getEnergy(energy);

        Eigen::MatrixX3d gdC_energy;
        minco.getEnergyPartialGradByCoeffs(gdC_energy);

        Eigen::VectorXd gdT_energy;
        minco.getEnergyPartialGradByTimes(gdT_energy);

        // ── Corridor penalty ──────────────────────────────────────
        Trajectory<5> trajectory;
        minco.getTrajectory(trajectory);

        double corridor_penalty = 0.0;
        Eigen::MatrixX3d gdC_corridor;
        if (!corridors.empty()){
            corridor_penalty = computeCorridorPenalty(trajectory, corridors);
            computeCorridorPenaltyGradByCoeffs(minco, trajectory, corridors, gdC_corridor);
        }

        const double cost = energy + config_.corridor_weight * corridor_penalty;

        // ── Sum gradients in coefficient space ────────────────────
        Eigen::MatrixX3d gdC_total = gdC_energy;
        if (!corridors.empty()){
            gdC_total += config_.corridor_weight * gdC_corridor;
        }
        // gdT_corridor is harder and minor — skip; just use energy part
        Eigen::VectorXd gdT_total = gdT_energy;

        // ── Adjoint: coefficient gradient → inner-point gradient ──
        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd gradByTimes;  // unused for our problem (times fixed)
        minco.propogateGrad(gdC_total, gdT_total, gradByPoints, gradByTimes);

        // ── Pack into flat gradient ───────────────────────────────
        grad.resize(x.size());
        for (int i = 0; i < piece_num - 1; i++){
            grad(2 * i)     = gradByPoints(0, i);  // x
            grad(2 * i + 1) = gradByPoints(1, i);  // y
        }

        return cost;
    }

    // ──────────────────────────────────────────────────────────────
    //  L-BFGS optimization wrapper (uses analytical gradient)
    // ──────────────────────────────────────────────────────────────
    bool MincoOptimizer::optimizerInnerPoints(
        Eigen::Matrix3Xd &inner_points, const Eigen::VectorXd &times,
        const Eigen::Matrix3d &head_state, const Eigen::Matrix3d &tail_state,
        const std::vector<Corridor> &corridors) const
    {
        if (!config_.lbfgs_enabled || inner_points.cols() == 0){
            std::cout << "[MincoOptimizer] lbfgs skipped, enabled="
                      << (config_.lbfgs_enabled ? "true" : "false")
                      << ", inner_points=" << inner_points.cols()
                      << std::endl;
            return true;
        }

        struct OptimizeContext{
            const MincoOptimizer *self;
            const Eigen::VectorXd *times;
            const Eigen::Matrix3d *head_state;
            const Eigen::Matrix3d *tail_state;
            const std::vector<Corridor> *corridors;
        };

        // Analytical gradient — one base evaluation + one adjoint solve per iteration.
        auto evaluate = [](void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &grad) -> double {
            auto *ctx = static_cast<OptimizeContext *>(instance);
            return ctx->self->evaluateCostWithGrad(
                x, *ctx->times, *ctx->head_state, *ctx->tail_state,
                *ctx->corridors, grad);
        };

        Eigen::VectorXd x = packInnerPoints(inner_points);

        OptimizeContext context;
        context.self = this;
        context.times = &times;
        context.head_state = &head_state;
        context.tail_state = &tail_state;
        context.corridors = &corridors;

        lbfgs::lbfgs_parameter_t params;
        params.max_iterations = config_.lbfgs_max_iterations;
        params.g_epsilon = config_.lbfgs_g_epsilon;
        params.past = 3;
        params.delta = 1e-4;

        double final_cost = 0.0;
        const int ret = lbfgs::lbfgs_optimize(x, final_cost, evaluate, nullptr, nullptr, &context, params);
        std::cout << "[MincoOptimizer] lbfgs ret=" << ret
                  << ", final_cost=" << final_cost
                  << ", variables=" << x.size()
                  << std::endl;
        if(ret < 0){
            return false;
        }
        inner_points = unpackInnerPoints(x);
        return true;
    }

}

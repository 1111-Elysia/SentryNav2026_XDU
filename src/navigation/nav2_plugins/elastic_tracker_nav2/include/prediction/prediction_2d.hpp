/*
    2D target motion prediction using A* search in acceleration space.
    Adapted from Elastic-Tracker's prediction.hpp.
*/

#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <queue>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace prediction_2d {

struct PredNode {
  Eigen::Vector2d p, v, a;
  double t = 0.0;
  double score = 0.0;
  double h = 0.0;
  PredNode *parent = nullptr;
};
using PredNodePtr = PredNode*;

class PredNodeComparator {
 public:
  bool operator()(PredNodePtr &lhs, PredNodePtr &rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};

class Predict2D {
  static constexpr int MAX_MEMORY = 1 << 18;  // ~262K nodes
  static constexpr double DEFAULT_MAX_TIME = 0.05;  // 50ms budget

  double dt_ = 0.2;
  double pre_dur_ = 3.0;
  double rho_a_ = 1.0;
  double vmax_ = 4.0;
  PredNode *data_[MAX_MEMORY];

  nav2_costmap_2d::Costmap2D *costmap_ = nullptr;
  double resolution_, origin_x_, origin_y_;
  int size_x_, size_y_;

  inline bool isOccupied(const Eigen::Vector2d &p) const {
    if (!costmap_) return false;
    int mx = static_cast<int>((p.x() - origin_x_) / resolution_);
    int my = static_cast<int>((p.y() - origin_y_) / resolution_);
    if (mx < 0 || mx >= size_x_ || my < 0 || my >= size_y_) return false;  // OOB = unknown, allow
    unsigned char c = costmap_->getCost(mx, my);
    return c == nav2_costmap_2d::LETHAL_OBSTACLE;  // only strictly lethal
  }
  inline bool isValid(const Eigen::Vector2d &p, const Eigen::Vector2d &v) const {
    return v.norm() < vmax_ && !isOccupied(p);
  }

 public:
  Predict2D() {
    for (int i = 0; i < MAX_MEMORY; ++i) data_[i] = new PredNode;
  }
  ~Predict2D() {
    for (int i = 0; i < MAX_MEMORY; ++i) delete data_[i];
  }

  void setParams(double dt, double pre_dur, double rho_a, double vmax) {
    dt_ = dt;
    pre_dur_ = pre_dur;
    rho_a_ = rho_a;
    vmax_ = vmax;
  }

  void setCostmap(nav2_costmap_2d::Costmap2D *costmap) {
    costmap_ = costmap;
    if (costmap) {
      resolution_ = costmap->getResolution();
      origin_x_ = costmap->getOriginX();
      origin_y_ = costmap->getOriginY();
      size_x_ = costmap->getSizeInCellsX();
      size_y_ = costmap->getSizeInCellsY();
    }
  }

  // Predict target trajectory using A* search in acceleration space
  // Input: target position, target velocity
  // Output: predicted positions over pre_dur_ horizon
  bool predict(const Eigen::Vector2d &target_p, const Eigen::Vector2d &target_v,
               std::vector<Eigen::Vector2d> &target_predict,
               const double &max_time = DEFAULT_MAX_TIME) {
    auto score_func = [&](const PredNodePtr &ptr) -> double {
      return rho_a_ * ptr->a.norm();
    };
    Eigen::Vector2d end_p = target_p + target_v * pre_dur_;
    auto calH = [&](const PredNodePtr &ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };

    auto t_start = std::chrono::steady_clock::now();
    std::priority_queue<PredNodePtr, std::vector<PredNodePtr>, PredNodeComparator> open_set;

    int stack_top = 0;
    PredNodePtr curPtr = data_[stack_top++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;

    double dt2_2 = dt_ * dt_ / 2;
    while (curPtr->t < pre_dur_) {
      for (double ax = -3.0; ax <= 3.0; ax += 3.0)
        for (double ay = -3.0; ay <= 3.0; ay += 3.0) {
          Eigen::Vector2d input(ax, ay);
          Eigen::Vector2d p = curPtr->p + curPtr->v * dt_ + input * dt2_2;
          Eigen::Vector2d v = curPtr->v + input * dt_;
          if (!isValid(p, v)) continue;
          if (stack_top == MAX_MEMORY) return false;

          auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
          if (elapsed > max_time) return false;

          PredNodePtr ptr = data_[stack_top++];
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt_;
          ptr->score = curPtr->score + score_func(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
        }
      if (open_set.empty()) return false;
      curPtr = open_set.top();
      open_set.pop();
    }

    target_predict.clear();
    while (curPtr != nullptr) {
      target_predict.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predict.begin(), target_predict.end());
    return true;
  }
};

}  // namespace prediction_2d

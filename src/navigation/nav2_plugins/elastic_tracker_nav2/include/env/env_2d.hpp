/*
    2D environment: A* path search + SFC generation for Nav2 costmap.
    Adapted from Elastic-Tracker's env.hpp.
*/

#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

// std::hash specialization for Eigen::Vector2i (must be before use)
namespace std {
template<>
struct hash<Eigen::Vector2i> {
  size_t operator()(const Eigen::Vector2i &v) const {
    return std::hash<int>()(v.x()) ^ (std::hash<int>()(v.y()) << 1);
  }
};
}  // namespace std

namespace env_2d {

enum State { OPEN, CLOSE, UNVISITED };

struct Node {
  Eigen::Vector2i idx;
  bool valid = false;
  State state = UNVISITED;
  double g = 0.0, h = 0.0;
  Node *parent = nullptr;
};
using NodePtr = Node*;

class NodeComparator {
 public:
  bool operator()(NodePtr &lhs, NodePtr &rhs) {
    return lhs->g + lhs->h > rhs->g + rhs->h;
  }
};

class Env2D {
  static constexpr int MAX_MEMORY = 1 << 18;  // ~262K nodes
  static constexpr double MAX_DURATION = 0.15;

 private:
  nav2_costmap_2d::Costmap2D *costmap_;
  std::unordered_map<Eigen::Vector2i, NodePtr, std::hash<Eigen::Vector2i>> visited_nodes_;
  NodePtr data_[MAX_MEMORY];
  double resolution_, origin_x_, origin_y_;
  int size_x_, size_y_;
  double cost_weight_ = 1.5;

  // Convert world to map index (Eigen row-major)
  inline Eigen::Vector2i world2idx(const Eigen::Vector2d &p) const {
    int mx = static_cast<int>((p.x() - origin_x_) / resolution_);
    int my = static_cast<int>((p.y() - origin_y_) / resolution_);
    return Eigen::Vector2i(mx, my);
  }
  inline Eigen::Vector2d idx2world(const Eigen::Vector2i &idx) const {
    return Eigen::Vector2d(
      origin_x_ + (idx.x() + 0.5) * resolution_,
      origin_y_ + (idx.y() + 0.5) * resolution_);
  }
  inline bool isOccupied(const Eigen::Vector2i &idx) const {
    if (idx.x() < 0 || idx.x() >= size_x_ || idx.y() < 0 || idx.y() >= size_y_)
      return true;  // out of bounds = occupied
    return costmap_->getCost(idx.x(), idx.y()) >= nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  inline bool isInMap(const Eigen::Vector2i &idx) const {
    return idx.x() >= 0 && idx.x() < size_x_ && idx.y() >= 0 && idx.y() < size_y_;
  }
  inline bool isFree(const Eigen::Vector2i &idx) const {
    if (!isInMap(idx)) return false;
    unsigned char c = costmap_->getCost(idx.x(), idx.y());
    return c != nav2_costmap_2d::LETHAL_OBSTACLE;  // only lethal (254) blocks; unknown OK for global planner
  }

  inline NodePtr visit(const Eigen::Vector2i &idx) {
    auto iter = visited_nodes_.find(idx);
    if (iter == visited_nodes_.end()) {
      auto ptr = data_[visited_nodes_.size()];
      ptr->idx = idx;
      ptr->valid = isFree(idx);
      ptr->state = UNVISITED;
      visited_nodes_[idx] = ptr;
      return ptr;
    }
    return iter->second;
  }

 public:
  Env2D() {
    for (int i = 0; i < MAX_MEMORY; ++i) data_[i] = new Node;
  }
  ~Env2D() {
    for (int i = 0; i < MAX_MEMORY; ++i) delete data_[i];
  }

  void setCostmap(nav2_costmap_2d::Costmap2D *costmap) {
    costmap_ = costmap;
    resolution_ = costmap->getResolution();
    origin_x_ = costmap->getOriginX();
    origin_y_ = costmap->getOriginY();
    size_x_ = costmap->getSizeInCellsX();
    size_y_ = costmap->getSizeInCellsY();
  }
  void setCostWeight(double w) { cost_weight_ = w; }

  // Standard A* path search (cost-aware)
  bool astar_search(const Eigen::Vector2d &start_p, const Eigen::Vector2d &end_p,
                    std::vector<Eigen::Vector2d> &path,
                    double timeout_ms = 50.0) {
    auto t_start = std::chrono::steady_clock::now();
    visited_nodes_.clear();
    Eigen::Vector2i start_idx = world2idx(start_p);
    Eigen::Vector2i end_idx = world2idx(end_p);

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    // 8-connected neighbors
    std::vector<std::pair<Eigen::Vector2i, double>> neighbors;
    for (int dx = -1; dx <= 1; dx++)
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        neighbors.emplace_back(Eigen::Vector2i(dx, dy),
                               (dx != 0 && dy != 0) ? 1.414 : 1.0);
      }

    NodePtr curPtr = visit(start_idx);
    if (!curPtr->valid) return false;
    curPtr->parent = nullptr;
    curPtr->g = 0;
    curPtr->h = (end_idx - start_idx).cast<double>().norm();
    curPtr->state = CLOSE;

    bool ret = false;
    while (visited_nodes_.size() < MAX_MEMORY) {
      // Hard timeout: abort if A* takes too long (costmap might be deactivated)
      auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
      if (elapsed > timeout_ms) break;

      for (const auto &nb : neighbors) {
        auto neighbor_idx = curPtr->idx + nb.first;
        NodePtr neighborPtr = visit(neighbor_idx);
        if (neighborPtr->state == CLOSE) continue;

        // 代价感知边权重
        unsigned char c = costmap_->getCost(neighbor_idx.x(), neighbor_idx.y());
        double new_g = curPtr->g + nb.second * (1.0 + cost_weight_ * c / 254.0);

        if (neighborPtr->state == OPEN) {
          if (neighborPtr->g > new_g) {
            neighborPtr->parent = curPtr;
            neighborPtr->g = new_g;
          }
          continue;
        }
        if (neighborPtr->state == UNVISITED && neighborPtr->valid) {
          neighborPtr->parent = curPtr;
          neighborPtr->state = OPEN;
          neighborPtr->g = new_g;
          neighborPtr->h = (end_idx - neighbor_idx).cast<double>().norm();
          open_set.push(neighborPtr);
        }
      }

      if (open_set.empty()) break;
      curPtr = open_set.top();
      open_set.pop();
      curPtr->state = CLOSE;

      if (curPtr->idx == end_idx) { ret = true; break; }
      if (visited_nodes_.size() == MAX_MEMORY) break;
    }

    if (ret) {
      std::vector<Eigen::Vector2i> idx_path;
      for (NodePtr ptr = curPtr; ptr != nullptr; ptr = ptr->parent)
        idx_path.push_back(ptr->idx);
      std::reverse(idx_path.begin(), idx_path.end());
      path.clear();
      for (const auto &idx : idx_path) path.push_back(idx2world(idx));
    }
    return ret;
  }

};

}  // namespace env_2d

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

  // DDA ray casting for line-of-sight check
  bool checkRayValid(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1) const {
    Eigen::Vector2d dp = p1 - p0;
    double dist = dp.norm();
    if (dist < resolution_) return true;

    Eigen::Vector2i idx0 = world2idx(p0);
    Eigen::Vector2i idx1 = world2idx(p1);
    Eigen::Vector2i d_idx = idx1 - idx0;
    Eigen::Vector2i step(
      d_idx.x() > 0 ? 1 : (d_idx.x() < 0 ? -1 : 0),
      d_idx.y() > 0 ? 1 : (d_idx.y() < 0 ? -1 : 0));

    Eigen::Vector2d delta_t;
    delta_t.x() = dp.x() == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(dp.x());
    delta_t.y() = dp.y() == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(dp.y());

    Eigen::Vector2d t_max;
    t_max.x() = step.x() > 0 ? (idx0.x() + 1) - p0.x() / resolution_ : p0.x() / resolution_ - idx0.x();
    t_max.y() = step.y() > 0 ? (idx0.y() + 1) - p0.y() / resolution_ : p0.y() / resolution_ - idx0.y();
    t_max = t_max.cwiseProduct(delta_t);

    Eigen::Vector2i rayIdx = idx0;
    while ((rayIdx - idx1).squaredNorm() > 1) {
      if (isOccupied(rayIdx)) return false;
      if (t_max.x() < t_max.y()) {
        rayIdx.x() += step.x();
        t_max.x() += delta_t.x();
      } else {
        rayIdx.y() += step.y();
        t_max.y() += delta_t.y();
      }
    }
    return true;
  }

  // Standard A* path search
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
        double new_g = curPtr->g + nb.second;

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

  // SFC generation: radial expansion along the path
  void generateSFC(const std::vector<Eigen::Vector2d> &path,
                   double bbox_width,
                   std::vector<Eigen::MatrixXd> &hPolys,
                   std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &keyPts) {
    hPolys.clear();
    keyPts.clear();
    if (path.size() < 2) return;

    int idx = 0;
    int path_len = path.size();

    while (idx < path_len - 1) {
      // Greedy forward extension
      int next_idx = idx;
      while (next_idx + 1 < path_len && checkRayValid(path[idx], path[next_idx + 1]))
        next_idx++;

      Eigen::Vector2d p0 = path[idx];
      Eigen::Vector2d p1 = path[next_idx];
      keyPts.emplace_back(p0, p1);

      // Generate a simple axis-aligned or rotated bounding box corridor
      Eigen::Vector2d dir = p1 - p0;
      double length = dir.norm();
      if (length < 1e-6) length = bbox_width;
      Eigen::Vector2d u = dir / length;       // forward direction
      Eigen::Vector2d n(-u.y(), u.x());       // perpendicular (left normal)

      // Build 4 half-planes forming a rectangular corridor around [p0, p1]
      // Columns: [nx, ny, d]^T  where nx*x + ny*y <= d
      Eigen::MatrixXd hPoly(3, 4);
      double half_w = bbox_width / 2.0;

      // n direction (left boundary): n·p <= n·p0 + half_w
      hPoly.col(0) << n.x(), n.y(), n.dot(p0) + half_w;
      // -n direction (right boundary): -n·p <= -n·p0 + half_w
      hPoly.col(1) << -n.x(), -n.y(), -n.dot(p0) + half_w;
      // -u direction (start): -u·p <= -u·p0
      hPoly.col(2) << -u.x(), -u.y(), -u.dot(p0);
      // u direction (end): u·p <= u·p1
      hPoly.col(3) << u.x(), u.y(), u.dot(p1);

      hPolys.push_back(hPoly);

      // Find furthest idx inside this corridor
      idx = next_idx;
      // Simple check: verify path[idx+1] is inside (relaxed)
      while (idx + 1 < path_len) {
        Eigen::Vector2d pt = path[idx + 1];
        bool inside = true;
        for (int i = 0; i < 4; i++) {
          double d = hPoly.col(i).head<2>().dot(pt) - hPoly.col(i)(2);
          if (d > 1e-3) { inside = false; break; }
        }
        if (inside) idx++; else break;
      }
    }
  }
};

}  // namespace env_2d

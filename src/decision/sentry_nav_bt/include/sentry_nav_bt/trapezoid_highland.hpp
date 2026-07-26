#ifndef SENTRY_NAV_BT_TRAPEZOID_HIGHLAND_HPP_
#define SENTRY_NAV_BT_TRAPEZOID_HIGHLAND_HPP_

#include <array>
#include <cmath>
#include <cstddef>

namespace sentry_nav_bt::trapezoid_highland
{

struct Point
{
  double x;
  double y;
};

// 2026 RMUC 场地为 28 m x 15 m，蓝方梯形高地与红方中心对称。
constexpr double kFieldLength = 28.0;
constexpr double kFieldWidth = 15.0;

// 红方梯形高地主体（含 23° 斜坡）的官方场地坐标。
// 顶点依据《RoboMaster 2026 超级对抗赛比赛规则手册 V2.0.0》
// 图 4-5（模块定位）和图 4-27（梯形高地尺寸）整理。
constexpr std::array<Point, 5> kRedHighlandPolygon{{
  {3.645, 14.850},
  {10.052, 14.850},
  {10.052, 13.847},
  {8.085, 10.770},
  {3.645, 10.770},
}};

inline Point normalizeToRedSide(double x, double y, int robot_id)
{
  if (robot_id == 107) {
    return {kFieldLength - x, kFieldWidth - y};
  }
  return {x, y};
}

inline bool pointOnSegment(const Point &point, const Point &start, const Point &end)
{
  constexpr double kEpsilon = 1e-6;
  const double cross =
    (point.x - start.x) * (end.y - start.y) -
    (point.y - start.y) * (end.x - start.x);
  if (std::abs(cross) > kEpsilon) {
    return false;
  }

  const double dot =
    (point.x - start.x) * (end.x - start.x) +
    (point.y - start.y) * (end.y - start.y);
  if (dot < -kEpsilon) {
    return false;
  }

  const double squared_length =
    (end.x - start.x) * (end.x - start.x) +
    (end.y - start.y) * (end.y - start.y);
  return dot <= squared_length + kEpsilon;
}

inline bool containsRedSidePoint(const Point &point)
{
  bool inside = false;
  for (std::size_t i = 0, j = kRedHighlandPolygon.size() - 1;
       i < kRedHighlandPolygon.size();
       j = i++)
  {
    const Point &current = kRedHighlandPolygon[i];
    const Point &previous = kRedHighlandPolygon[j];
    if (pointOnSegment(point, previous, current)) {
      return true;
    }

    const bool crosses_y = (current.y > point.y) != (previous.y > point.y);
    if (!crosses_y) {
      continue;
    }

    const double intersection_x =
      (previous.x - current.x) * (point.y - current.y) /
      (previous.y - current.y) + current.x;
    if (point.x < intersection_x) {
      inside = !inside;
    }
  }
  return inside;
}

inline bool contains(double hero_x, double hero_y, int robot_id)
{
  if (!std::isfinite(hero_x) || !std::isfinite(hero_y) ||
      (robot_id != 7 && robot_id != 107))
  {
    return false;
  }
  return containsRedSidePoint(normalizeToRedSide(hero_x, hero_y, robot_id));
}

}  // namespace sentry_nav_bt::trapezoid_highland

#endif  // SENTRY_NAV_BT_TRAPEZOID_HIGHLAND_HPP_

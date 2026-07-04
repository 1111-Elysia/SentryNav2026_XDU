#include "elastic_tracker/corridor_2d.hpp"
#include <algorithm>
#include <cmath>

namespace elastic_tracker{
    bool CorridorGenerator::generate(const std::vector<Eigen::Vector2d> &path, const nav2_costmap_2d::Costmap2D *costmap,
        double width, std::vector<Corridor> &corridors) const
    {
        corridors.clear();

        if (!costmap || path.size() < 2 || width <= 0.0){
            return false;
        }

        for (size_t i = 0; i + 1 < path.size(); i++){
            const auto &p0 = path[i];
            const auto &p1 = path[i+1];

            if (!segmentIsFree(p0, p1, costmap)){
                return false;
            }

            corridors.push_back(makeRectangleCorridor(p0, p1, width));
        }
        return !corridors.empty();
    }

    bool CorridorGenerator::segmentIsFree(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const nav2_costmap_2d::Costmap2D *costmap) const
    {
        const double resolution = costmap->getResolution();
        const double length = (p1 - p0).norm();

        const int steps = std::max(1, static_cast<int>(length/resolution));

        for (int i = 0; i <= steps; i++){
            const double t = static_cast<double>(i) / static_cast<double>(steps);
            const Eigen::Vector2d p =p0 + t * (p1 - p0);

            unsigned int mx = 0;
            unsigned int my = 0;
            if (!costmap->worldToMap(p.x(), p.y(), mx, my)){
                return false;
            }

            const unsigned char cost = costmap->getCost(mx, my);
            if(cost >= 254){
                return false;
            }
        }

        return true;
    }

    Corridor CorridorGenerator::makeRectangleCorridor(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, double width) const
    {
        const Eigen::Vector2d d = p1 - p0;
        const double len = std::max(d.norm(),1e-6);

        const Eigen::Vector2d u = d / len;
        const Eigen::Vector2d n(-u.y(), u.x());
        const double half_width = 0.5 * width;
        
        Corridor corridor;
        corridor.A.row(0) = n.transpose();
        corridor.b(0) = n.dot(p0) + half_width;

        corridor.A.row(1) = (-n).transpose();
        corridor.b(1) = (-n).dot(p0) + half_width;

        corridor.A.row(2) = (-u).transpose();
        corridor.b(2) = (-u).dot(p0);
    
        corridor.A.row(3) = u.transpose();
        corridor.b(3) = u.dot(p1);

        return corridor;
    }
}

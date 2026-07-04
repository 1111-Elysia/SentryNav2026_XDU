#pragma once 

#include <vector>
#include <Eigen/Eigen>
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace elastic_tracker{
    
    struct Corridor{
        Eigen::Matrix<double, 4, 2> A;
        Eigen::Matrix<double, 4, 1> b;
    };

    class CorridorGenerator{
    public:
        bool generate(const std::vector<Eigen::Vector2d> &path, const nav2_costmap_2d::Costmap2D *costmap, 
            double width, std::vector<Corridor> &corridors) const;

    private:
        bool segmentIsFree(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const nav2_costmap_2d::Costmap2D *costmap) const;

        Corridor makeRectangleCorridor(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, double width) const;
    };
}

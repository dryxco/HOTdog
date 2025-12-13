#pragma once

#include <vector>

namespace path_planning
{

class MapInflation
{
public:
    // map: 0 = free, 1 = obstacle
    // resolution: meter per cell
    // robot_radius: meters
    static std::vector<std::vector<int>> inflate(
        const std::vector<std::vector<int>>& map,
        double resolution,
        double robot_radius);
};

}  // namespace path_planning

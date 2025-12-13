#pragma once

#include <vector>
#include <cmath>
#include "path_planning/types.hpp"

namespace path_planning
{

class Utils
{
public:
    // World → Grid (origin = 0,0 기준)
    static inline int worldToGridX(double x, double resolution)
    {
        return static_cast<int>(std::floor(x / resolution));
    }

    static inline int worldToGridY(double y, double resolution)
    {
        return static_cast<int>(std::floor(y / resolution));
    }

    // Grid → World (cell center)
    static inline double gridToWorldX(int gx, double resolution)
    {
        return (gx + 0.5) * resolution;
    }

    static inline double gridToWorldY(int gy, double resolution)
    {
        return (gy + 0.5) * resolution;
    }

    // 단순 버전의 충돌 체크 (origin = 0,0 가정)
    static inline bool isPathCollisionFree(
        const std::vector<Point2D>& path,
        const std::vector<std::vector<int>>& map,
        double resolution)
    {
        if (map.empty()) return false;
        int h = static_cast<int>(map.size());
        int w = static_cast<int>(map[0].size());

        for (const auto& p : path)
        {
            int gx = static_cast<int>(std::floor(p.x / resolution));
            int gy = static_cast<int>(std::floor(p.y / resolution));

            if (gx < 0 || gy < 0 || gx >= w || gy >= h)
                return false;

            if (map[gy][gx] == 1)
                return false;
        }
        return true;
    }
};

}  // namespace path_planning

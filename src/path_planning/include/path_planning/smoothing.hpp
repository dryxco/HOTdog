#pragma once

#include <vector>
#include "path_planning/types.hpp"

namespace path_planning
{

class PathSmoothing
{
public:
    // Chaikin 알고리즘 기반 path smoothing
    static std::vector<Point2D> chaikin(
        const std::vector<Point2D>& path,
        int iterations = 3)
    {
        std::vector<Point2D> pts = path;
        if (pts.size() < 2) return pts;

        for (int it = 0; it < iterations; ++it)
        {
            std::vector<Point2D> new_pts;
            new_pts.reserve(pts.size() * 2);

            for (size_t i = 0; i + 1 < pts.size(); ++i)
            {
                const Point2D& p0 = pts[i];
                const Point2D& p1 = pts[i + 1];

                Point2D Q{
                    0.75 * p0.x + 0.25 * p1.x,
                    0.75 * p0.y + 0.25 * p1.y
                };
                Point2D R{
                    0.25 * p0.x + 0.75 * p1.x,
                    0.25 * p0.y + 0.75 * p1.y
                };

                new_pts.push_back(Q);
                new_pts.push_back(R);
            }

            pts.swap(new_pts);
        }

        return pts;
    }
};

}  // namespace path_planning

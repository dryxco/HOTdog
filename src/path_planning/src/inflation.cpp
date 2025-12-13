#include "path_planning/inflation.hpp"

#include <queue>
#include <cmath>

namespace path_planning
{

std::vector<std::vector<int>> MapInflation::inflate(
    const std::vector<std::vector<int>>& map,
    double resolution,
    double robot_radius)
{
    int h = static_cast<int>(map.size());
    int w = static_cast<int>(map[0].size());

    // Distance field 초기화
    std::vector<std::vector<double>> dist(h, std::vector<double>(w, 1e9));
    std::queue<std::pair<int, int>> q;

    // 장애물 셀을 distance 0으로 설정
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            if (map[y][x] == 1)
            {
                dist[y][x] = 0.0;
                q.push({x, y});
            }
        }
    }

    // 8방향 distance propagation
    int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    int dy[8] = { 0, 1, 1,  1,  0, -1,-1,-1 };

    while (!q.empty())
    {
        auto cur = q.front();
        q.pop();
        int x = cur.first;
        int y = cur.second;

        for (int k = 0; k < 8; ++k)
        {
            int nx = x + dx[k];
            int ny = y + dy[k];

            if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                continue;

            double step_cost = (k % 2 == 0) ? resolution : resolution * std::sqrt(2.0);

            if (dist[ny][nx] > dist[y][x] + step_cost)
            {
                dist[ny][nx] = dist[y][x] + step_cost;
                q.push({nx, ny});
            }
        }
    }

    std::vector<std::vector<int>> inflated = map;
    double inflate_dist = robot_radius;

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            if (dist[y][x] < inflate_dist)
            {
                inflated[y][x] = 1;
            }
        }
    }

    return inflated;
}

}  // namespace path_planning

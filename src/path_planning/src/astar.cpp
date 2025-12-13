#include "path_planning/astar.hpp"
#include "path_planning/utils.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace path_planning
{

AStar::AStar()
    : map_width_(0), map_height_(0)
{
}

void AStar::setMap(const std::vector<std::vector<int>>& map)
{
    map_ = map;

    if (!map_.empty())
    {
        map_height_ = static_cast<int>(map.size());
        map_width_  = static_cast<int>(map[0].size());
    }
    else
    {
        map_height_ = 0;
        map_width_ = 0;
    }
}

double AStar::calculateHeuristic(const GridCell& a, const GridCell& b) const
{
    return std::hypot(static_cast<double>(a.x - b.x),
                      static_cast<double>(a.y - b.y));
}

bool AStar::isValid(const GridCell& c) const
{
    return (c.x >= 0 && c.x < map_width_ &&
            c.y >= 0 && c.y < map_height_ &&
            map_[c.y][c.x] == 0);
}

std::vector<GridCell> AStar::getNeighbors(const GridCell& c) const
{
    std::vector<GridCell> nbrs;
    nbrs.reserve(8);

    const int dx[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
    const int dy[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };

    for (int i = 0; i < 8; ++i)
    {
        GridCell nb{ c.x + dx[i], c.y + dy[i] };
        if (isValid(nb))
            nbrs.push_back(nb);
    }

    return nbrs;
}

std::vector<GridCell> AStar::reconstructPath(
    const std::unordered_map<GridCell, GridCell, GridCellHash>& came_from,
    const GridCell& start,
    const GridCell& goal) const
{
    std::vector<GridCell> path;
    GridCell current = goal;

    while (!(current == start))
    {
        path.push_back(current);

        auto it = came_from.find(current);
        if (it == came_from.end())
            break;

        current = it->second;
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<GridCell> AStar::findPathGrid(const GridCell& start, const GridCell& goal)
{
    std::vector<GridCell> empty_path;

    if (!isValid(start) || !isValid(goal))
    {
        std::cerr << "[A*] Invalid start or goal\n";
        return empty_path;
    }

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<GridCell, bool, GridCellHash> closed;
    std::unordered_map<GridCell, double, GridCellHash> g_score;
    std::unordered_map<GridCell, GridCell, GridCellHash> came_from;

    Node start_node;
    start_node.cell = start;
    start_node.g_cost = 0.0;
    start_node.h_cost = calculateHeuristic(start, goal);
    start_node.f_cost = start_node.g_cost + start_node.h_cost;
    start_node.parent = GridCell(-1, -1);

    open_set.push(start_node);
    g_score[start] = 0.0;

    while (!open_set.empty())
    {
        Node current = open_set.top();
        open_set.pop();

        if (current.cell == goal)
            return reconstructPath(came_from, start, goal);

        if (closed[current.cell])
            continue;

        closed[current.cell] = true;

        for (const auto& nb : getNeighbors(current.cell))
        {
            if (closed[nb]) continue;

            double cost = calculateHeuristic(current.cell, nb);
            double tentative_g = current.g_cost + cost;

            auto it = g_score.find(nb);
            if (it == g_score.end() || tentative_g < it->second)
            {
                came_from[nb] = current.cell;
                g_score[nb] = tentative_g;

                Node next;
                next.cell = nb;
                next.g_cost = tentative_g;
                next.h_cost = calculateHeuristic(nb, goal);
                next.f_cost = next.g_cost + next.h_cost;
                next.parent = current.cell;

                open_set.push(next);
            }
        }
    }

    std::cerr << "[A*] No path found.\n";
    return empty_path;
}

bool AStar::findPathWorld(
    const Point2D& start_w,
    const Point2D& goal_w,
    double resolution,
    std::vector<Point2D>& path_out)
{
    if (map_.empty()) return false;

    GridCell start(
        Utils::worldToGridX(start_w.x, resolution),
        Utils::worldToGridY(start_w.y, resolution)
    );

    GridCell goal(
        Utils::worldToGridX(goal_w.x, resolution),
        Utils::worldToGridY(goal_w.y, resolution)
    );

    auto grid_path = findPathGrid(start, goal);
    if (grid_path.empty())
        return false;

    path_out.clear();
    path_out.reserve(grid_path.size());

    for (const auto& c : grid_path)
    {
        path_out.emplace_back(
            Utils::gridToWorldX(c.x, resolution),
            Utils::gridToWorldY(c.y, resolution)
        );
    }

    return true;
}

}  // namespace path_planning

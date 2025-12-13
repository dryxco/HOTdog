#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include "path_planning/types.hpp"
#include "path_planning/utils.hpp"

namespace path_planning
{

struct GridCell
{
    int x;
    int y;

    GridCell() : x(0), y(0) {}
    GridCell(int _x, int _y) : x(_x), y(_y) {}

    bool operator==(const GridCell& other) const
    {
        return x == other.x && y == other.y;
    }
};

struct GridCellHash
{
    std::size_t operator()(const GridCell& cell) const
    {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

struct Node
{
    GridCell cell;
    double g_cost;  // cost from start
    double h_cost;  // heuristic to goal
    double f_cost;  // g + h
    GridCell parent;

    bool operator>(const Node& other) const
    {
        return f_cost > other.f_cost;
    }
};

class AStar
{
public:
    AStar();

    // set inflated map (0 free, 1 obstacle)
    void setMap(const std::vector<std::vector<int>>& map);

    int getMapWidth() const  { return map_width_; }
    int getMapHeight() const { return map_height_; }

    // 1) Grid 좌표 기반 A*
    std::vector<GridCell> findPathGrid(
        const GridCell& start,
        const GridCell& goal);

    // 2) World 좌표 기반 A* (원하면 사용 가능)
    bool findPathWorld(
        const Point2D& start_w,
        const Point2D& goal_w,
        double resolution,
        std::vector<Point2D>& path_out);

private:
    std::vector<std::vector<int>> map_;
    int map_width_;
    int map_height_;

    double calculateHeuristic(const GridCell& a, const GridCell& b) const;
    bool isValid(const GridCell& c) const;
    std::vector<GridCell> getNeighbors(const GridCell& c) const;

    std::vector<GridCell> reconstructPath(
        const std::unordered_map<GridCell, GridCell, GridCellHash>& came_from,
        const GridCell& start,
        const GridCell& goal) const;
};

}  // namespace path_planning

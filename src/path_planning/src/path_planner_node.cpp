#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "path_planning/astar.hpp"
#include "path_planning/inflation.hpp"
#include "path_planning/types.hpp"
#include "path_planning/smoothing.hpp"

using namespace std::chrono_literals;
using path_planning::Point2D;
using path_planning::GridCell;
using path_planning::AStar;
using path_planning::MapInflation;



// ------------------------
// Collision check
// ------------------------
static bool isPathCollisionFree(
    const std::vector<Point2D>& path,
    const std::vector<std::vector<int>>& inflated_map,
    double resolution,
    double origin_x,
    double origin_y)
{
    if (inflated_map.empty()) return false;
    int h = static_cast<int>(inflated_map.size());
    int w = static_cast<int>(inflated_map[0].size());

    for (const auto& p : path) {
        int gx = static_cast<int>(std::floor((p.x - origin_x) / resolution));
        int gy = static_cast<int>(std::floor((p.y - origin_y) / resolution));

        if (gx < 0 || gy < 0 || gx >= w || gy >= h) {
            return false; // out of bounds → collision
        }
        if (inflated_map[gy][gx] == 1) {
            return false;
        }
    }
    return true;
}

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode()
  : Node("path_planner_node")
  {
    // Parameters
    this->declare_parameter<double>("resolution", 0.05);     // fallback
    this->declare_parameter<double>("robot_radius", 0.3);    // meters
    this->declare_parameter<int>("smoothing_iterations", 3);

    resolution_           = this->get_parameter("resolution").as_double();
    robot_radius_         = this->get_parameter("robot_radius").as_double();
    smoothing_iterations_ = this->get_parameter("smoothing_iterations").as_int();

    has_map_ = false;
    has_goal_ = false;
    has_current_pose_ = false;
    goal_reached_ = false;

    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PathPlannerNode::mapCallback, this, std::placeholders::_1));

    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/go1_pose", 10,
      std::bind(&PathPlannerNode::currentPoseCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

    // Publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "Use RViz2 '2D Goal Pose' tool to set a goal");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;

    int width  = static_cast<int>(msg->info.width);
    int height = static_cast<int>(msg->info.height);

    // Override resolution with map info
    resolution_ = msg->info.resolution;

    raw_map_grid_.clear();
    raw_map_grid_.resize(height, std::vector<int>(width));

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        int8_t v = msg->data[idx];

        if (v > 50 || v < 0) {
          raw_map_grid_[y][x] = 1;   // obstacle
        } else {
          raw_map_grid_[y][x] = 0;   // free
        }
      }
    }

    // Inflate map
    inflated_map_grid_ = MapInflation::inflate(raw_map_grid_, resolution_, robot_radius_);

    // Set inflated map to A*
    astar_.setMap(inflated_map_grid_);

    if (!has_map_) {
      has_map_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Map received & inflated: %dx%d (res=%.3f, robot_radius=%.2f)",
                  width, height, resolution_, robot_radius_);
    }
  }

  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_current_pose_) {
      has_current_pose_ = true;
      current_pose_ = *msg;
      previous_pose_ = *msg;
      RCLCPP_INFO(this->get_logger(), "Robot position initialized at (%.2f, %.2f)",
        current_pose_.pose.position.x, current_pose_.pose.position.y);
      return;
    }

    double dx = msg->pose.position.x - previous_pose_.pose.position.x;
    double dy = msg->pose.position.y - previous_pose_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 0.01) {
      return;
    }

    current_pose_ = *msg;

    if (has_goal_) {
      double goal_dx = current_pose_.pose.position.x - goal_pose_.pose.position.x;
      double goal_dy = current_pose_.pose.position.y - goal_pose_.pose.position.y;
      double goal_distance = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);

      if (goal_distance < 0.5) {
        if (!goal_reached_) {
          RCLCPP_INFO(this->get_logger(), "✓ Goal reached!");
          goal_reached_ = true;
        }
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Robot moved to (%.2f, %.2f)",
      current_pose_.pose.position.x, current_pose_.pose.position.y);

    previous_pose_ = current_pose_;

    if (has_map_ && has_goal_ && !goal_reached_) {
      replanPath();
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    goal_reached_ = false;

    RCLCPP_INFO(this->get_logger(),
      "New goal received: (%.2f, %.2f)",
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y);

    publishGoalMarker();

    if (has_map_ && has_current_pose_) {
      replanPath();
    }
  }

 void replanPath()
  {
    // 1. 필수 데이터 확인
    if (!has_map_ || !has_current_pose_ || !has_goal_) {
      return;
    }

    // 2. Start / Goal을 Grid 좌표로 변환
    GridCell start = worldToGrid(
      current_pose_.pose.position.x,
      current_pose_.pose.position.y);

    GridCell goal = worldToGrid(
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y);

    // 3. A* 경로 탐색 (Grid 기반)
    auto path_cells = astar_.findPathGrid(start, goal);

    if (path_cells.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path found!");
      return;
    }

    // 4. Grid Path → World Path 변환
    std::vector<Point2D> world_path;
    world_path.reserve(path_cells.size());
    for (const auto& cell : path_cells) {
      auto xy = gridToWorld(cell.x, cell.y);
      world_path.emplace_back(xy.first, xy.second);
    }

    // 5. Smoothing (Chaikin 알고리즘 적용)
    auto smooth_path = path_planning::PathSmoothing::chaikin(world_path, smoothing_iterations_);

    // 6. 충돌 체크 (스무딩된 경로가 안전한지 확인)
    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;

    if (!isPathCollisionFree(smooth_path, inflated_map_grid_, resolution_, origin_x, origin_y)) {
      RCLCPP_WARN(this->get_logger(), "Smoothed path collides, fallback to raw path.");
      smooth_path = world_path; // 충돌 시 원본 경로 사용
    }

    // 7. ROS Message 생성
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    // 첫 번째 점은 현재 로봇의 자세(Pose)를 그대로 사용 (부드러운 출발 위함)
    geometry_msgs::msg::PoseStamped first_pose;
    first_pose.header.stamp = this->now();
    first_pose.header.frame_id = "map";
    first_pose.pose = current_pose_.pose;
    path_msg.poses.push_back(first_pose);

    // 경로상의 점들을 메시지에 추가
    for (size_t i = 0; i < smooth_path.size(); ++i) {
      const auto& pt = smooth_path[i];

      // 현재 위치와 너무 가까운 첫 번째 웨이포인트는 건너뜀 (진동 방지)
      double dx = pt.x - current_pose_.pose.position.x;
      double dy = pt.y - current_pose_.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (i == 0 && dist < 0.3) {
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.3; // Go1 높이 고려 (선택 사항)

      // [핵심 수정] 다음 웨이포인트를 바라보는 방향(Yaw) 계산
      double yaw = 0.0;
      if (i + 1 < smooth_path.size()) {
          // 다음 점이 있다면, 다음 점을 향하는 각도 계산
          double next_dx = smooth_path[i+1].x - pt.x;
          double next_dy = smooth_path[i+1].y - pt.y;
          yaw = std::atan2(next_dy, next_dx);
      } else if (i > 0) {
          // 마지막 점이라면, 바로 이전 점과의 각도를 유지
          double prev_dx = pt.x - smooth_path[i-1].x;
          double prev_dy = pt.y - smooth_path[i-1].y;
          yaw = std::atan2(prev_dy, prev_dx);
      }

      // Yaw -> Quaternion 변환 (z, w만 설정하면 됨)
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);

      path_msg.poses.push_back(pose);
    }

    // 8. Publish
    path_pub_->publish(path_msg);

    // 디버깅용 마커 Publish (Grid Path 기준)
    publishPathMarkers(path_cells);

    // 로그 출력 (경로 크기가 크게 변했을 때만)
    static size_t last_path_size = 0;
    if (last_path_size == 0 || std::abs(static_cast<int>(path_cells.size()) - static_cast<int>(last_path_size)) > 3) {
      RCLCPP_INFO(this->get_logger(), "Path updated: %zu waypoints (grid)", path_cells.size());
      last_path_size = path_cells.size();
    }
  }

  GridCell worldToGrid(double x, double y)
  {
    GridCell cell;

    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;
    double res = map_msg_->info.resolution;

    cell.x = static_cast<int>((x - origin_x) / res);
    cell.y = static_cast<int>((y - origin_y) / res);

    return cell;
  }

  std::pair<double, double> gridToWorld(int x, int y)
  {
    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;
    double res = map_msg_->info.resolution;

    double wx = origin_x + (x + 0.5) * res;
    double wy = origin_y + (y + 0.5) * res;
    return {wx, wy};
  }

  void publishPathMarkers(const std::vector<GridCell>& path)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.1;
    line_marker.color.r = 0.0f;
    line_marker.color.g = 1.0f;
    line_marker.color.b = 0.0f;
    line_marker.color.a = 1.0f;

    for (const auto& cell : path) {
      geometry_msgs::msg::Point p;
      auto xy = gridToWorld(cell.x, cell.y);
      p.x = xy.first;
      p.y = xy.second;
      p.z = 0.1;
      line_marker.points.push_back(p);
    }

    marker_array.markers.push_back(line_marker);
    viz_pub_->publish(marker_array);
  }

  void publishGoalMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = goal_pose_.pose.position.x;
    marker.pose.position.y = goal_pose_.pose.position.y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;

    goal_marker_pub_->publish(marker);
  }

  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

  // State
  bool has_map_;
  bool has_goal_;
  bool has_current_pose_;
  bool goal_reached_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped previous_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  // Maps
  std::vector<std::vector<int>> raw_map_grid_;
  std::vector<std::vector<int>> inflated_map_grid_;

  // A*
  AStar astar_;

  // Parameters
  double resolution_;
  double robot_radius_;
  int    smoothing_iterations_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

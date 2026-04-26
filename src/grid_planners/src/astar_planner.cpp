#include "grid_planners/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "grid_planners/planner_stats.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(grid_planners::AStarPlanner, nav2_core::GlobalPlanner)

namespace grid_planners
{

// 8-connected grid: dx, dy, movement cost
static constexpr int DX[8]        = {-1,  0,  1, -1,  1, -1,  0,  1};
static constexpr int DY[8]        = {-1, -1, -1,  0,  0,  1,  1,  1};
static constexpr float STEP[8]    = {1.414f, 1.0f, 1.414f, 1.0f, 1.0f, 1.414f, 1.0f, 1.414f};

float AStarPlanner::heuristic(int idx, int gx, int gy, int W) const
{
  const float dx = std::abs(static_cast<float>(idx % W) - static_cast<float>(gx));
  const float dy = std::abs(static_cast<float>(idx / W) - static_cast<float>(gy));
  return std::max(dx, dy) + (std::sqrt(2.0f) - 1.0f) * std::min(dx, dy);
}

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  allow_unknown_ = node->get_parameter(name_ + ".allow_unknown").as_bool();
  stats_pub_ = node->create_publisher<std_msgs::msg::String>("/planner_stats", rclcpp::QoS(10));

  RCLCPP_INFO(logger_, "AStarPlanner configured (allow_unknown=%s)",
              allow_unknown_ ? "true" : "false");
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = goal.header.frame_id;
  path.header.stamp = node_.lock()->now();
  RCLCPP_INFO(logger_, "createPlan called: (%.2f,%.2f) -> (%.2f,%.2f)",
              start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);

  auto * cm = costmap_ros_->getCostmap();
  const int W = static_cast<int>(cm->getSizeInCellsX());
  const int H = static_cast<int>(cm->getSizeInCellsY());
  const int N = W * H;

  unsigned int sx, sy, gx, gy;
  if (!cm->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
      !cm->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
  {
    RCLCPP_WARN(logger_, "Start or goal out of costmap bounds");
    return path;
  }

  const int s_idx = static_cast<int>(sy) * W + static_cast<int>(sx);
  const int g_idx = static_cast<int>(gy) * W + static_cast<int>(gx);

  if (s_idx == g_idx) {
    path.poses.push_back(goal);
    return path;
  }

  static constexpr float INF = std::numeric_limits<float>::infinity();
  std::vector<float> g_cost(N, INF);
  std::vector<int> parent(N, -1);
  std::vector<bool> closed(N, false);

  // min-heap: (f, cell_index)
  using Entry = std::pair<float, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  g_cost[s_idx] = 0.0f;
  open.push({this->priority(0.0f, this->heuristic(s_idx, static_cast<int>(gx), static_cast<int>(gy), W)), s_idx});

  const auto t0 = std::chrono::high_resolution_clock::now();
  int nodes_expanded = 0;
  bool found = false;
  nav_msgs::msg::Path result;

  while (!open.empty()) {
    const auto [f, cur] = open.top();
    open.pop();

    if (closed[cur]) continue;
    closed[cur] = true;
    ++nodes_expanded;

    if (cur == g_idx) {
      result = buildPath(parent, g_idx, cm, path.header.frame_id, path.header.stamp);
      found = true;
      break;
    }

    const int cx = cur % W;
    const int cy = cur / W;

    for (int d = 0; d < 8; ++d) {
      const int nx = cx + DX[d];
      const int ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;

      const int nidx = ny * W + nx;
      if (closed[nidx]) continue;

      const unsigned char cost = cm->getCost(nx, ny);
      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        if (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) {
          // treat as free but add a penalty
        } else {
          continue;
        }
      }

      // scale costmap value into a small additive term (0–1 range)
      const float terrain = (cost == nav2_costmap_2d::NO_INFORMATION)
                            ? 0.5f
                            : 0.01f * static_cast<float>(cost);
      const float ng = g_cost[cur] + STEP[d] + terrain;

      if (ng < g_cost[nidx]) {
        g_cost[nidx] = ng;
        parent[nidx] = cur;
        open.push({this->priority(ng, this->heuristic(nidx, static_cast<int>(gx), static_cast<int>(gy), W)), nidx});
      }
    }
  }

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now() - t0).count();
  publishPlannerStats(stats_pub_, name_, ms, computePathLength(result), nodes_expanded, found);

  if (!found) {
    RCLCPP_WARN(logger_, "A* could not find a path from (%.2f,%.2f) to (%.2f,%.2f)",
                start.pose.position.x, start.pose.position.y,
                goal.pose.position.x, goal.pose.position.y);
    return path;
  }
  return result;
}

nav_msgs::msg::Path AStarPlanner::buildPath(
  const std::vector<int> & parent,
  int goal_idx,
  nav2_costmap_2d::Costmap2D * costmap,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  std::vector<int> indices;
  for (int idx = goal_idx; idx != -1; idx = parent[idx]) {
    indices.push_back(idx);
  }
  std::reverse(indices.begin(), indices.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(indices.size());

  const int W = static_cast<int>(costmap->getSizeInCellsX());
  for (int idx : indices) {
    double wx, wy;
    costmap->mapToWorld(idx % W, idx / W, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace grid_planners

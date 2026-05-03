#include "grid_planners/jps_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

#include "grid_planners/planner_stats.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(grid_planners::JPSPlanner, nav2_core::GlobalPlanner)

namespace grid_planners
{

bool JPSPlanner::blocked(int x, int y, nav2_costmap_2d::Costmap2D * cm) const
{
  if (x < 0 || y < 0 || x >= W_ || y >= H_) return true;
  const unsigned char c = cm->getCost(static_cast<unsigned int>(x),
                                      static_cast<unsigned int>(y));
  if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return !(c == nav2_costmap_2d::NO_INFORMATION && allow_unknown_);
  }
  return false;
}

// 跳点只在目标或强迫邻居处停下，跳过对最短路没有区分度的对称节点
int JPSPlanner::jump(
  int x, int y, int dx, int dy, int gx, int gy,
  nav2_costmap_2d::Costmap2D * cm) const
{
  int nx = x + dx;
  int ny = y + dy;

  while (true) {
    if (blocked(nx, ny, cm)) return -1;

    if (nx == gx && ny == gy) return ny * W_ + nx;

    if (dx != 0 && dy != 0) {
      // 对角跳跃还要检查两个正交方向，保证不会错过被障碍逼出的转折点
      if ((!blocked(nx - dx, ny + dy, cm) && blocked(nx - dx, ny, cm)) ||
          (!blocked(nx + dx, ny - dy, cm) && blocked(nx, ny - dy, cm))) {
        return ny * W_ + nx;
      }
      if (jump(nx, ny, dx, 0, gx, gy, cm) != -1 ||
          jump(nx, ny, 0, dy, gx, gy, cm) != -1) {
        return ny * W_ + nx;
      }
    } else if (dx != 0) {
      if ((!blocked(nx + dx, ny + 1, cm) && blocked(nx, ny + 1, cm)) ||
          (!blocked(nx + dx, ny - 1, cm) && blocked(nx, ny - 1, cm))) {
        return ny * W_ + nx;
      }
    } else {
      if ((!blocked(nx + 1, ny + dy, cm) && blocked(nx + 1, ny, cm)) ||
          (!blocked(nx - 1, ny + dy, cm) && blocked(nx - 1, ny, cm))) {
        return ny * W_ + nx;
      }
    }

    nx += dx;
    ny += dy;
  }
}

// JPS 父链只记录跳点，发布给 Nav2 前需要补回连续栅格
nav_msgs::msg::Path JPSPlanner::buildJPSPath(
  const std::vector<int> & parent,
  int goal_idx,
  nav2_costmap_2d::Costmap2D * cm,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  std::vector<int> jps;
  for (int idx = goal_idx; idx != -1; idx = parent[idx]) {
    jps.push_back(idx);
  }
  std::reverse(jps.begin(), jps.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;

  auto appendCell = [&](int idx) {
    double wx, wy;
    cm->mapToWorld(idx % W_, idx / W_, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  };

  if (jps.empty()) return path;
  appendCell(jps.front());

  for (size_t i = 1; i < jps.size(); ++i) {
    int ax = jps[i - 1] % W_,  ay = jps[i - 1] / W_;
    int bx = jps[i]     % W_,  by = jps[i]     / W_;

    const int sdx = (bx > ax) ? 1 : (bx < ax) ? -1 : 0;
    const int sdy = (by > ay) ? 1 : (by < ay) ? -1 : 0;

    int cx = ax + sdx, cy = ay + sdy;
    while (cx != bx || cy != by) {
      appendCell(cy * W_ + cx);
      cx += sdx;
      cy += sdy;
    }
    appendCell(jps[i]);
  }

  return path;
}

nav_msgs::msg::Path JPSPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = goal.header.frame_id;
  path.header.stamp = node_.lock()->now();

  RCLCPP_INFO(logger_, "JPS createPlan: (%.2f,%.2f) → (%.2f,%.2f)",
              start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);

  auto * cm = costmap_ros_->getCostmap();
  W_ = static_cast<int>(cm->getSizeInCellsX());
  H_ = static_cast<int>(cm->getSizeInCellsY());
  const int N = W_ * H_;

  unsigned int sx, sy, gx, gy;
  if (!cm->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
      !cm->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
  {
    RCLCPP_WARN(logger_, "JPS: start or goal outside costmap");
    return path;
  }

  const int s_idx = static_cast<int>(sy) * W_ + static_cast<int>(sx);
  const int g_idx = static_cast<int>(gy) * W_ + static_cast<int>(gx);

  if (s_idx == g_idx) {
    path.poses.push_back(goal);
    return path;
  }

  static constexpr float INF = std::numeric_limits<float>::infinity();
  std::vector<float> g_cost(N, INF);
  std::vector<int>   parent(N, -1);

  // 队列保留入射方向，后续才能按 JPS 规则剪枝邻居方向
  using Entry = std::tuple<float, int, int, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  static constexpr int DX8[8] = {-1, 0, 1, -1, 1, -1, 0,  1};
  static constexpr int DY8[8] = {-1,-1,-1,  0, 0,  1, 1,  1};

  // 起点没有入射方向，先向八个方向各跳一次作为搜索入口
  g_cost[s_idx] = 0.0f;
  for (int d = 0; d < 8; ++d) {
    const int jp = jump(static_cast<int>(sx), static_cast<int>(sy),
                        DX8[d], DY8[d], static_cast<int>(gx), static_cast<int>(gy), cm);
    if (jp < 0) continue;
    const int jpx = jp % W_, jpy = jp / W_;
    const float dist = std::sqrt(static_cast<float>((jpx - static_cast<int>(sx)) *
                                                     (jpx - static_cast<int>(sx)) +
                                                     (jpy - static_cast<int>(sy)) *
                                                     (jpy - static_cast<int>(sy))));
    if (dist < g_cost[jp]) {
      g_cost[jp] = dist;
      parent[jp] = s_idx;
      const float h = heuristic(jp, static_cast<int>(gx), static_cast<int>(gy), W_);
      open.push({dist + h, jp, DX8[d], DY8[d]});
    }
  }

  const auto t0 = std::chrono::high_resolution_clock::now();
  int nodes_expanded = 0;
  bool found = false;
  nav_msgs::msg::Path result;

  while (!open.empty()) {
    auto [f, cur, dx, dy] = open.top();
    open.pop();

    // 懒删除避免在优先队列中做昂贵的定点更新
    if (g_cost[cur] + 1e-5f < f - heuristic(cur, static_cast<int>(gx),
                                                    static_cast<int>(gy), W_)) {
      continue;
    }
    ++nodes_expanded;

    if (cur == g_idx) {
      result = buildJPSPath(parent, g_idx, cm, path.header.frame_id, path.header.stamp);
      found = true;
      break;
    }

    const int cx = cur % W_, cy = cur / W_;
    const float g = g_cost[cur];

    auto tryJump = [&](int ndx, int ndy) {
      const int jp = jump(cx, cy, ndx, ndy,
                          static_cast<int>(gx), static_cast<int>(gy), cm);
      if (jp < 0) return;
      const int jpx = jp % W_, jpy = jp / W_;
      const float dist = std::sqrt(static_cast<float>((jpx - cx) * (jpx - cx) +
                                                       (jpy - cy) * (jpy - cy)));
      const float ng = g + dist;
      if (ng < g_cost[jp]) {
        g_cost[jp] = ng;
        parent[jp] = cur;
        const float h = heuristic(jp, static_cast<int>(gx), static_cast<int>(gy), W_);
        open.push({ng + h, jp, ndx, ndy});
      }
    };

    // 只保留自然邻居和强迫邻居，减少与 A* 等价的重复展开
    if (dx != 0 && dy != 0) {
      tryJump(dx, 0);
      tryJump(0, dy);
      tryJump(dx, dy);
      if (blocked(cx - dx, cy, cm))     tryJump(-dx, dy);
      if (blocked(cx, cy - dy, cm))     tryJump(dx, -dy);
    } else if (dx != 0) {
      tryJump(dx, 0);
      if (blocked(cx, cy + 1, cm))  tryJump(dx, +1);
      if (blocked(cx, cy - 1, cm))  tryJump(dx, -1);
    } else {
      tryJump(0, dy);
      if (blocked(cx + 1, cy, cm))  tryJump(+1, dy);
      if (blocked(cx - 1, cy, cm))  tryJump(-1, dy);
    }
  }

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now() - t0).count();
  publishPlannerStats(stats_pub_, name_, ms, computePathLength(result), nodes_expanded, found);

  if (!found) {
    RCLCPP_WARN(logger_, "JPS: no path found from (%.2f,%.2f) to (%.2f,%.2f)",
                start.pose.position.x, start.pose.position.y,
                goal.pose.position.x, goal.pose.position.y);
    return path;
  }
  return result;
}

}

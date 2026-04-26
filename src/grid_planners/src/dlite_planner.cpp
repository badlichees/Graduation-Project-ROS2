#include "grid_planners/dlite_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "grid_planners/planner_stats.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(grid_planners::DLitePlanner, nav2_core::GlobalPlanner)

namespace grid_planners
{

// ── ODR definitions for static constexpr members (C++14 compat) ────────────
constexpr float DLitePlanner::INF;
constexpr int   DLitePlanner::DX[];
constexpr int   DLitePlanner::DY[];
constexpr float DLitePlanner::STEP[];

// ── Constructor ─────────────────────────────────────────────────────────────
DLitePlanner::DLitePlanner()
: logger_(rclcpp::get_logger("DLitePlanner")) {}

// ── Heuristic: octile distance (admissible for 8-connected grid) ────────────
float DLitePlanner::h(int a, int b) const
{
  const float dx = std::abs(static_cast<float>(a % W_) - static_cast<float>(b % W_));
  const float dy = std::abs(static_cast<float>(a / W_) - static_cast<float>(b / W_));
  return std::max(dx, dy) + (std::sqrt(2.0f) - 1.0f) * std::min(dx, dy);
}

// ── Edge cost: destination-based terrain penalty (matches A* implementation) ─
float DLitePlanner::edgeCost(int /*from*/, int to, int d,
                              nav2_costmap_2d::Costmap2D * cm) const
{
  const unsigned char cv = cm->getCost(to % W_, to / W_);
  if (cv >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    if (cv == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) {
      return STEP[d] + 0.5f;
    }
    return INF;
  }
  return STEP[d] + 0.01f * static_cast<float>(cv);
}

// ── D* Lite key: [min(g,rhs)+h(s_start,s)+km, min(g,rhs)] ──────────────────
DLitePlanner::Key DLitePlanner::calcKey(int s) const
{
  const float m = std::min(g_[s], rhs_[s]);
  return {m + h(s_start_, s) + km_, m};
}

// ── Priority-queue helpers ───────────────────────────────────────────────────
void DLitePlanner::pqClean()
{
  while (!pq_.empty() && pq_.top().ver != ver_[pq_.top().idx]) {
    pq_.pop();
  }
}

bool DLitePlanner::pqEmpty()
{
  pqClean();
  return pq_.empty();
}

DLitePlanner::Key DLitePlanner::pqTopKey()
{
  pqClean();
  return pq_.top().key;
}

int DLitePlanner::pqTopIdx()
{
  pqClean();
  return pq_.top().idx;
}

void DLitePlanner::pqPop()
{
  pqClean();
  if (!pq_.empty()) { pq_.pop(); }
}

void DLitePlanner::pqInsert(int idx, Key k)
{
  ++ver_[idx];
  pq_.push({k, idx, ver_[idx]});
}

// Logical removal: bumping the version invalidates any existing queue entry.
void DLitePlanner::pqRemove(int idx)
{
  ++ver_[idx];
}

// ── UpdateVertex: recompute rhs[u] then manage queue membership ─────────────
void DLitePlanner::updateVertex(int u, nav2_costmap_2d::Costmap2D * cm)
{
  if (u != s_goal_) {
    rhs_[u] = INF;
    const int ux = u % W_, uy = u / W_;
    for (int d = 0; d < 8; ++d) {
      const int vx = ux + DX[d], vy = uy + DY[d];
      if (vx < 0 || vy < 0 || vx >= W_ || vy >= H_) { continue; }
      const int v = vy * W_ + vx;
      const float c = edgeCost(u, v, d, cm);
      if (c < INF && g_[v] < INF) {
        rhs_[u] = std::min(rhs_[u], c + g_[v]);
      }
    }
  }
  pqRemove(u);
  if (g_[u] != rhs_[u]) {
    pqInsert(u, calcKey(u));
  }
}

// ── ComputeShortestPath: main D* Lite loop ───────────────────────────────────
//
// Invariant on exit: g[s_start] = rhs[s_start] = optimal cost (or INF if
// unreachable), and all cells on the optimal path are locally consistent.
void DLitePlanner::computeShortestPath(nav2_costmap_2d::Costmap2D * cm)
{
  const int max_iter = N_ * 5;
  int iter = 0;

  while (!pqEmpty()) {
    const Key top_key  = pqTopKey();
    const Key s_key    = calcKey(s_start_);

    // Termination: top key is not smaller than start's key AND start is consistent.
    if (!(top_key < s_key) && rhs_[s_start_] == g_[s_start_]) { break; }

    if (++iter > max_iter) {
      RCLCPP_WARN(logger_, "D* Lite: safety limit reached (%d iter)", max_iter);
      break;
    }

    const int  u     = pqTopIdx();
    const Key  k_new = calcKey(u);

    if (top_key < k_new) {
      // Key has gone stale (increased); re-insert with corrected key.
      pqPop();
      pqInsert(u, k_new);
    } else if (g_[u] > rhs_[u]) {
      // Overconsistent: lower g[u] to rhs[u] and propagate.
      g_[u] = rhs_[u];
      pqPop();
      ++expanded_count_;
      const int ux = u % W_, uy = u / W_;
      for (int d = 0; d < 8; ++d) {
        const int vx = ux + DX[d], vy = uy + DY[d];
        if (vx < 0 || vy < 0 || vx >= W_ || vy >= H_) { continue; }
        updateVertex(vy * W_ + vx, cm);
      }
    } else {
      // Underconsistent: raise g[u] to INF and propagate.
      g_[u] = INF;
      pqPop();
      ++expanded_count_;
      updateVertex(u, cm);
      const int ux = u % W_, uy = u / W_;
      for (int d = 0; d < 8; ++d) {
        const int vx = ux + DX[d], vy = uy + DY[d];
        if (vx < 0 || vy < 0 || vx >= W_ || vy >= H_) { continue; }
        updateVertex(vy * W_ + vx, cm);
      }
    }
  }
}

// ── Full initialisation (called when goal changes or on first use) ───────────
void DLitePlanner::initDLite(int goal_idx, nav2_costmap_2d::Costmap2D * cm)
{
  s_goal_ = goal_idx;
  km_     = 0.0f;
  pq_     = MinPQ{};
  g_.assign(N_,   INF);
  rhs_.assign(N_, INF);
  ver_.assign(N_, 0);

  rhs_[s_goal_] = 0.0f;
  pqInsert(s_goal_, calcKey(s_goal_));

  const unsigned char * data = cm->getCharMap();
  prev_cm_.assign(data, data + N_);
}

// ── configure ───────────────────────────────────────────────────────────────
void DLitePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_        = parent;
  name_        = name;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  allow_unknown_ = node->get_parameter(name_ + ".allow_unknown").as_bool();
  stats_pub_ = node->create_publisher<std_msgs::msg::String>("/planner_stats", rclcpp::QoS(10));

  RCLCPP_INFO(logger_, "DLitePlanner configured (allow_unknown=%s)",
              allow_unknown_ ? "true" : "false");
}

// ── createPlan ───────────────────────────────────────────────────────────────
nav_msgs::msg::Path DLitePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = goal.header.frame_id;
  path.header.stamp    = node_.lock()->now();

  RCLCPP_INFO(logger_, "createPlan called: (%.2f,%.2f) -> (%.2f,%.2f)",
              start.pose.position.x, start.pose.position.y,
              goal.pose.position.x,  goal.pose.position.y);

  auto * cm = costmap_ros_->getCostmap();
  const int W = static_cast<int>(cm->getSizeInCellsX());
  const int H = static_cast<int>(cm->getSizeInCellsY());
  const int N = W * H;

  unsigned int sx, sy, gx, gy;
  if (!cm->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
      !cm->worldToMap(goal.pose.position.x,  goal.pose.position.y,  gx, gy))
  {
    RCLCPP_WARN(logger_, "Start or goal out of costmap bounds");
    return path;
  }

  const int new_start = static_cast<int>(sy) * W + static_cast<int>(sx);
  const int new_goal  = static_cast<int>(gy) * W + static_cast<int>(gx);

  if (new_start == new_goal) {
    path.poses.push_back(goal);
    return path;
  }

  const auto t0 = std::chrono::high_resolution_clock::now();
  expanded_count_ = 0;

  const bool need_reinit = (W != W_ || N != N_ || new_goal != s_goal_ || s_goal_ == -1);

  if (need_reinit) {
    W_ = W;
    H_ = H;
    N_ = N;
    s_start_ = new_start;
    s_last_  = new_start;
    initDLite(new_goal, cm);
    RCLCPP_INFO(logger_, "D* Lite: full init (goal=%s)",
                (new_goal != s_goal_) ? "changed" : "first run");
  } else {
    // ── Incremental update ─────────────────────────────────────────────────
    // Account for the robot having moved since the last call.
    km_     += h(s_last_, new_start);
    s_last_  = new_start;
    s_start_ = new_start;

    // Find cells whose costmap cost changed and propagate.
    const unsigned char * cur_data = cm->getCharMap();
    int changes = 0;
    for (int i = 0; i < N_; ++i) {
      if (cur_data[i] != prev_cm_[i]) {
        ++changes;
        updateVertex(i, cm);
        const int ix = i % W_, iy = i / W_;
        for (int d = 0; d < 8; ++d) {
          const int nx = ix + DX[d], ny = iy + DY[d];
          if (nx >= 0 && ny >= 0 && nx < W_ && ny < H_) {
            updateVertex(ny * W_ + nx, cm);
          }
        }
      }
    }
    std::copy(cur_data, cur_data + N_, prev_cm_.begin());

    if (changes > 0) {
      RCLCPP_INFO(logger_, "D* Lite: %d changed costmap cells, replanning", changes);
    }
  }

  computeShortestPath(cm);

  if (g_[s_start_] >= INF) {
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - t0).count();
    publishPlannerStats(stats_pub_, name_, ms, 0.0, expanded_count_, false);
    RCLCPP_WARN(logger_, "D* Lite: no path from (%.2f,%.2f) to (%.2f,%.2f)",
                start.pose.position.x, start.pose.position.y,
                goal.pose.position.x,  goal.pose.position.y);
    return path;
  }

  auto result = extractPath(cm, path.header.frame_id, path.header.stamp);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now() - t0).count();
  publishPlannerStats(stats_pub_, name_, ms, computePathLength(result), expanded_count_, true);
  return result;
}

// ── extractPath: greedy descent along g-gradient toward s_goal_ ─────────────
nav_msgs::msg::Path DLitePlanner::extractPath(
  nav2_costmap_2d::Costmap2D * cm,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp    = stamp;

  std::vector<bool> visited(N_, false);
  std::vector<int>  indices;
  indices.reserve(static_cast<size_t>(std::max(W_, H_)) * 4);

  int cur = s_start_;
  indices.push_back(cur);
  visited[cur] = true;

  while (cur != s_goal_) {
    const int cx = cur % W_, cy = cur / W_;
    int   best      = -1;
    float best_cost = INF;

    for (int d = 0; d < 8; ++d) {
      const int nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= W_ || ny >= H_) { continue; }
      const int n = ny * W_ + nx;
      const float c = edgeCost(cur, n, d, cm);
      if (c >= INF || g_[n] >= INF) { continue; }
      const float total = c + g_[n];
      if (total < best_cost) {
        best_cost = total;
        best      = n;
      }
    }

    if (best == -1) {
      RCLCPP_WARN(logger_, "D* Lite: path extraction blocked at cell %d", cur);
      return nav_msgs::msg::Path{};
    }
    if (visited[best]) {
      RCLCPP_WARN(logger_, "D* Lite: cycle detected during path extraction");
      return nav_msgs::msg::Path{};
    }

    indices.push_back(best);
    visited[best] = true;
    cur = best;
  }

  path.poses.reserve(indices.size());
  for (int idx : indices) {
    double wx, wy;
    cm->mapToWorld(idx % W_, idx / W_, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header            = path.header;
    pose.pose.position.x   = wx;
    pose.pose.position.y   = wy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace grid_planners

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

// 保留类内 constexpr 数组的定义，兼容较旧的编译模式
constexpr float DLitePlanner::INF;
constexpr int DLitePlanner::DX[];
constexpr int DLitePlanner::DY[];
constexpr float DLitePlanner::STEP[];

DLitePlanner::DLitePlanner()
: logger_(rclcpp::get_logger("DLitePlanner")) {}

// 八邻接启发式必须低估真实代价，D* Lite 的一致性依赖这一点
float DLitePlanner::h(int a, int b) const
{
  const float dx = std::abs(static_cast<float>(a % W_) - static_cast<float>(b % W_));
  const float dy = std::abs(static_cast<float>(a / W_) - static_cast<float>(b / W_));
  return std::max(dx, dy) + (std::sqrt(2.0f) - 1.0f) * std::min(dx, dy);
}

// 与 A* 使用同一套代价偏好，便于横向比较不同规划器
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

// km 把起点移动折算进启发式，避免每次重规划都从零开始
DLitePlanner::Key DLitePlanner::calcKey(int s) const
{
  const float m = std::min(g_[s], rhs_[s]);
  return {m + h(s_start_, s) + km_, m};
}

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

// 版本号实现逻辑删除，绕开标准优先队列不能原地降低优先级的限制
void DLitePlanner::pqRemove(int idx)
{
  ++ver_[idx];
}

// rhs 表示一步前瞻代价，只有 g 与 rhs 不一致的节点才需要进入队列
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

// 循环结束时起点局部一致，当前最优路径上的代价已经被传播到位
void DLitePlanner::computeShortestPath(nav2_costmap_2d::Costmap2D * cm)
{
  const int max_iter = N_ * 5;
  int iter = 0;

  while (!pqEmpty()) {
    const Key top_key  = pqTopKey();
    const Key s_key    = calcKey(s_start_);

    // 队首不再优于起点且起点一致时，继续展开不会改善当前路径
    if (!(top_key < s_key) && rhs_[s_start_] == g_[s_start_]) { break; }

    if (++iter > max_iter) {
      RCLCPP_WARN(logger_, "D* Lite: safety limit reached (%d iter)", max_iter);
      break;
    }

    const int  u     = pqTopIdx();
    const Key  k_new = calcKey(u);

    if (top_key < k_new) {
      // 地图或起点变化后旧 key 可能失效，重新入队即可
      pqPop();
      pqInsert(u, k_new);
    } else if (g_[u] > rhs_[u]) {
      // 代价变好时把 g 下调，并向前驱传播收益
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
      // 代价变差时先抬高 g，再让邻域重新寻找替代路径
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

// 目标变化会让反向搜索树失效，此时完整重建比修补更清晰
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
    // 起点移动只需要更新 km，保留已有反向搜索结果
    km_     += h(s_last_, new_start);
    s_last_  = new_start;
    s_start_ = new_start;

    // 只把变化栅格及其邻域重新入队，D* Lite 的优势在这里体现
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
    prev_cm_.assign(cur_data, cur_data + N_);

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

// 从起点沿 g 梯度贪心下降，局部一致性保证这条链指向目标
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

}

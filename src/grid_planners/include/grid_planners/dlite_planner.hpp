#pragma once

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace grid_planners
{

// D* Lite (Koenig & Likhachev 2002) incremental backward-search planner.
// On the first call or when the goal changes, runs a full backward Dijkstra from
// the goal.  On subsequent calls with the same goal, only cells whose costmap
// values changed (and their 8-neighbors) are re-queued, so replanning is O(changes).
class DLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DLitePlanner();
  ~DLitePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override {}
  void activate()   override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Lexicographic key used by the D* Lite open list.
  struct Key {
    float k1{0.0f}, k2{0.0f};
    bool operator<(const Key & o) const {
      return k1 < o.k1 || (k1 == o.k1 && k2 < o.k2);
    }
    bool operator>(const Key & o) const { return o < *this; }
    bool operator<=(const Key & o) const { return !(o < *this); }
  };

  // Priority-queue entry; stale entries are skipped via version check.
  struct QEntry {
    Key  key;
    int  idx{-1};
    int  ver{0};
    bool operator>(const QEntry & o) const { return key > o.key; }
  };

  using MinPQ = std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>>;

  // Octile-distance heuristic (admissible for 8-connected grid).
  float h(int a, int b) const;

  // Cost of moving from cell `from` to adjacent cell `to` in direction `d`.
  // Returns INF when `to` is a lethal obstacle (or unknown if !allow_unknown_).
  float edgeCost(int from, int to, int d, nav2_costmap_2d::Costmap2D * cm) const;

  // D* Lite key for cell s: [min(g,rhs)+h(s_start,s)+km, min(g,rhs)].
  Key calcKey(int s) const;

  // Priority-queue helpers with lazy-deletion via per-cell version counters.
  void  pqClean();
  bool  pqEmpty();
  Key   pqTopKey();
  int   pqTopIdx();
  void  pqPop();
  void  pqInsert(int idx, Key k);
  void  pqRemove(int idx);   // logical removal: increment version

  // Core D* Lite operations.
  void initDLite(int goal_idx, nav2_costmap_2d::Costmap2D * cm);
  void updateVertex(int u, nav2_costmap_2d::Costmap2D * cm);
  void computeShortestPath(nav2_costmap_2d::Costmap2D * cm);

  // Walk g-gradient from s_start_ to s_goal_ and build the path message.
  nav_msgs::msg::Path extractPath(
    nav2_costmap_2d::Costmap2D * cm,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  // ── D* Lite persistent state ─────────────────────────────────────────────
  std::vector<float> g_;    // backward cost estimate: s → s_goal
  std::vector<float> rhs_;  // one-step lookahead value
  std::vector<int>   ver_;  // per-cell version for lazy deletion
  MinPQ pq_;

  float km_{0.0f};   // accumulated heuristic correction for start movement
  int s_start_{-1};
  int s_goal_{-1};
  int s_last_{-1};   // start from the previous call (for km accumulation)
  int W_{0};         // costmap width  (cells)
  int H_{0};         // costmap height (cells)
  int N_{0};         // W_ * H_

  // Costmap snapshot for change detection.
  std::vector<unsigned char> prev_cm_;

  // ── ROS / Nav2 bookkeeping ────────────────────────────────────────────────
  bool allow_unknown_{true};
  rclcpp_lifecycle::LifecycleNode::WeakPtr        node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>  costmap_ros_;
  std::string  name_;
  rclcpp::Logger logger_;

  static constexpr float INF     = std::numeric_limits<float>::infinity();
  static constexpr int   DX[8]   = {-1,  0,  1, -1,  1, -1,  0,  1};
  static constexpr int   DY[8]   = {-1, -1, -1,  0,  0,  1,  1,  1};
  static constexpr float STEP[8] = {1.414f, 1.0f, 1.414f, 1.0f,
                                     1.0f, 1.414f, 1.0f, 1.414f};
};

}  // namespace grid_planners

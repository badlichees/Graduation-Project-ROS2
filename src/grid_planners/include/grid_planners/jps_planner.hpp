#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// Jump Point Search planner.
// Inherits configure/buildPath/heuristic from AStarPlanner.
// Overrides createPlan with JPS-accelerated A* that jumps over
// symmetric, prunable nodes instead of expanding every neighbor.
class JPSPlanner : public AStarPlanner
{
public:
  explicit JPSPlanner() : AStarPlanner("JPSPlanner") {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Returns grid index of the next jump point when jumping from (x,y)
  // in direction (dx,dy) toward goal (gx,gy), or -1 if none found.
  int jump(int x, int y, int dx, int dy, int gx, int gy,
           nav2_costmap_2d::Costmap2D * cm) const;

  // True if (x,y) is out-of-bounds or is an impassable obstacle cell.
  bool blocked(int x, int y, nav2_costmap_2d::Costmap2D * cm) const;

  // Build the full path by interpolating between consecutive jump points.
  nav_msgs::msg::Path buildJPSPath(
    const std::vector<int> & parent,
    int goal_idx,
    nav2_costmap_2d::Costmap2D * cm,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

  // Grid dimensions; set at the start of each createPlan call.
  mutable int W_{0}, H_{0};
};

}  // namespace grid_planners

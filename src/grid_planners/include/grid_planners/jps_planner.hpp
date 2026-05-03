#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// JPS 复用 A* 的代价模型，只改变邻居生成方式来消除对称展开
class JPSPlanner : public AStarPlanner
{
public:
  explicit JPSPlanner() : AStarPlanner("JPSPlanner") {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // 跳跃方向本身携带剪枝上下文
  int jump(int x, int y, int dx, int dy, int gx, int gy,
           nav2_costmap_2d::Costmap2D * cm) const;

  bool blocked(int x, int y, nav2_costmap_2d::Costmap2D * cm) const;

  // 跳点路径需要补成 Nav2 控制器更容易消费的连续路径
  nav_msgs::msg::Path buildJPSPath(
    const std::vector<int> & parent,
    int goal_idx,
    nav2_costmap_2d::Costmap2D * cm,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

  mutable int W_{0}, H_{0};
};

}

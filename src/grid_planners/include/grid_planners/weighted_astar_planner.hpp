#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// 用启发式权重换速度，w 越大越贪心，最优性让位给响应时间
class WeightedAStarPlanner : public AStarPlanner
{
public:
  WeightedAStarPlanner() : AStarPlanner("WeightedAStarPlanner") {}
  ~WeightedAStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

protected:
  float priority(float g, float h) const override { return g + w_ * h; }

private:
  float w_{2.0f};
};

}

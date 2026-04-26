#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// Weighted A*: f = g + w * h
//
// w = 1  → standard A* (optimal)
// w > 1  → suboptimal but faster; path cost ≤ w * optimal (ε-admissible)
// w → ∞  → degenerates to GBFS
//
// Configure via yaml:  weight: 2.0  (default)
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

}  // namespace grid_planners

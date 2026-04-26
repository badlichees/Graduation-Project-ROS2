#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace grid_planners
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  explicit AStarPlanner(const std::string & logger_name = "AStarPlanner")
  : logger_(rclcpp::get_logger(logger_name)) {}

  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // Returns heuristic estimate from cell `idx` to goal (gx, gy) on a grid of width W.
  virtual float heuristic(int idx, int gx, int gy, int W) const;

  // Returns priority (f-value) for the open-list heap.
  // Default: f = g + h  (A*).  Override for other search strategies.
  virtual float priority(float g, float h) const { return g + h; }

  nav_msgs::msg::Path buildPath(
    const std::vector<int> & parent,
    int goal_idx,
    nav2_costmap_2d::Costmap2D * costmap,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  bool allow_unknown_{true};
  rclcpp::Logger logger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
};

}  // namespace grid_planners

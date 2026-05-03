#include "grid_planners/weighted_astar_planner.hpp"

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(grid_planners::WeightedAStarPlanner, nav2_core::GlobalPlanner)

namespace grid_planners
{

void WeightedAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  AStarPlanner::configure(parent, name, tf, costmap_ros);

  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".weight", rclcpp::ParameterValue(2.0));
  w_ = static_cast<float>(node->get_parameter(name_ + ".weight").as_double());

  RCLCPP_INFO(logger_, "WeightedAStarPlanner configured (w=%.2f)", w_);
}

}

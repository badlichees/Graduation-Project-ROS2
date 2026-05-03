#pragma once

#include <memory>
#include <random>
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

class RRTStarPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTStarPlanner() : logger_(rclcpp::get_logger("RRTStarPlanner")) {}
  ~RRTStarPlanner() override = default;

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

private:
  struct Node {
    double x, y;
    int parent;
    double cost;
  };

  int nearestNode(const std::vector<Node> & nodes, double x, double y) const;
  std::vector<int> nearNodes(const std::vector<Node> & nodes, double x, double y) const;
  bool isCollisionFree(double x1, double y1, double x2, double y2) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_;

  int max_iterations_;
  double step_size_;
  double search_radius_;
  double goal_tolerance_;
  double goal_bias_;
  bool allow_unknown_;

  mutable std::mt19937 rng_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
};

}

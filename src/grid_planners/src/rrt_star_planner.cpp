#include "grid_planners/rrt_star_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "grid_planners/planner_stats.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(grid_planners::RRTStarPlanner, nav2_core::GlobalPlanner)

namespace grid_planners
{

void RRTStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(5000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step_size", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".search_radius", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_bias", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));

  max_iterations_ = node->get_parameter(name_ + ".max_iterations").as_int();
  step_size_      = node->get_parameter(name_ + ".step_size").as_double();
  search_radius_  = node->get_parameter(name_ + ".search_radius").as_double();
  goal_tolerance_ = node->get_parameter(name_ + ".goal_tolerance").as_double();
  goal_bias_      = node->get_parameter(name_ + ".goal_bias").as_double();
  allow_unknown_  = node->get_parameter(name_ + ".allow_unknown").as_bool();
  stats_pub_ = node->create_publisher<std_msgs::msg::String>("/planner_stats", rclcpp::QoS(10));

  rng_.seed(std::random_device{}());

  RCLCPP_INFO(logger_,
    "RRTStarPlanner configured (max_iter=%d, step=%.2f, radius=%.2f, goal_tol=%.2f, bias=%.2f)",
    max_iterations_, step_size_, search_radius_, goal_tolerance_, goal_bias_);
}

nav_msgs::msg::Path RRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = goal.header.frame_id;
  path.header.stamp = node_.lock()->now();

  RCLCPP_INFO(logger_, "RRT* createPlan: (%.2f,%.2f) -> (%.2f,%.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x,  goal.pose.position.y);

  auto * cm = costmap_ros_->getCostmap();
  const double ox  = cm->getOriginX();
  const double oy  = cm->getOriginY();
  const double res = cm->getResolution();
  const double max_x = ox + cm->getSizeInCellsX() * res;
  const double max_y = oy + cm->getSizeInCellsY() * res;

  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  // Validate start / goal
  unsigned int mx, my;
  if (!cm->worldToMap(sx, sy, mx, my) || !cm->worldToMap(gx, gy, mx, my)) {
    RCLCPP_WARN(logger_, "Start or goal out of costmap bounds");
    return path;
  }
  cm->worldToMap(gx, gy, mx, my);
  if (cm->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_WARN(logger_, "Goal is inside an obstacle");
    return path;
  }

  // Trivial case
  const double start_goal_dist = std::hypot(gx - sx, gy - sy);
  if (start_goal_dist < goal_tolerance_) {
    path.poses.push_back(start);
    path.poses.push_back(goal);
    return path;
  }

  // RRT* main loop
  std::vector<Node> nodes;
  nodes.reserve(static_cast<size_t>(max_iterations_) + 1);
  nodes.push_back({sx, sy, -1, 0.0});

  std::uniform_real_distribution<double> dist_x(ox, max_x);
  std::uniform_real_distribution<double> dist_y(oy, max_y);
  std::uniform_real_distribution<double> dist_01(0.0, 1.0);

  int goal_node_idx = -1;
  const auto t0 = std::chrono::high_resolution_clock::now();

  for (int iter = 0; iter < max_iterations_; ++iter) {
    // Sample
    double rx, ry;
    if (dist_01(rng_) < goal_bias_) {
      rx = gx; ry = gy;
    } else {
      rx = dist_x(rng_);
      ry = dist_y(rng_);
    }

    // Nearest node
    const int near_idx = nearestNode(nodes, rx, ry);
    const double nx = nodes[near_idx].x;
    const double ny = nodes[near_idx].y;

    // Steer
    const double dx = rx - nx;
    const double dy = ry - ny;
    const double d  = std::hypot(dx, dy);
    double new_x, new_y;
    if (d <= step_size_) {
      new_x = rx; new_y = ry;
    } else {
      new_x = nx + (dx / d) * step_size_;
      new_y = ny + (dy / d) * step_size_;
    }

    // Bounds check
    if (new_x < ox || new_x >= max_x || new_y < oy || new_y >= max_y) continue;

    // Obstacle check at new node
    unsigned int nmx, nmy;
    if (!cm->worldToMap(new_x, new_y, nmx, nmy)) continue;
    const auto cell_cost = cm->getCost(nmx, nmy);
    if (cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      if (!(cell_cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_)) continue;
    }

    // Edge collision check
    if (!isCollisionFree(nx, ny, new_x, new_y)) continue;

    // Near nodes for parent selection and rewiring
    const auto near_ids = nearNodes(nodes, new_x, new_y);

    // Choose best parent
    int best_parent = near_idx;
    double best_cost = nodes[near_idx].cost + std::hypot(new_x - nx, new_y - ny);
    for (int nid : near_ids) {
      const double c = nodes[nid].cost + std::hypot(new_x - nodes[nid].x, new_y - nodes[nid].y);
      if (c < best_cost && isCollisionFree(nodes[nid].x, nodes[nid].y, new_x, new_y)) {
        best_cost = c;
        best_parent = nid;
      }
    }

    // Add new node
    const int new_idx = static_cast<int>(nodes.size());
    nodes.push_back({new_x, new_y, best_parent, best_cost});

    // Rewire
    for (int nid : near_ids) {
      const double new_cost =
        best_cost + std::hypot(new_x - nodes[nid].x, new_y - nodes[nid].y);
      if (new_cost < nodes[nid].cost &&
          isCollisionFree(new_x, new_y, nodes[nid].x, nodes[nid].y))
      {
        nodes[nid].parent = new_idx;
        nodes[nid].cost   = new_cost;
      }
    }

    // Check goal reached
    const double dist_to_goal = std::hypot(new_x - gx, new_y - gy);
    if (dist_to_goal < goal_tolerance_) {
      if (goal_node_idx == -1 || best_cost < nodes[goal_node_idx].cost) {
        goal_node_idx = new_idx;
      }
    }
  }

  if (goal_node_idx == -1) {
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - t0).count();
    publishPlannerStats(stats_pub_, name_, ms, 0.0,
                        static_cast<int>(nodes.size()), false);
    RCLCPP_WARN(logger_, "RRT* failed to reach goal after %d iterations", max_iterations_);
    return path;
  }

  // Trace path from goal node back to root
  std::vector<std::pair<double, double>> waypoints;
  for (int idx = goal_node_idx; idx != -1; idx = nodes[idx].parent) {
    waypoints.push_back({nodes[idx].x, nodes[idx].y});
  }
  std::reverse(waypoints.begin(), waypoints.end());
  waypoints.push_back({gx, gy});

  // Interpolate between waypoints at costmap resolution for smooth following
  path.poses.reserve(waypoints.size() * static_cast<size_t>(step_size_ / res + 1));
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const double x1 = waypoints[i].first,  y1 = waypoints[i].second;
    const double x2 = waypoints[i+1].first, y2 = waypoints[i+1].second;
    const double seg = std::hypot(x2 - x1, y2 - y1);
    const int n_pts  = std::max(1, static_cast<int>(seg / res));
    for (int j = 0; j < n_pts; ++j) {
      const double t = static_cast<double>(j) / n_pts;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x1 + t * (x2 - x1);
      pose.pose.position.y = y1 + t * (y2 - y1);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }
  // Final point
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = gx;
    pose.pose.position.y = gy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now() - t0).count();
  publishPlannerStats(stats_pub_, name_, ms, computePathLength(path),
                      static_cast<int>(nodes.size()), true);
  RCLCPP_INFO(logger_, "RRT* found path: %zu waypoints -> %zu interpolated poses",
    waypoints.size(), path.poses.size());
  return path;
}

int RRTStarPlanner::nearestNode(const std::vector<Node> & nodes, double x, double y) const
{
  int best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const double d2 = (nodes[i].x - x) * (nodes[i].x - x) +
                      (nodes[i].y - y) * (nodes[i].y - y);
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }
  return best;
}

std::vector<int> RRTStarPlanner::nearNodes(
  const std::vector<Node> & nodes, double x, double y) const
{
  std::vector<int> result;
  const double r2 = search_radius_ * search_radius_;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const double d2 = (nodes[i].x - x) * (nodes[i].x - x) +
                      (nodes[i].y - y) * (nodes[i].y - y);
    if (d2 <= r2) result.push_back(i);
  }
  return result;
}

bool RRTStarPlanner::isCollisionFree(double x1, double y1, double x2, double y2) const
{
  auto * cm = costmap_ros_->getCostmap();
  const double res = cm->getResolution();
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double dist = std::hypot(dx, dy);
  const int steps = std::max(1, static_cast<int>(dist / (res * 0.5)));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double cx = x1 + t * dx;
    const double cy = y1 + t * dy;
    unsigned int mx, my;
    if (!cm->worldToMap(cx, cy, mx, my)) return false;
    const auto cost = cm->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      if (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) continue;
      return false;
    }
  }
  return true;
}

}  // namespace grid_planners

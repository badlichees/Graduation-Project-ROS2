#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace grid_planners
{

inline double computePathLength(const nav_msgs::msg::Path & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    len += std::hypot(b.x - a.x, b.y - a.y);
  }
  return len;
}

inline void publishPlannerStats(
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub,
  const std::string & algorithm,
  double plan_time_ms,
  double path_length_m,
  int nodes_expanded,
  bool path_found)
{
  if (!pub) return;
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"algorithm\":\"%s\",\"plan_time_ms\":%.2f,"
    "\"path_length_m\":%.3f,\"nodes_expanded\":%d,\"path_found\":%s}",
    algorithm.c_str(), plan_time_ms, path_length_m, nodes_expanded,
    path_found ? "true" : "false");
  std_msgs::msg::String msg;
  msg.data = buf;
  pub->publish(msg);
}

}

cmake_minimum_required(VERSION 3.8)
project(grid_planners)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(grid_planners SHARED
  src/astar_planner.cpp
  src/dijkstra_planner.cpp
  src/gbfs_planner.cpp
)

target_compile_features(grid_planners PUBLIC cxx_std_17)
target_compile_options(grid_planners PRIVATE -std=c++17)

target_include_directories(grid_planners PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(grid_planners
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS grid_planners DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_package()

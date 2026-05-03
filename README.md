# TurtleBot3 Unity-ROS2 导航后端

## 1. 概述

本工作区是毕业设计的 ROS2 导航后端，用于接收 Unity 仿真端发布的地图、传感器、里程计和目标点，并通过 Nav2 调用不同全局路径规划算法完成导航实验。

ROS2 侧负责：

- 启动 ROS TCP Endpoint，接入 Unity topic。
- 将 Unity 发布的 `/map_raw` 转发为 Nav2 可用的 `/map`。
- 将 Unity 发布的 `/odom` 转换为 TF。
- 启动 Nav2 导航栈。
- 注册并运行自定义全局规划器插件。
- 接收 Unity 的算法选择并转发给 Nav2 PlannerSelector。
- 发布规划统计数据供 Unity 面板显示和导出。

## 2. 架构

```txt
Unity
  ├─ /map_raw
  ├─ /odom
  ├─ /scan
  ├─ /goal_pose
  └─ /planner_selector_unity
        │
        ▼
ros_tcp_endpoint :10000
        │
        ▼
tb3_unity_nav
  ├─ map_relay: /map_raw -> /map
  ├─ odom_to_tf: /odom -> odom TF
  ├─ planner_selector_relay: /planner_selector_unity -> /planner_selector
  ├─ unity_nav2.launch.py
  └─ unity_nav2.yaml
        │
        ▼
Nav2
  ├─ PlannerSelector 行为树
  ├─ controller_server
  ├─ global_costmap / local_costmap
  └─ planner_server
        │
        ▼
grid_planners
  ├─ Astar
  ├─ Dijkstra
  ├─ Greedy
  ├─ RRTStar
  ├─ DLite
  ├─ JPS
  └─ WAStar
```

主要 topic：

| Topic | 方向 | 类型 | 用途 |
|---|---|---|---|
| `/map_raw` | Unity -> ROS2 | `nav_msgs/OccupancyGrid` | Unity 原始地图 |
| `/map` | ROS2 -> Nav2 | `nav_msgs/OccupancyGrid` | Nav2 使用的地图 |
| `/odom` | Unity -> ROS2 | `nav_msgs/Odometry` | 机器人里程计 |
| `/scan` | Unity -> ROS2 | `sensor_msgs/LaserScan` | 局部避障输入 |
| `/goal_pose` | Unity -> ROS2 | `geometry_msgs/PoseStamped` | Nav2 目标点 |
| `/planner_selector_unity` | Unity -> ROS2 | `std_msgs/String` | Unity 算法选择 |
| `/planner_selector` | ROS2 -> Nav2 | `std_msgs/String` | Nav2 PlannerSelector 输入 |
| `/plan` | Nav2 -> Unity | `nav_msgs/Path` | 全局路径 |
| `/planner_stats` | grid_planners -> Unity | `std_msgs/String` | 规划性能统计 |
| `/cmd_vel` | Nav2 -> Unity | `geometry_msgs/Twist` | 机器人速度控制 |

TF 关系：

```txt
map -> odom              静态 TF
odom -> base_footprint   由 odom_to_tf 根据 /odom 发布
```

## 3. 依赖

系统环境：

```txt
Ubuntu 22.04 / WSL2
ROS2 Humble
Nav2
TurtleBot3 ROS2 packages
Unity ROS TCP Endpoint
```

工作区内自定义包：

| 包 | 类型 | 说明 |
|---|---|---|
| `tb3_unity_nav` | `ament_python` | Unity/Nav2 联调启动与中继节点 |
| `grid_planners` | `ament_cmake` | Nav2 全局规划器插件 |

`tb3_unity_nav` 主要运行依赖：

```txt
rclpy, tf2_ros, nav_msgs, geometry_msgs, std_msgs,
ros_tcp_endpoint, robot_state_publisher, rviz2,
turtlebot3_bringup, turtlebot3_navigation2,
nav2_bringup, nav2_bt_navigator, nav2_controller,
nav2_costmap_2d, nav2_navfn_planner, nav2_behaviors,
nav2_waypoint_follower, dwb_core, grid_planners
```

`grid_planners` 主要依赖：

```txt
rclcpp, rclcpp_lifecycle, nav2_core, nav2_costmap_2d,
nav2_util, nav_msgs, geometry_msgs, pluginlib, tf2_ros, std_msgs
```

## 4. 模块介绍

### 4.1 tb3_unity_nav

| 文件 | 说明 |
|---|---|
| `launch/unity_nav2.launch.py` | 启动完整 Unity/Nav2 联调栈 |
| `launch/unity_sim.launch.py` | 启动 TurtleBot3 状态发布和 odom TF 节点 |
| `param/unity_nav2.yaml` | Nav2 参数、规划器插件、costmap、controller 配置 |
| `behavior_trees/navigate_w_planner_selector.xml` | 支持运行时切换 planner 的 Nav2 行为树 |
| `tb3_unity_nav/map_relay.py` | 将 `/map_raw` 转发为带 transient_local QoS 的 `/map` |
| `tb3_unity_nav/odom_to_tf.py` | 将 `/odom` 补成 TF |
| `tb3_unity_nav/planner_selector_relay.py` | 将 Unity 算法选择转发为 Nav2 PlannerSelector 输入 |

### 4.2 grid_planners

| 插件 ID | 实现 | 说明 |
|---|---|---|
| `Astar` | `grid_planners/AStarPlanner` | 八邻接 A* 搜索 |
| `Dijkstra` | `grid_planners/DijkstraPlanner` | 零启发式的最短路基线 |
| `Greedy` | `grid_planners/GBFSPlanner` | 只按启发式扩展的贪心搜索 |
| `RRTStar` | `grid_planners/RRTStarPlanner` | 采样式 RRT* 规划 |
| `DLite` | `grid_planners/DLitePlanner` | 面向变化地图的 D* Lite |
| `JPS` | `grid_planners/JPSPlanner` | Jump Point Search |
| `WAStar` | `grid_planners/WeightedAStarPlanner` | Weighted A* |
| `NavFn` | `nav2_navfn_planner/NavfnPlanner` | Nav2 内置规划器对照组 |

自定义规划器会向 `/planner_stats` 发布统一格式的统计数据，Unity 端负责接收和展示。

## 5. 启动、运行与具体操作

### 5.1 编译

```bash
cd /home/$USERNAME/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select grid_planners tb3_unity_nav
source install/setup.bash
```

### 5.2 启动完整联调栈

```bash
cd /home/$USERNAME/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_unity_nav unity_nav2.launch.py
```

该 launch 会启动：

- `ros_tcp_endpoint`，监听 `0.0.0.0:10000`。
- TurtleBot3 `robot_state_publisher`。
- `odom_to_tf`。
- `map_relay`。
- `planner_selector_relay`。
- Nav2 navigation stack。
- RViz2。

### 5.3 单独启动 Unity 仿真辅助栈

如只需要机器人状态发布和 odom TF：

```bash
ros2 launch tb3_unity_nav unity_sim.launch.py
```

### 5.4 常用检查命令

```bash
ros2 node list
ros2 topic list
ros2 topic echo /planner_selector
ros2 topic echo /planner_stats
ros2 topic echo /cmd_vel
ros2 topic hz /map
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

### 5.5 与 Unity 配合操作

1. 先启动 ROS2：`ros2 launch tb3_unity_nav unity_nav2.launch.py`。
2. 再打开 Unity 场景 `Assets/Scenes/TurtleBot3.unity`。
3. 点击 Play。
4. 在 Unity 中按 `Tab` 切换算法。
5. 在 Unity 中按 `Space` 发布目标点。
6. 观察 RViz、Unity 路径显示和 Unity 统计面板。

## 6. 实验流程

1. 编译 `grid_planners` 和 `tb3_unity_nav`。
2. 启动 `unity_nav2.launch.py`。
3. 启动 Unity 场景并进入 Play Mode。
4. 固定地图参数和目标点。
5. 选择一个规划算法并发布目标。
6. 等待 `/plan` 和 `/planner_stats` 更新。
7. 对每个算法重复相同目标点测试。
8. 在 Unity 统计面板导出 CSV。
9. 对比规划耗时、路径长度、节点展开数和成功率。

建议实验记录表：

| 字段 | 说明 |
|---|---|
| 地图尺寸 | Unity `MapGenerator.mapSize` |
| 障碍物比例 | Unity `MapGenerator.obstaclePercent` |
| 地图种子 | Unity `MapGenerator.seed` |
| 起点 | 机器人初始位置 |
| 目标点 | Unity `GoalPublisher.targetPositionXZ` |
| 算法 | `/planner_selector` 当前值 |
| 是否成功 | `/planner_stats.path_found` |
| 规划耗时 | `/planner_stats.plan_time_ms` |
| 路径长度 | `/planner_stats.path_length_m` |
| 节点展开数 | `/planner_stats.nodes_expanded` |

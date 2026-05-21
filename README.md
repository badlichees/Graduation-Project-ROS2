# TurtleBot3 Unity-ROS2 导航后端

ROS2 导航后端，接收 Unity 仿真端发布的地图、传感器、里程计和目标点，通过 Nav2 调用不同全局路径规划算法完成导航实验。

## 架构

```txt
Unity
  ├─ /map_raw
  ├─ /odom
  ├─ /scan
  ├─ /goal_pose
  ├─ /planner_selector_unity
  └─ /planner_param_updates
        │
        ▼
ros_tcp_endpoint :10000
        │
        ▼
unity_nav2
  ├─ map_bridge:       /map_raw -> /map
  ├─ odom_tf_bridge:   /odom -> odom TF
  ├─ planner_switch:   /planner_selector_unity -> /planner_selector
  ├─ param_bridge:     /planner_param_updates -> Nav2 set_parameters
  ├─ unity_nav2.launch.py
  └─ unity_nav2.yaml
        │
        ▼
Nav2
  ├─ PlannerSelector 行为树 (navigate.xml)
  ├─ controller_server
  ├─ global_costmap / local_costmap
  └─ planner_server
        │
        ▼
grid_planners
  ├─ Astar / Dijkstra / Greedy
  ├─ RRTStar / DLite / JPS / WAStar
  └─ (NavFn — Nav2 内置对照组)
```

主要 topic：

| Topic | 方向 | 类型 | 用途 |
|---|---|---|---|
| `/map_raw` | Unity→ROS2 | `nav_msgs/OccupancyGrid` | Unity 原始地图 |
| `/map` | ROS2→Nav2 | `nav_msgs/OccupancyGrid` | Nav2 使用的地图 |
| `/odom` | Unity→ROS2 | `nav_msgs/Odometry` | 机器人里程计 |
| `/scan` | Unity→ROS2 | `sensor_msgs/LaserScan` | 局部避障输入 |
| `/goal_pose` | Unity→ROS2 | `geometry_msgs/PoseStamped` | Nav2 目标点 |
| `/planner_selector_unity` | Unity→ROS2 | `std_msgs/String` | 算法选择（Unity侧） |
| `/planner_selector` | ROS2→Nav2 | `std_msgs/String` | PlannerSelector 输入 |
| `/planner_param_updates` | Unity→ROS2 | `std_msgs/String` (JSON) | 规划器参数热更新 |
| `/plan` | Nav2→Unity | `nav_msgs/Path` | 全局路径 |
| `/planner_stats` | grid_planners→Unity | `std_msgs/String` | 规划性能统计 |
| `/cmd_vel` | Nav2→Unity | `geometry_msgs/Twist` | 机器人速度控制 |

TF：`map`(静态)→`odom`→`base_footprint`（由 `odom_tf_bridge` 根据 `/odom` 发布）

## 依赖

```
Ubuntu 22.04 / WSL2 · ROS2 Humble · Nav2 · TurtleBot3 ROS2 packages · ros_tcp_endpoint
```

工作区内自定义包：

| 包 | 类型 | 说明 |
|---|---|---|
| `unity_nav2` | `ament_python` | Unity/Nav2 联调启动与桥接节点 |
| `grid_planners` | `ament_cmake` | Nav2 全局规划器插件 |

## 模块

### unity_nav2

| 文件 | 说明 |
|---|---|
| `launch/unity_nav2.launch.py` | 启动完整 Unity/Nav2 联调栈 |
| `param/unity_nav2.yaml` | Nav2 参数、规划器插件、costmap、controller 配置 |
| `behavior_trees/navigate.xml` | 支持运行时切换 planner 的 Nav2 行为树 |
| `unity_nav2/map_bridge.py` | `/map_raw` → transient_local QoS `/map` |
| `unity_nav2/odom_tf_bridge.py` | `/odom` → odom TF |
| `unity_nav2/planner_switch.py` | `/planner_selector_unity` → `/planner_selector` |
| `unity_nav2/param_bridge.py` | `/planner_param_updates` JSON → Nav2 set_parameters |

### grid_planners

| 插件 ID | 实现 | 说明 |
|---|---|---|
| `Astar` | `grid_planners/AStarPlanner` | A*（octile 启发式，8邻接） |
| `Dijkstra` | `grid_planners/DijkstraPlanner` | Dijkstra（零启发式） |
| `Greedy` | `grid_planners/GBFSPlanner` | 贪心最优优先搜索 |
| `RRTStar` | `grid_planners/RRTStarPlanner` | RRT*（渐近最优采样） |
| `DLite` | `grid_planners/DLitePlanner` | D* Lite（增量式反向搜索） |
| `JPS` | `grid_planners/JPSPlanner` | Jump Point Search |
| `WAStar` | `grid_planners/WeightedAStarPlanner` | Weighted A*（f = g + w·h） |

所有自定义规划器向 `/planner_stats` 发布统一格式的统计数据（规划耗时、路径长度、节点展开数）。

## 编译与运行

```bash
# 编译
source /opt/ros/humble/setup.bash
colcon build --packages-select grid_planners unity_nav2
source install/setup.bash

# 启动完整联调栈
ros2 launch unity_nav2 unity_nav2.launch.py
```

launch 启动内容：`ros_tcp_endpoint`(0.0.0.0:10000)、`robot_state_publisher`、`odom_tf_bridge`、`map_bridge`、`planner_switch`、`param_bridge`、Nav2 导航栈、RViz2。

可选参数：

```bash
# 禁用 param_bridge（默认启用）
ros2 launch unity_nav2 unity_nav2.launch.py enable_planner_param_bridge:=false
```

## 与 Unity 配合操作

1. 先启动 ROS2：`ros2 launch unity_nav2 unity_nav2.launch.py`
2. 打开 Unity 场景 `Assets/Scenes/TurtleBot3.unity` 并点击 Play
3. `Tab` 切换算法，`Space` 发布目标点
4. 观察 RViz 路径显示和 Unity 统计面板

## 常用检查命令

```bash
ros2 node list
ros2 topic list
ros2 topic echo /planner_selector
ros2 topic echo /planner_stats
ros2 topic hz /map
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

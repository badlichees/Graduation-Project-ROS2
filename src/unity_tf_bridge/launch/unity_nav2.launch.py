#!/usr/bin/env python3
"""
unity_nav2.launch.py
Unity + Nav2 联合仿真主启动文件。

启动内容：
  1. ros_tcp_endpoint       — Unity↔ROS2 TCP 桥接（端口 10000）
  2. robot_state_publisher  — 发布 URDF / TF 静态链路
  3. odom_to_tf             — /odom → TF(odom→base_footprint)
  4. map_to_odom            — 静态变换 map→odom（恒等），仿真中以里程计为定位基准
  5. map_relay              — /map_raw(volatile) → /map(transient_local)
  6. nav2 navigation_launch — 全局/局部规划器、代价地图、BT导航（不含 AMCL / map_server）
  7. rviz2                  — 可视化

不使用 AMCL 的原因：
  Unity DirectPhysics 里程计无累积误差，可直接作为定位来源。
  map→odom 发布恒等变换，机器人在地图中的位姿即里程计位姿。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    os.environ['TURTLEBOT3_MODEL'] = turtlebot3_model
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Nav2 参数文件（使用 turtlebot3_navigation2 中已有的配置）
    nav2_param_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', ros_distro,
        f'{turtlebot3_model}.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz', 'tb3_navigation2.rviz'
    )

    # ── 1. ros_tcp_endpoint ──────────────────────────────────────────────────
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}],
        output='screen',
    )

    # ── 2 & 3. robot_state_publisher + odom_to_tf ───────────────────────────
    unity_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('unity_tf_bridge'),
            '/launch/unity_sim.launch.py',
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 4. map → odom 恒等静态变换 ───────────────────────────────────────────
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # ── 5. map_relay（QoS 桥接）────────────────────────────────────────────
    map_relay = Node(
        package='unity_tf_bridge',
        executable='map_relay',
        name='map_relay',
        output='screen',
    )

    # ── 6. Nav2 navigation（不含 map_server / AMCL）──────────────────────────
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py',
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_file,
            'autostart': 'true',
        }.items(),
    )

    # ── 7. RViz2 ─────────────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='使用仿真时钟（本项目用系统时钟，保持 false）'
        ),
        ros_tcp_endpoint,
        unity_sim,
        map_to_odom,
        map_relay,
        nav2_navigation,
        rviz2,
    ])

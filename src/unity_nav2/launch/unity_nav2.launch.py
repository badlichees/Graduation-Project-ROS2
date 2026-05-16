#!/usr/bin/env python3

"""TurtleBot3 Unity 仿真导航"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    os.environ['TURTLEBOT3_MODEL'] = turtlebot3_model

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_planner_param_bridge = LaunchConfiguration('enable_planner_param_bridge', default='true')

    nav2_param_file = os.path.join(
        get_package_share_directory('unity_nav2'),
        'param',
        'unity_nav2.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz', 'tb3_navigation2.rviz'
    )

    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}],
        output='screen',
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_bringup'),
            '/launch/turtlebot3_state_publisher.launch.py',
        ]),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': ''}.items(),
    )

    odom_tf_bridge = Node(
        package='unity_nav2',
        executable='odom_tf_bridge',
        name='odom_tf_bridge',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    map_bridge = Node(
        package='unity_nav2',
        executable='map_bridge',
        name='map_bridge',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    planner_switch = Node(
        package='unity_nav2',
        executable='planner_switch',
        name='planner_switch',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    param_bridge = Node(
        package='unity_nav2',
        executable='param_bridge',
        name='param_bridge',
        output='screen',
        condition=IfCondition(enable_planner_param_bridge),
    )

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
            description='是否使用仿真时钟'
        ),
        DeclareLaunchArgument(
            'enable_planner_param_bridge', default_value='true',
            description='是否启用 Unity 到 Nav2 的动态参数中继'
        ),
        ros_tcp_endpoint,
        state_publisher,
        odom_tf_bridge,
        map_to_odom,
        map_bridge,
        planner_switch,
        param_bridge,
        nav2_navigation,
        rviz2,
    ])

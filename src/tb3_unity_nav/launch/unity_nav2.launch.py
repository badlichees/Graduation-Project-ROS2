#!/usr/bin/env python3

"""启动 Unity + Nav2 测试栈。"""

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

    nav2_param_file = os.path.join(
        get_package_share_directory('tb3_unity_nav'),
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

    unity_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tb3_unity_nav'),
            '/launch/unity_sim.launch.py',
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    map_relay = Node(
        package='tb3_unity_nav',
        executable='map_relay',
        name='map_relay',
        output='screen',
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
        ros_tcp_endpoint,
        unity_sim,
        map_to_odom,
        map_relay,
        nav2_navigation,
        rviz2,
    ])

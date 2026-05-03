#!/usr/bin/env python3

"""启动 Unity 场景所需的机器人状态和 TF 桥接"""

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

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_bringup'),
            '/launch/turtlebot3_state_publisher.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }.items()
    )

    odom_to_tf_node = Node(
        package='tb3_unity_nav',
        executable='odom_to_tf',
        output='screen',
        name='odom_to_tf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时钟'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='机器人命名空间'
        ),
        state_publisher_launch,
        odom_to_tf_node
    ])

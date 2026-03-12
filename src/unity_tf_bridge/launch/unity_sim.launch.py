#!/usr/bin/env python3

# 同时启动robot_state_publisher和odom_to_tf节点

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 读取环境变量TURTLEBOT3_MODEL，默认为burger
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    os.environ['TURTLEBOT3_MODEL'] = turtlebot3_model

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') # 使用系统时钟，确保TF时间戳与仿真时间同步
    namespace = LaunchConfiguration('namespace', default='') # 无命名空间

    # 包含现有的turtlebot3_state_publisher启动文件，启动对应节点
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

    # odom_to_tf节点（来自本包）
    odom_to_tf_node = Node(
        package='unity_tf_bridge',
        executable='odom_to_tf',
        output='screen',
        name='odom_to_tf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot (empty for none)'
        ),
        state_publisher_launch,
        odom_to_tf_node
    ])
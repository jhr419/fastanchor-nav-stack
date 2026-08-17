#!/usr/bin/env python3
#
# Standalone launch for the yifanLIO -> FastAnchor interface adapter.
# Normally composed by fast_anchor_mid360.launch.py when lio_backend:=yifanlio.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('yifan_lio_adapter'),
        'config',
        'yifan_lio_adapter.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='yifan_lio_adapter',
            executable='yifan_lio_adapter_node',
            name='yifan_lio_adapter_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])

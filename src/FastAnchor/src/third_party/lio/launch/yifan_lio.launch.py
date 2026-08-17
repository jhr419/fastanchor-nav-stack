#!/usr/bin/env python3
#
# yifanLIO ROS2 launch
# 对应 ROS1 的 launch/start.launch:
#   - 启动 lio 节点 (config_path 参数, 与 ROS1 的 ~config_path 一致)
#   - 可选启动 rviz2 (与 ROS1 的 rviz 节点对应)
#
# 用法:
#   ros2 launch lio yifan_lio.launch.py
#   ros2 launch lio yifan_lio.launch.py rviz:=false
#   ros2 launch lio yifan_lio.launch.py use_sim_time:=true   # rosbag2 回放时使用
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lio')

    # ---------- launch 参数 ----------
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='root_config.yaml',
        description='root config yaml 文件名 (相对 lio/yaml 目录)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动 rviz2 可视化',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间 (rosbag2 回放时设为 true)',
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='是否发布 yifanLIO 自身的 TF (FastAnchor 集成时设为 false)',
    )

    # ---------- LIO 节点 ----------
    lio_node = Node(
        package='lio',
        executable='lio',
        name='lio_node',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }],
    )

    # ---------- RViz2 (可选) ----------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'LIO.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        config_path_arg,
        rviz_arg,
        use_sim_time_arg,
        publish_tf_arg,
        lio_node,
        rviz_node,
    ])

"""Bring up FastAnchor MID360 localization and SCAN-Planner for a real robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_pcd_path = LaunchConfiguration("map_pcd_path")
    localization_rviz_config = LaunchConfiguration("localization_rviz_config")
    planner_rviz_config = LaunchConfiguration("planner_rviz_config")

    fast_anchor_launch = PathJoinSubstitution([
        FindPackageShare("fast_anchor_bringup"),
        "launch",
        "fast_anchor_mid360.launch.py",
    ])
    planner_launch = PathJoinSubstitution([
        FindPackageShare("scan_planner"),
        "launch",
        "run.launch.py",
    ])
    default_localization_rviz = PathJoinSubstitution([
        FindPackageShare("fast_anchor_bringup"),
        "rviz",
        "fast_anchor.rviz",
    ])
    default_planner_rviz = PathJoinSubstitution([
        FindPackageShare("scan_planner"),
        "rviz",
        "default.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_pcd_path"),
        DeclareLaunchArgument("navi_mode", default_value="1"),
        DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("start_localization_rviz", default_value="true"),
        DeclareLaunchArgument("start_planner_rviz", default_value="true"),
        DeclareLaunchArgument("localization_rviz_config", default_value=default_localization_rviz),
        DeclareLaunchArgument("planner_rviz_config", default_value=default_planner_rviz),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_anchor_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_pcd_path": map_pcd_path,
                "visualization_map_pcd_path": map_pcd_path,
                "icp_map_pcd_path": map_pcd_path,
                # RViz instances are launched separately below.
                "rviz": "false",
                "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                "start_fastlio": LaunchConfiguration("start_fastlio"),
                "fastlio_localization_mode": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner_launch),
            launch_arguments={
                "is_real_world": "true",
                "localization_source": "fast_anchor",
                "sensor_type": "lidar",
                "navi_mode": LaunchConfiguration("navi_mode"),
                "controller_mode": LaunchConfiguration("controller_mode"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="fast_anchor_rviz",
            output="screen",
            arguments=["-d", localization_rviz_config],
            condition=IfCondition(LaunchConfiguration("start_localization_rviz")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="scan_planner_rviz",
            output="screen",
            # Keep SCAN-Planner's original config and only adapt its fixed
            # frame and global-map source to FastAnchor at runtime.
            arguments=["-d", planner_rviz_config, "-f", "map"],
            remappings=[
                ("/map_generator/global_cloud", "/fast_anchor/global_map"),
            ],
            condition=IfCondition(LaunchConfiguration("start_planner_rviz")),
        ),
    ])

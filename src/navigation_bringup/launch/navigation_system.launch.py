"""Minimal FastAnchor -> FastPlanner -> SCAN-Planner navigation bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_pcd_path = LaunchConfiguration("map_pcd_path")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    localization_pose_topic = LaunchConfiguration("localization_pose_topic")
    localization_cloud_topic = LaunchConfiguration("localization_cloud_topic")
    initial_pose_topic = LaunchConfiguration("initial_pose_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    fast_anchor_launch = PathJoinSubstitution([
        FindPackageShare("fast_anchor_bringup"),
        "launch",
        "fast_anchor_mid360.launch.py",
    ])
    scan_planner_launch = PathJoinSubstitution([
        FindPackageShare("scan_planner"),
        "launch",
        "run.launch.py",
    ])
    astar_config = PathJoinSubstitution([
        FindPackageShare("nav3d_global_planning"),
        "config",
        "astar_global_planner.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map_pcd_path", description="PCD map used by localization and global planning"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("localization_pose_topic", default_value="/fast_anchor/odom"),
        DeclareLaunchArgument("localization_cloud_topic", default_value="/fast_anchor/aligned_cloud"),
        DeclareLaunchArgument("initial_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("goal_topic", default_value="/move_base_simple/goal"),
        DeclareLaunchArgument("global_path_topic", default_value="/planned_path"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
        DeclareLaunchArgument("lidar_model", default_value="mid360"),
        DeclareLaunchArgument("start_localization", default_value="true"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_anchor_launch),
            condition=IfCondition(LaunchConfiguration("start_localization")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_pcd_path": map_pcd_path,
                "visualization_map_pcd_path": map_pcd_path,
                "icp_map_pcd_path": map_pcd_path,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "initial_pose_topic": initial_pose_topic,
                "output_odom_topic": localization_pose_topic,
                "aligned_cloud_topic": localization_cloud_topic,
                "lidar_model": LaunchConfiguration("lidar_model"),
                "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                "start_fastlio": LaunchConfiguration("start_fastlio"),
                "start_fastlio_rviz": "false",
                "rviz": "false",
            }.items(),
        ),
        Node(
            package="nav3d_global_planning",
            executable="astar_global_planner_node",
            name="astar_global_planner_node",
            output="screen",
            parameters=[
                astar_config,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "map_frame": map_frame,
                    "odom_frame": odom_frame,
                    "base_frame": base_frame,
                    "start_source": "odom",
                    "odom_topic": localization_pose_topic,
                    "goal_pose_topic": goal_topic,
                    "goal_point_topic": "",
                    "rviz_2d_goal_topic": "",
                    "map_source": "pcd",
                    "octomap_file": "",
                    "pcd_file": map_pcd_path,
                    "tomogram_topic": "",
                    "require_traversable_support": False,
                    "tomogram_cost_enabled": False,
                    "publish_path_topic": global_path_topic,
                    "publish_alias_path_topic": "",
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(scan_planner_launch),
            launch_arguments={
                "is_real_world": "true",
                "localization_source": "fast_anchor",
                "sensor_type": "lidar",
                "navi_mode": "3",
                "controller_mode": LaunchConfiguration("controller_mode"),
                "body_pose_topic": localization_pose_topic,
                "sensor_pose_topic": localization_pose_topic,
                "cloud_topic": localization_cloud_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "initial_pose_topic": initial_pose_topic,
                "goal_topic": goal_topic,
                "global_path_topic": global_path_topic,
                "reference_path_topic": "/scan_planner/manual_reference_path",
                "publish_marker_reference_path": "false",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])

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
    aligned_cloud_interval_s = LaunchConfiguration("aligned_cloud_interval_s")
    aligned_cloud_publish_rate_hz = LaunchConfiguration("aligned_cloud_publish_rate_hz")
    path_publish_interval_s = LaunchConfiguration("path_publish_interval_s")
    initial_pose_topic = LaunchConfiguration("initial_pose_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    local_target_distance = LaunchConfiguration("local_target_distance")
    grid_visualization_rate_hz = LaunchConfiguration("grid_visualization_rate_hz")
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
    global_planner_rviz_config = PathJoinSubstitution([
        FindPackageShare("pct_global_planner"),
        "rviz",
        "pct_global_planner.rviz",
    ])
    local_planner_rviz_config = PathJoinSubstitution([
        FindPackageShare("scan_planner"),
        "rviz",
        "default.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map_pcd_path", description="PCD map used by localization and global planning"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("localization_pose_topic", default_value="/fast_anchor/odom"),
        DeclareLaunchArgument("localization_cloud_topic", default_value="/fast_anchor/aligned_cloud"),
        DeclareLaunchArgument(
            "aligned_cloud_interval_s",
            default_value="0.0",
            description="FastAnchor aligned-cloud period; 0 restores sensor-rate output",
        ),
        DeclareLaunchArgument(
            "aligned_cloud_publish_rate_hz",
            default_value="25.0",
            description="Fixed FastAnchor aligned-cloud output rate; 0 disables cached repeats",
        ),
        DeclareLaunchArgument(
            "path_publish_interval_s",
            default_value="0.5",
            description="FastAnchor full-path publication period",
        ),
        DeclareLaunchArgument("initial_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("goal_topic", default_value="/move_base_simple/goal"),
        DeclareLaunchArgument("global_path_topic", default_value="/planned_path"),
        DeclareLaunchArgument(
            "local_target_distance",
            default_value="4.0",
            description="Distance from the robot to the SCAN local target in metres",
        ),
        DeclareLaunchArgument(
            "grid_visualization_rate_hz",
            default_value="5.0",
            description="SCAN occupancy visualization rate; 0 disables it on headless platforms",
        ),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
        DeclareLaunchArgument(
            "lidar_model",
            default_value="mid360s",
            choices=["mid360", "mid360s"],
            description="Livox LiDAR model: mid360 or mid360s",
        ),
        DeclareLaunchArgument("start_localization", default_value="true"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument(
            "start_localization_rviz",
            default_value="false",
            description="Start localization RViz (disabled by default for onboard CPU savings)",
        ),
        DeclareLaunchArgument("start_global_planner_rviz", default_value="false"),
        DeclareLaunchArgument(
            "start_local_planner_rviz",
            default_value="false",
            description="Start SCAN RViz (disabled by default for onboard CPU savings)",
        ),
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
                "aligned_cloud_interval_s": aligned_cloud_interval_s,
                "aligned_cloud_publish_rate_hz": aligned_cloud_publish_rate_hz,
                "path_publish_interval_s": path_publish_interval_s,
                "lidar_model": LaunchConfiguration("lidar_model"),
                "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                "start_fastlio": LaunchConfiguration("start_fastlio"),
                "start_fastlio_rviz": "false",
                "rviz": LaunchConfiguration("start_localization_rviz"),
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
        Node(
            package="rviz2",
            executable="rviz2",
            name="global_planner_rviz2",
            output="screen",
            arguments=["-d", global_planner_rviz_config, "-f", map_frame],
            condition=IfCondition(LaunchConfiguration("start_global_planner_rviz")),
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
                "local_target_distance": local_target_distance,
                "grid_visualization_rate_hz": grid_visualization_rate_hz,
                "reference_path_topic": "/scan_planner/manual_reference_path",
                "publish_marker_reference_path": "false",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="local_planner_rviz2",
            output="screen",
            arguments=["-d", local_planner_rviz_config, "-f", map_frame],
            condition=IfCondition(LaunchConfiguration("start_local_planner_rviz")),
        ),
    ])

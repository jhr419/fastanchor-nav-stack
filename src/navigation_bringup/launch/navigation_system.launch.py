"""Minimal FastAnchor -> FastPlanner -> SCAN-Planner navigation bringup."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from navigation_bringup.launch_config import (
    load_navigation_system_defaults,
    load_node_parameters,
)


LEG_ODOM_CONFIG = [
    False, False, False,
    False, False, False,
    True, True, True,
    False, False, False,
    False, False, False,
]


def _is_true(value):
    return value.strip().lower() in ("true", "1", "yes", "on")


def _launch_odometry_fusion(context, fusion_parameters):
    mode = LaunchConfiguration("odometry_fusion_mode").perform(context).strip().lower()
    if mode not in ("none", "leg"):
        raise RuntimeError(
            "odometry_fusion_mode must be one of: none, leg "
            f"(got '{mode}')"
        )
    if mode == "none":
        return []

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    raw_fast_lio_odom_topic = LaunchConfiguration("raw_fast_lio_odom_topic")
    fast_lio_base_odom_topic = LaunchConfiguration("fast_lio_base_odom_topic")
    filtered_base_odom_topic = LaunchConfiguration("filtered_base_odom_topic")
    fused_body_odom_topic = LaunchConfiguration("fused_body_odom_topic")
    leg_odom_topic = LaunchConfiguration("leg_odom_topic")

    actions = []
    if _is_true(
        LaunchConfiguration("enable_unitree_sportmode_adapter").perform(context)
    ):
        actions.append(
            Node(
                package="fast_anchor_fusion",
                executable="unitree_sportmode_to_odom.py",
                name="unitree_sportmode_to_odom",
                output="screen",
                parameters=[
                    fusion_parameters["unitree_sportmode_to_odom"],
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": LaunchConfiguration("unitree_sportmode_topic"),
                        "output_topic": leg_odom_topic,
                        "base_frame": base_frame,
                    },
                ],
            )
        )

    actions.extend([
        Node(
            package="fast_anchor_fusion",
            executable="odom_frame_adapter_node",
            name="fast_anchor_odom_frame_adapter",
            output="screen",
            parameters=[
                fusion_parameters["fast_anchor_odom_frame_adapter"],
                {
                    "use_sim_time": use_sim_time,
                    "frames.base": base_frame,
                    "topics.raw_fast_lio_odom": raw_fast_lio_odom_topic,
                    "topics.fast_lio_base_odom": fast_lio_base_odom_topic,
                    "topics.filtered_base_odom": filtered_base_odom_topic,
                    "topics.fused_body_odom": fused_body_odom_topic,
                },
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="fast_anchor_ekf",
            output="screen",
            parameters=[
                fusion_parameters["fast_anchor_ekf"],
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": map_frame,
                    "odom_frame": odom_frame,
                    "world_frame": odom_frame,
                    "base_link_frame": base_frame,
                    "odom0": fast_lio_base_odom_topic,
                    "odom1": leg_odom_topic,
                    "odom1_config": LEG_ODOM_CONFIG,
                },
            ],
            remappings=[("odometry/filtered", filtered_base_odom_topic)],
        ),
    ])
    return actions


def generate_launch_description():
    config_path = (
        f"{get_package_share_directory('navigation_bringup')}"
        "/config/navigation_system.yaml"
    )
    configured_defaults = load_navigation_system_defaults(config_path)
    fusion_parameters = {
        node_name: load_node_parameters(config_path, node_name)
        for node_name in (
            "unitree_sportmode_to_odom",
            "fast_anchor_odom_frame_adapter",
            "fast_anchor_ekf",
        )
    }

    def configured(name, fallback):
        return configured_defaults.get(name, str(fallback))

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
    self_filter_enabled = LaunchConfiguration("self_filter_enabled")
    self_filter_min_x = LaunchConfiguration("self_filter_min_x")
    self_filter_max_x = LaunchConfiguration("self_filter_max_x")
    self_filter_min_y = LaunchConfiguration("self_filter_min_y")
    self_filter_max_y = LaunchConfiguration("self_filter_max_y")
    self_filter_min_z = LaunchConfiguration("self_filter_min_z")
    self_filter_max_z = LaunchConfiguration("self_filter_max_z")
    initial_pose_topic = LaunchConfiguration("initial_pose_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    scan_initial_path_topic = LaunchConfiguration("scan_initial_path_topic")
    path_cropper_update_rate = LaunchConfiguration("path_cropper_update_rate")
    path_window_robot_aligned = LaunchConfiguration("path_window_robot_aligned")
    local_target_distance = LaunchConfiguration("local_target_distance")
    grid_visualization_rate_hz = LaunchConfiguration("grid_visualization_rate_hz")
    runtime_log_enabled = LaunchConfiguration("runtime_log_enabled")
    runtime_log_report_interval_sec = LaunchConfiguration("runtime_log_report_interval_sec")
    runtime_log_map_target_rate_hz = LaunchConfiguration("runtime_log_map_target_rate_hz")
    runtime_log_csv_path = LaunchConfiguration("runtime_log_csv_path")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    waypoints_file = LaunchConfiguration("waypoints_file")
    lio_backend = LaunchConfiguration("lio_backend")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    fastlio_lidar_topic = LaunchConfiguration("fastlio_lidar_topic")
    fastlio_imu_topic = LaunchConfiguration("fastlio_imu_topic")
    fastlio_lidar_type = LaunchConfiguration("fastlio_lidar_type")
    fastlio_localization_mode = LaunchConfiguration("fastlio_localization_mode")
    odometry_fusion_mode = LaunchConfiguration("odometry_fusion_mode")
    raw_fast_lio_odom_topic = LaunchConfiguration("raw_fast_lio_odom_topic")
    fused_body_odom_topic = LaunchConfiguration("fused_body_odom_topic")
    localization_odom_topic = PythonExpression([
        "'", raw_fast_lio_odom_topic, "' if '", odometry_fusion_mode,
        "' == 'none' else '", fused_body_odom_topic, "'",
    ])
    lio_sync_tolerance_s = PythonExpression([
        LaunchConfiguration("direct_lio_sync_tolerance_s"),
        " if '", odometry_fusion_mode, "' == 'none' else ",
        LaunchConfiguration("fusion_lio_sync_tolerance_s"),
    ])

    fast_anchor_launch = PathJoinSubstitution([
        FindPackageShare("fast_anchor_bringup"),
        "launch",
        "fast_anchor_mid360.launch.py",
    ])
    scan_planner_launch = PathJoinSubstitution([
        FindPackageShare("scan_planner"),
        "launch",
        "run_windowed.launch.py",
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
        DeclareLaunchArgument(
            "scan_initial_path_topic",
            default_value=configured(
                "scan_initial_path_topic", "/scan_planner/initial_path"
            ),
            description="Cropped global path passed to SCAN-Planner",
        ),
        DeclareLaunchArgument(
            "map_pcd_path",
            default_value=configured("map_pcd_path", ""),
            description="PCD map used by localization and global planning",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value=configured("use_sim_time", "false")
        ),
        DeclareLaunchArgument("map_frame", default_value=configured("map_frame", "map")),
        DeclareLaunchArgument(
            "odom_frame", default_value=configured("odom_frame", "camera_init")
        ),
        DeclareLaunchArgument(
            "base_frame", default_value=configured("base_frame", "base_link")
        ),
        DeclareLaunchArgument(
            "localization_pose_topic",
            default_value=configured("localization_pose_topic", "/fast_anchor/odom"),
        ),
        DeclareLaunchArgument(
            "localization_cloud_topic",
            default_value=configured(
                "localization_cloud_topic", "/fast_anchor/aligned_cloud"
            ),
        ),
        DeclareLaunchArgument(
            "aligned_cloud_interval_s",
            default_value=configured("aligned_cloud_interval_s", "0.0"),
            description="FastAnchor aligned-cloud period; 0 restores sensor-rate output",
        ),
        DeclareLaunchArgument(
            "aligned_cloud_publish_rate_hz",
            default_value=configured("aligned_cloud_publish_rate_hz", "25.0"),
            description="Fixed FastAnchor aligned-cloud output rate; 0 disables cached repeats",
        ),
        DeclareLaunchArgument(
            "path_publish_interval_s",
            default_value=configured("path_publish_interval_s", "0.5"),
            description="FastAnchor full-path publication period",
        ),
        DeclareLaunchArgument(
            "self_filter_enabled",
            default_value=configured("self_filter_enabled", "true"),
        ),
        DeclareLaunchArgument(
            "self_filter_min_x", default_value=configured("self_filter_min_x", "-0.25")
        ),
        DeclareLaunchArgument(
            "self_filter_max_x", default_value=configured("self_filter_max_x", "0.35")
        ),
        DeclareLaunchArgument(
            "self_filter_min_y", default_value=configured("self_filter_min_y", "-0.15")
        ),
        DeclareLaunchArgument(
            "self_filter_max_y", default_value=configured("self_filter_max_y", "0.15")
        ),
        DeclareLaunchArgument(
            "self_filter_min_z", default_value=configured("self_filter_min_z", "-0.10")
        ),
        DeclareLaunchArgument(
            "self_filter_max_z", default_value=configured("self_filter_max_z", "0.30")
        ),
        DeclareLaunchArgument(
            "initial_pose_topic",
            default_value=configured("initial_pose_topic", "/initialpose"),
        ),
        DeclareLaunchArgument(
            "goal_topic",
            default_value=configured("goal_topic", "/move_base_simple/goal"),
        ),
        DeclareLaunchArgument(
            "global_path_topic",
            default_value=configured("global_path_topic", "/planned_path"),
            description="Stable full global path published by the global planner",
        ),
        DeclareLaunchArgument(
            "path_cropper_update_rate",
            default_value=configured("path_cropper_update_rate", "20.0"),
            description="Global-path window cropper update rate in Hz",
        ),
        DeclareLaunchArgument(
            "path_window_robot_aligned",
            default_value=configured("path_window_robot_aligned", "false"),
            description=(
                "Whether the crop window rotates with robot yaw; must match the "
                "SCAN-Planner sliding-window definition"
            ),
        ),
        DeclareLaunchArgument(
            "local_target_distance",
            default_value=configured("local_target_distance", "4.0"),
            description="Distance from the robot to the SCAN local target in metres",
        ),
        DeclareLaunchArgument(
            "grid_visualization_rate_hz",
            default_value=configured("grid_visualization_rate_hz", "5.0"),
            description="SCAN occupancy visualization rate; 0 disables it on headless platforms",
        ),
        DeclareLaunchArgument(
            "runtime_log_enabled",
            default_value=configured("runtime_log_enabled", "true"),
            description="Log local-map timing, bottlenecks, CPU, and memory usage",
        ),
        DeclareLaunchArgument(
            "runtime_log_report_interval_sec",
            default_value=configured("runtime_log_report_interval_sec", "5.0"),
        ),
        DeclareLaunchArgument(
            "runtime_log_map_target_rate_hz",
            default_value=configured("runtime_log_map_target_rate_hz", "10.0"),
            description="Required unique local obstacle-map fusion rate",
        ),
        DeclareLaunchArgument(
            "runtime_log_csv_path",
            default_value=configured("runtime_log_csv_path", ""),
            description="Optional append-only runtime metrics CSV path",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value=configured("cmd_vel_topic", "/cmd_vel")
        ),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=configured("waypoints_file", ""),
            description=(
                "ROS 2 parameter YAML for waypoint_mission_manager; an empty value keeps "
                "the normal single-goal mode"
            ),
        ),
        DeclareLaunchArgument(
            "waypoint_status_topic",
            default_value=configured(
                "waypoint_status_topic", "/waypoint_mission/status"
            ),
        ),
        DeclareLaunchArgument(
            "waypoint_xy_tolerance",
            default_value=configured("waypoint_xy_tolerance", "0.5"),
            description="XY arrival tolerance for advancing to the next waypoint",
        ),
        DeclareLaunchArgument(
            "waypoint_z_tolerance",
            default_value=configured("waypoint_z_tolerance", "-1.0"),
            description="Z arrival tolerance; a negative value disables the Z check",
        ),
        DeclareLaunchArgument(
            "waypoint_path_goal_tolerance",
            default_value=configured("waypoint_path_goal_tolerance", "0.75"),
            description="Maximum XY difference between a planned path endpoint and its waypoint",
        ),
        DeclareLaunchArgument(
            "waypoint_hold_time",
            default_value=configured("waypoint_hold_time", "0.5"),
            description="Time that odometry must remain inside the arrival tolerance",
        ),
        DeclareLaunchArgument(
            "waypoint_loop",
            default_value=configured("waypoint_loop", "false"),
            description="Restart from the first waypoint after completing the mission",
        ),
        DeclareLaunchArgument(
            "controller_mode",
            default_value=configured("controller_mode", "closed_loop"),
        ),
        DeclareLaunchArgument(
            "lio_backend",
            default_value=configured("lio_backend", "fastlio2"),
            choices=["fastlio2"],
            description="LIO frontend used by the integrated system",
        ),
        DeclareLaunchArgument(
            "lidar_model",
            default_value=configured("lidar_model", "mid360s"),
            choices=["mid360", "mid360s"],
            description="Livox LiDAR model: mid360 or mid360s",
        ),
        DeclareLaunchArgument(
            "fastlio_config_file",
            default_value=configured(
                "fastlio_config_file", "mid360_localization.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "fastlio_lidar_topic",
            default_value=configured("fastlio_lidar_topic", "/livox/lidar"),
        ),
        DeclareLaunchArgument(
            "fastlio_imu_topic",
            default_value=configured("fastlio_imu_topic", "/livox/imu"),
        ),
        DeclareLaunchArgument(
            "fastlio_lidar_type",
            default_value=configured("fastlio_lidar_type", "1"),
        ),
        DeclareLaunchArgument(
            "fastlio_localization_mode",
            default_value=configured("fastlio_localization_mode", "false"),
        ),
        DeclareLaunchArgument(
            "odometry_fusion_mode",
            default_value=configured("odometry_fusion_mode", "leg"),
            choices=["none", "leg"],
        ),
        DeclareLaunchArgument(
            "enable_unitree_sportmode_adapter",
            default_value=configured("enable_unitree_sportmode_adapter", "true"),
        ),
        DeclareLaunchArgument(
            "unitree_sportmode_topic",
            default_value=configured("unitree_sportmode_topic", "/lf/sportmodestate"),
        ),
        DeclareLaunchArgument(
            "leg_odom_topic",
            default_value=configured("leg_odom_topic", "/leg_odom"),
        ),
        DeclareLaunchArgument(
            "raw_fast_lio_odom_topic",
            default_value=configured("raw_fast_lio_odom_topic", "/Odometry"),
        ),
        DeclareLaunchArgument(
            "fast_lio_base_odom_topic",
            default_value=configured(
                "fast_lio_base_odom_topic",
                "/fast_anchor/fusion/fast_lio_base_odom",
            ),
        ),
        DeclareLaunchArgument(
            "filtered_base_odom_topic",
            default_value=configured(
                "filtered_base_odom_topic",
                "/fast_anchor/fusion/filtered_base_odom",
            ),
        ),
        DeclareLaunchArgument(
            "fused_body_odom_topic",
            default_value=configured(
                "fused_body_odom_topic",
                "/fast_anchor/fusion/fused_body_odom",
            ),
        ),
        DeclareLaunchArgument(
            "direct_lio_sync_tolerance_s",
            default_value=configured("direct_lio_sync_tolerance_s", "0.001"),
        ),
        DeclareLaunchArgument(
            "fusion_lio_sync_tolerance_s",
            default_value=configured("fusion_lio_sync_tolerance_s", "0.05"),
        ),
        DeclareLaunchArgument(
            "start_localization",
            default_value=configured("start_localization", "true"),
        ),
        DeclareLaunchArgument(
            "start_livox_driver",
            default_value=configured("start_livox_driver", "true"),
        ),
        DeclareLaunchArgument(
            "start_fastlio",
            default_value=configured("start_fastlio", "true"),
        ),
        DeclareLaunchArgument(
            "start_localization_rviz",
            default_value=configured("start_localization_rviz", "false"),
            description="Start localization RViz (disabled by default for onboard CPU savings)",
        ),
        DeclareLaunchArgument(
            "start_global_planner_rviz",
            default_value=configured("start_global_planner_rviz", "false"),
        ),
        DeclareLaunchArgument(
            "start_local_planner_rviz",
            default_value=configured("start_local_planner_rviz", "false"),
            description="Start SCAN RViz (disabled by default for onboard CPU savings)",
        ),
        OpaqueFunction(
            function=_launch_odometry_fusion,
            kwargs={"fusion_parameters": fusion_parameters},
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
                "localization_odom_topic": localization_odom_topic,
                "lio_sync_tolerance_s": lio_sync_tolerance_s,
                "initial_pose_topic": initial_pose_topic,
                "output_odom_topic": localization_pose_topic,
                "aligned_cloud_topic": localization_cloud_topic,
                "aligned_cloud_interval_s": aligned_cloud_interval_s,
                "aligned_cloud_publish_rate_hz": aligned_cloud_publish_rate_hz,
                "path_publish_interval_s": path_publish_interval_s,
                "self_filter_enabled": self_filter_enabled,
                "self_filter_min_x": self_filter_min_x,
                "self_filter_max_x": self_filter_max_x,
                "self_filter_min_y": self_filter_min_y,
                "self_filter_max_y": self_filter_max_y,
                "self_filter_min_z": self_filter_min_z,
                "self_filter_max_z": self_filter_max_z,
                "lio_backend": lio_backend,
                "lidar_model": LaunchConfiguration("lidar_model"),
                "fastlio_config_file": fastlio_config_file,
                "fastlio_lidar_topic": fastlio_lidar_topic,
                "fastlio_imu_topic": fastlio_imu_topic,
                "fastlio_lidar_type": fastlio_lidar_type,
                "fastlio_localization_mode": fastlio_localization_mode,
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
            package="navigation_bringup",
            executable="waypoint_mission_manager",
            name="waypoint_mission_manager",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "frame_id": map_frame,
                    "odom_topic": localization_pose_topic,
                    "goal_topic": goal_topic,
                    "path_topic": global_path_topic,
                    "status_topic": LaunchConfiguration("waypoint_status_topic"),
                    "xy_tolerance": ParameterValue(
                        LaunchConfiguration("waypoint_xy_tolerance"), value_type=float
                    ),
                    "z_tolerance": ParameterValue(
                        LaunchConfiguration("waypoint_z_tolerance"), value_type=float
                    ),
                    "path_goal_tolerance": ParameterValue(
                        LaunchConfiguration("waypoint_path_goal_tolerance"), value_type=float
                    ),
                    "hold_time": ParameterValue(
                        LaunchConfiguration("waypoint_hold_time"), value_type=float
                    ),
                    "loop": ParameterValue(
                        LaunchConfiguration("waypoint_loop"), value_type=bool
                    ),
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
                # Full FastPlanner path. run_windowed.launch.py crops this path before
                # remapping the cropped segment into SCAN-Planner's `initial_path`.
                "global_path_topic": global_path_topic,
                "scan_initial_path_topic": scan_initial_path_topic,
                "path_cropper_robot_frame": base_frame,
                "path_cropper_update_rate": path_cropper_update_rate,
                "path_window_robot_aligned": path_window_robot_aligned,
                "local_target_distance": local_target_distance,
                "grid_visualization_rate_hz": grid_visualization_rate_hz,
                "runtime_log_enabled": runtime_log_enabled,
                "runtime_log_report_interval_sec": runtime_log_report_interval_sec,
                "runtime_log_map_target_rate_hz": runtime_log_map_target_rate_hz,
                "runtime_log_csv_path": runtime_log_csv_path,
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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


LIVOX_LAUNCH_FILES = {
    "mid360": "msg_MID360_launch.py",
    "mid360s": "msg_MID360s_launch.py",
}

LIO_BACKENDS = ("fastlio2", "yifanlio")


def _include_livox_driver(context):
    lidar_model = LaunchConfiguration("lidar_model").perform(context)
    try:
        launch_file = LIVOX_LAUNCH_FILES[lidar_model]
    except KeyError as exc:
        supported_models = ", ".join(LIVOX_LAUNCH_FILES)
        raise RuntimeError(
            f"Unsupported lidar_model '{lidar_model}'. "
            f"Choose one of: {supported_models}."
        ) from exc

    livox_launch = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "launch_ROS2",
        launch_file,
    ])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch),
            condition=IfCondition(LaunchConfiguration("start_livox_driver")),
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    lio_backend = LaunchConfiguration("lio_backend")
    start_fastlio = LaunchConfiguration("start_fastlio")
    start_fastlio_rviz = LaunchConfiguration("start_fastlio_rviz")
    start_rviz = LaunchConfiguration("rviz")
    config_file = LaunchConfiguration("config_file")
    map_pcd_path = LaunchConfiguration("map_pcd_path")
    visualization_map_pcd_path = LaunchConfiguration("visualization_map_pcd_path")
    icp_map_pcd_path = LaunchConfiguration("icp_map_pcd_path")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    fastlio_lidar_topic = LaunchConfiguration("fastlio_lidar_topic")
    fastlio_imu_topic = LaunchConfiguration("fastlio_imu_topic")
    fastlio_lidar_type = LaunchConfiguration("fastlio_lidar_type")
    fastlio_localization_mode = LaunchConfiguration("fastlio_localization_mode")
    yifanlio_config_path = LaunchConfiguration("yifanlio_config_path")
    yifanlio_publish_tf = LaunchConfiguration("yifanlio_publish_tf")
    yifanlio_adapter_config = LaunchConfiguration("yifanlio_adapter_config")
    rviz_config = LaunchConfiguration("rviz_config")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    localization_odom_topic = LaunchConfiguration("localization_odom_topic")
    lio_sync_tolerance_s = LaunchConfiguration("lio_sync_tolerance_s")
    initial_pose_topic = LaunchConfiguration("initial_pose_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    aligned_cloud_topic = LaunchConfiguration("aligned_cloud_topic")
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

    default_config = PathJoinSubstitution([
        FindPackageShare("fast_anchor_bringup"),
        "config",
        "fast_anchor_localization.yaml",
    ])
    default_rviz = PathJoinSubstitution([
        FindPackageShare("fast_anchor_bringup"),
        "rviz",
        "fast_anchor.rviz",
    ])
    default_yifanlio_adapter_config = PathJoinSubstitution([
        FindPackageShare("yifan_lio_adapter"),
        "config",
        "yifan_lio_adapter.yaml",
    ])

    fastlio_launch = PathJoinSubstitution([
        FindPackageShare("fast_lio"),
        "launch",
        "mapping.launch.py",
    ])

    fastlio_enabled = PythonExpression([
        "'", lio_backend, "' == 'fastlio2' and '", start_fastlio, "' == 'true'",
    ])
    yifanlio_enabled = PythonExpression(["'", lio_backend, "' == 'yifanlio'"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "lio_backend",
            default_value="fastlio2",
            choices=list(LIO_BACKENDS),
            description="Select LIO frontend: fastlio2 (default) or yifanlio",
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="camera_init"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("localization_odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("lio_sync_tolerance_s", default_value="0.001"),
        DeclareLaunchArgument("initial_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("output_odom_topic", default_value="/fast_anchor/odom"),
        DeclareLaunchArgument("aligned_cloud_topic", default_value="/fast_anchor/aligned_cloud"),
        DeclareLaunchArgument(
            "aligned_cloud_interval_s",
            default_value="0.0",
            description="Minimum aligned-cloud period; 0 publishes every input scan",
        ),
        DeclareLaunchArgument(
            "aligned_cloud_publish_rate_hz",
            default_value="25.0",
            description="Fixed aligned-cloud output rate; 0 publishes only fresh input scans",
        ),
        DeclareLaunchArgument(
            "path_publish_interval_s",
            default_value="0.5",
            description="Minimum full localization-path publication period",
        ),
        DeclareLaunchArgument("self_filter_enabled", default_value="true"),
        DeclareLaunchArgument("self_filter_min_x", default_value="-0.25"),
        DeclareLaunchArgument("self_filter_max_x", default_value="0.35"),
        DeclareLaunchArgument("self_filter_min_y", default_value="-0.15"),
        DeclareLaunchArgument("self_filter_max_y", default_value="0.15"),
        DeclareLaunchArgument("self_filter_min_z", default_value="-0.10"),
        DeclareLaunchArgument("self_filter_max_z", default_value="0.30"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument(
            "lidar_model",
            default_value="mid360s",
            choices=list(LIVOX_LAUNCH_FILES),
            description="Livox LiDAR model used to select the driver launch file",
        ),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("start_fastlio_rviz", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("map_pcd_path", default_value="maps/map_preprocessed.pcd"),
        DeclareLaunchArgument(
            "visualization_map_pcd_path",
            default_value="maps/map_visualization.pcd",
        ),
        DeclareLaunchArgument("icp_map_pcd_path", default_value="maps/map_preprocessed.pcd"),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360_localization.yaml"),
        DeclareLaunchArgument("fastlio_lidar_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("fastlio_imu_topic", default_value="/livox/imu"),
        DeclareLaunchArgument("fastlio_lidar_type", default_value="1"),
        # The bundled FAST-LIO odom-only branch skips LiDAR map matching and
        # quickly drifts under pure IMU integration. Keep normal scan matching
        # enabled; map publication and PCD saving remain disabled by YAML.
        DeclareLaunchArgument("fastlio_localization_mode", default_value="false"),
        DeclareLaunchArgument(
            "yifanlio_config_path",
            default_value="root_localization.yaml",
            description="yifanLIO root config (relative to lio/yaml)",
        ),
        DeclareLaunchArgument(
            "yifanlio_publish_tf",
            default_value="false",
            description="Disable yifanLIO's own TF; the adapter/FastAnchor provide the TF tree",
        ),
        DeclareLaunchArgument(
            "yifanlio_adapter_config",
            default_value=default_yifanlio_adapter_config,
            description="yifan_lio_adapter parameter YAML",
        ),
        LogInfo(msg=["[FastAnchor] LIO backend: ", lio_backend]),
        GroupAction(
            scoped=True,
            actions=[
                OpaqueFunction(function=_include_livox_driver),
            ],
        ),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(fastlio_launch),
                    condition=IfCondition(fastlio_enabled),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "config_path": PathJoinSubstitution([FindPackageShare("fast_lio"), "config"]),
                        "config_file": fastlio_config_file,
                        "rviz": start_fastlio_rviz,
                        "lid_topic": fastlio_lidar_topic,
                        "imu_topic": fastlio_imu_topic,
                        "lidar_type": fastlio_lidar_type,
                        "localization_mode": fastlio_localization_mode,
                    }.items(),
                ),
            ],
        ),
        Node(
            package="lio",
            executable="lio",
            name="lio_node",
            output="screen",
            condition=IfCondition(yifanlio_enabled),
            parameters=[
                {
                    "config_path": yifanlio_config_path,
                    "use_sim_time": use_sim_time,
                    "publish_tf": yifanlio_publish_tf,
                }
            ],
        ),
        Node(
            package="yifan_lio_adapter",
            executable="yifan_lio_adapter_node",
            name="yifan_lio_adapter_node",
            output="screen",
            condition=IfCondition(yifanlio_enabled),
            parameters=[
                yifanlio_adapter_config,
                {
                    "use_sim_time": use_sim_time,
                    # Keep the adapter's topics/extrinsic bound to the exact
                    # root config selected for the yifanLIO node.
                    "lio_root_config": yifanlio_config_path,
                },
            ],
        ),
        Node(
            package="fast_anchor_localization",
            executable="fast_anchor_localization_node",
            name="fast_anchor_localization_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_pcd_path": map_pcd_path,
                    "visualization_map_pcd_path": visualization_map_pcd_path,
                    "icp_map_pcd_path": icp_map_pcd_path,
                    "map.pcd_path": map_pcd_path,
                    "map.visualization_pcd_path": visualization_map_pcd_path,
                    "map.icp_pcd_path": icp_map_pcd_path,
                    "map_frame": map_frame,
                    "odom_frame": odom_frame,
                    "base_frame": base_frame,
                    "frames.map_frame": map_frame,
                    "frames.odom_frame": odom_frame,
                    "frames.base_frame": base_frame,
                    "odom_topic": localization_odom_topic,
                    "topics.fast_lio_odom": localization_odom_topic,
                    "lio_sync.tolerance_s": ParameterValue(
                        lio_sync_tolerance_s, value_type=float
                    ),
                    "initialpose_topic": initial_pose_topic,
                    "odom_output_topic": output_odom_topic,
                    "aligned_cloud_topic": aligned_cloud_topic,
                    "topics.initial_pose": initial_pose_topic,
                    "topics.output_odom": output_odom_topic,
                    "topics.aligned_cloud": aligned_cloud_topic,
                    "output.aligned_cloud_interval_s": ParameterValue(
                        aligned_cloud_interval_s, value_type=float
                    ),
                    "output.aligned_cloud_publish_rate_hz": ParameterValue(
                        aligned_cloud_publish_rate_hz, value_type=float
                    ),
                    "output.path_publish_interval_s": ParameterValue(
                        path_publish_interval_s, value_type=float
                    ),
                    "cloud_preprocess.self_filter.enabled": ParameterValue(
                        self_filter_enabled, value_type=bool
                    ),
                    "cloud_preprocess.self_filter.min_x": ParameterValue(
                        self_filter_min_x, value_type=float
                    ),
                    "cloud_preprocess.self_filter.max_x": ParameterValue(
                        self_filter_max_x, value_type=float
                    ),
                    "cloud_preprocess.self_filter.min_y": ParameterValue(
                        self_filter_min_y, value_type=float
                    ),
                    "cloud_preprocess.self_filter.max_y": ParameterValue(
                        self_filter_max_y, value_type=float
                    ),
                    "cloud_preprocess.self_filter.min_z": ParameterValue(
                        self_filter_min_z, value_type=float
                    ),
                    "cloud_preprocess.self_filter.max_z": ParameterValue(
                        self_filter_max_z, value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_livox_frame",
            arguments=[
                "--x", "0.3",
                "--y", "0.0",
                "--z", "0.38",
                "--roll", "-0.034101",
                "--pitch", "0.566395",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "livox_frame",
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(start_rviz),
        ),
    ])

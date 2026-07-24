from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LIVOX_LAUNCH_FILES = {
    "mid360": "msg_MID360_launch.py",
    "mid360s": "msg_MID360s_launch.py",
}


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
    rviz_config = LaunchConfiguration("rviz_config")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    initial_pose_topic = LaunchConfiguration("initial_pose_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    aligned_cloud_topic = LaunchConfiguration("aligned_cloud_topic")

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

    fastlio_launch = PathJoinSubstitution([
        FindPackageShare("fast_lio"),
        "launch",
        "mapping.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="camera_init"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("initial_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("output_odom_topic", default_value="/fast_anchor/odom"),
        DeclareLaunchArgument("aligned_cloud_topic", default_value="/fast_anchor/aligned_cloud"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument(
            "lidar_model",
            default_value="mid360",
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
                    condition=IfCondition(start_fastlio),
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
                    "initialpose_topic": initial_pose_topic,
                    "odom_output_topic": output_odom_topic,
                    "aligned_cloud_topic": aligned_cloud_topic,
                    "topics.initial_pose": initial_pose_topic,
                    "topics.output_odom": output_odom_topic,
                    "topics.aligned_cloud": aligned_cloud_topic,
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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LIO_BACKENDS = ("fastlio2", "yifanlio")


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lio_backend = LaunchConfiguration("lio_backend")
    config_file = LaunchConfiguration("config_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    fastlio_lidar_topic = LaunchConfiguration("fastlio_lidar_topic")
    fastlio_imu_topic = LaunchConfiguration("fastlio_imu_topic")
    fastlio_lidar_type = LaunchConfiguration("fastlio_lidar_type")
    yifanlio_config_path = LaunchConfiguration("yifanlio_config_path")
    yifanlio_publish_tf = LaunchConfiguration("yifanlio_publish_tf")
    yifanlio_adapter_config = LaunchConfiguration("yifanlio_adapter_config")

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

    fastlio_enabled = PythonExpression(["'", lio_backend, "' == 'fastlio2'"])
    yifanlio_enabled = PythonExpression(["'", lio_backend, "' == 'yifanlio'"])

    return LaunchDescription([
        DeclareLaunchArgument("bag_path", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "lio_backend",
            default_value="fastlio2",
            choices=list(LIO_BACKENDS),
            description="Select LIO frontend: fastlio2 (default) or yifanlio",
        ),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360_localization.yaml"),
        DeclareLaunchArgument("fastlio_lidar_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("fastlio_imu_topic", default_value="/livox/imu"),
        DeclareLaunchArgument("fastlio_lidar_type", default_value="1"),
        DeclareLaunchArgument(
            "yifanlio_config_path",
            default_value="root_localization.yaml",
            description="yifanLIO root config (relative to lio/yaml)",
        ),
        DeclareLaunchArgument(
            "yifanlio_publish_tf",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "yifanlio_adapter_config",
            default_value=default_yifanlio_adapter_config,
        ),
        LogInfo(msg=["[FastAnchor] LIO backend: ", lio_backend]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_launch),
            condition=IfCondition(fastlio_enabled),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "config_path": PathJoinSubstitution([FindPackageShare("fast_lio"), "config"]),
                "config_file": fastlio_config_file,
                "rviz": "false",
                "lid_topic": fastlio_lidar_topic,
                "imu_topic": fastlio_imu_topic,
                "lidar_type": fastlio_lidar_type,
                "localization_mode": "false",
            }.items(),
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
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="fast_anchor_localization",
            executable="fast_anchor_localization_node",
            name="fast_anchor_localization_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag_path, "--clock"],
            output="screen",
            condition=IfCondition(PythonExpression(["'", bag_path, "' != ''"])),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])

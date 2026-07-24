from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_pcd_path = LaunchConfiguration("map_pcd_path")
    visualization_map_pcd_path = LaunchConfiguration("visualization_map_pcd_path")
    icp_map_pcd_path = LaunchConfiguration("icp_map_pcd_path")

    default_config = PathJoinSubstitution([
        FindPackageShare("fast_anchor_localization"),
        "config",
        "fast_anchor_localization.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_pcd_path", default_value="maps/map_preprocessed.pcd"),
        DeclareLaunchArgument(
            "visualization_map_pcd_path",
            default_value="maps/map_visualization.pcd",
        ),
        DeclareLaunchArgument("icp_map_pcd_path", default_value="maps/map_preprocessed.pcd"),
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
                },
            ],
        ),
    ])

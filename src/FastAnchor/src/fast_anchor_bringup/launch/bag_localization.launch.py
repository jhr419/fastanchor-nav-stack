from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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

    return LaunchDescription([
        DeclareLaunchArgument("bag_path", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
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

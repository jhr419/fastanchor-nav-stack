from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Optional YAML file consumed by standalone PCD tool scripts.",
        ),
        LogInfo(msg=["PCD tools are standalone scripts. Config file: ", config_file]),
    ])

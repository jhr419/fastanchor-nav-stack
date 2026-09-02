from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 默认加载独立 Twist 控制配置，也允许调用者传入其他参数文件。
    default_config = str(
        Path(get_package_share_directory("genisom_l1_control"))
        / "config"
        / "twist.yaml"
    )
    config_argument = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="GENISOM Twist 控制参数文件",
    )

    twist_node = Node(
        package="genisom_l1_control",
        executable="genisom_twist_node",
        name="genisom_twist_control",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_argument, twist_node])

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 默认加载 Manager 安全配置，也允许调用者传入另一份参数文件。
    default_config = str(
        Path(get_package_share_directory("genisom_l1_control"))
        / "config"
        / "manager.yaml"
    )
    config_argument = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="GENISOM Manager 参数文件",
    )

    manager_node = Node(
        package="genisom_l1_control",
        executable="genisom_manager_node",
        name="genisom_manager",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_argument, manager_node])

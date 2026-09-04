import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch.actions import EmitEvent, TimerAction
from launch.events import Shutdown
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


@pytest.mark.launch_test
def generate_test_description():
    cropper = launch_ros.actions.Node(
        package="scan_planner",
        executable="global_path_window_node",
        name="global_path_window_node",
        parameters=[
            {
                "global_path_topic": "/test/global_path",
                "local_path_topic": "/test/local_path",
                "scan_planner_node_name": "/test/missing_scan_planner",
                "update_rate": 20.0,
            }
        ],
        output="screen",
    )
    shutdown_timer = TimerAction(
        period=10.0,
        actions=[EmitEvent(event=Shutdown(reason="全局路径清理测试结束"))],
    )
    return (
        launch.LaunchDescription(
            [cropper, launch_testing.actions.ReadyToTest(), shutdown_timer]
        ),
        {"cropper": cropper},
    )


class TestGlobalPathWindowClear(unittest.TestCase):
    def test_empty_global_path_clears_local_path(self):
        context = rclpy.context.Context()
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("global_path_window_clear_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        received_paths = []
        global_path_pub = node.create_publisher(Path, "/test/global_path", qos)
        local_path_sub = node.create_subscription(
            Path, "/test/local_path", received_paths.append, qos
        )

        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if (
                    global_path_pub.get_subscription_count() > 0
                    and node.count_publishers("/test/local_path") > 0
                ):
                    break
                executor.spin_once(timeout_sec=0.05)
            else:
                raise AssertionError("路径裁剪节点连接未在超时时间内建立")

            empty_path = Path()
            empty_path.header.frame_id = "map"
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                empty_path.header.stamp = node.get_clock().now().to_msg()
                global_path_pub.publish(empty_path)
                executor.spin_once(timeout_sec=0.05)
                if any(not path.poses for path in received_paths):
                    return

            raise AssertionError("未收到用于清理缓存的空局部路径")
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_subscription(local_path_sub)
            node.destroy_node()
            context.shutdown()

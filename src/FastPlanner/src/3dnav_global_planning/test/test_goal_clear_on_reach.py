import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from launch.actions import EmitEvent, TimerAction
from launch.events import Shutdown
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    map_path = os.path.join(os.path.dirname(__file__), "data", "goal_clear_map.pcd")
    planner = launch_ros.actions.Node(
        package="nav3d_global_planning",
        executable="astar_global_planner_node",
        name="astar_global_planner_node",
        parameters=[
            {
                "map_frame": "map",
                "start_source": "odom",
                "odom_topic": "/test/odom",
                "goal_pose_topic": "/test/goal",
                "goal_point_topic": "",
                "rviz_2d_goal_topic": "",
                "map_source": "pcd",
                "octomap_file": "",
                "pcd_file": map_path,
                "planning_mode": "2.5d",
                "unknown_as_occupied": False,
                "require_traversable_support": False,
                "enforce_obstacle_clearance": False,
                "terrain_following_enabled": False,
                "clearance_cost_enabled": False,
                "wall_clearance_cost_enabled": False,
                "tomogram_cost_enabled": False,
                "postprocess_enabled": False,
                "publish_path_topic": "/test/planned_path",
                "publish_alias_path_topic": "",
                "status_topic": "/test/planner_status",
                "publish_debug_markers": False,
                "clear_goal_on_reach": True,
                "goal_reached_tolerance": 0.15,
                "start_replan_min_interval_sec": 0.0,
            }
        ],
        output="screen",
    )
    shutdown_timer = TimerAction(
        period=15.0,
        actions=[EmitEvent(event=Shutdown(reason="A* 目标清理测试结束"))],
    )
    return (
        launch.LaunchDescription(
            [planner, launch_testing.actions.ReadyToTest(), shutdown_timer]
        ),
        {"planner": planner},
    )


class TestGoalClearOnReach(unittest.TestCase):
    def test_goal_is_not_replanned_after_reaching_it(self):
        context = rclpy.context.Context()
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("goal_clear_on_reach_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        paths = []
        statuses = []
        odom_pub = node.create_publisher(Odometry, "/test/odom", 20)
        goal_pub = node.create_publisher(PoseStamped, "/test/goal", latched_qos)
        path_sub = node.create_subscription(
            Path, "/test/planned_path", paths.append, latched_qos
        )
        status_sub = node.create_subscription(
            String, "/test/planner_status", statuses.append, latched_qos
        )

        try:
            self._wait_for_connections(node, executor, odom_pub, goal_pub)
            odom_pub.publish(self._make_odom(node, 0.0))
            goal_pub.publish(self._make_goal(node, 1.0))

            self._spin_until(
                executor,
                lambda: any(path.poses for path in paths),
                "A* 未发布测试目标的非空路径",
            )

            odom_pub.publish(self._make_odom(node, 0.90))
            self._spin_until(
                executor,
                lambda: any(not path.poses for path in paths)
                and any(status.data == "GOAL_REACHED" for status in statuses),
                "到达目标后未发布空路径或 GOAL_REACHED 状态",
            )

            nonempty_count = sum(bool(path.poses) for path in paths)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                odom_pub.publish(self._make_odom(node, -1.0))
                executor.spin_once(timeout_sec=0.05)

            self.assertEqual(
                sum(bool(path.poses) for path in paths),
                nonempty_count,
                "清理目标后机器人移开仍触发了旧目标重规划",
            )
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_subscription(path_sub)
            node.destroy_subscription(status_sub)
            node.destroy_node()
            context.shutdown()

    @staticmethod
    def _wait_for_connections(node, executor, odom_pub, goal_pub):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if (
                odom_pub.get_subscription_count() > 0
                and goal_pub.get_subscription_count() > 0
                and node.count_publishers("/test/planned_path") > 0
            ):
                return
            executor.spin_once(timeout_sec=0.05)
        raise AssertionError("A* 测试话题连接未在超时时间内建立")

    @staticmethod
    def _spin_until(executor, predicate, failure_message):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError(failure_message)

    @staticmethod
    def _make_odom(node, x):
        message = Odometry()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def _make_goal(node, x):
        message = PoseStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = x
        message.pose.orientation.w = 1.0
        return message

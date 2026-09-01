import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point, Twist
from launch.actions import EmitEvent, TimerAction
from launch.events import Shutdown
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from scan_planner_msgs.msg import Bspline


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        parameters=[
            {
                "motion_model": "nonholonomic",
                "forward_only": True,
                "time_forward": 0.2,
                "heading_error_threshold": 0.5,
                "kp_pos": 0.8,
                "kp_yaw": 1.5,
                "max_vx": 0.75,
                "max_vy": 0.35,
                "max_vyaw": 1.0,
                "finish_dist": 0.05,
            }
        ],
        output="screen",
    )
    shutdown_timer = TimerAction(
        period=7.0,
        actions=[EmitEvent(event=Shutdown(reason="闭环控制约束测试结束"))],
    )
    return (
        launch.LaunchDescription(
            [controller, launch_testing.actions.ReadyToTest(), shutdown_timer]
        ),
        {"controller": controller},
    )


class TestClosedLoopConstraints(unittest.TestCase):
    def test_published_command_is_nonholonomic(self):
        context = rclpy.context.Context()
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("closed_loop_constraints_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        commands = []
        command_sub = node.create_subscription(Twist, "/cmd_vel", commands.append, 20)
        odom_pub = node.create_publisher(Odometry, "/body_pose", 20)
        trajectory_pub = node.create_publisher(Bspline, "/planning/bspline", 10)

        try:
            self._wait_for_connections(executor, odom_pub, trajectory_pub)
            trajectory = self._make_diagonal_trajectory()
            trajectory_pub.publish(trajectory)
            odom = self._make_odometry(0.0)

            rotate_command = self._wait_for_command(
                node,
                executor,
                odom_pub,
                odom,
                commands,
                lambda command: command.angular.z > 0.05,
            )
            self.assertEqual(rotate_command.linear.x, 0.0)

            odom = self._make_odometry(math.pi / 4.0)
            forward_command = self._wait_for_command(
                node,
                executor,
                odom_pub,
                odom,
                commands,
                lambda command: command.linear.x > 0.05,
            )
            self.assertGreater(forward_command.linear.x, 0.0)

            self.assertTrue(commands)
            for command in commands:
                self.assertGreaterEqual(command.linear.x, 0.0)
                self.assertLessEqual(command.linear.x, 0.75)
                self.assertEqual(command.linear.y, 0.0)
                self.assertEqual(command.linear.z, 0.0)
                self.assertEqual(command.angular.x, 0.0)
                self.assertEqual(command.angular.y, 0.0)
                self.assertLessEqual(abs(command.angular.z), 1.0)
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_subscription(command_sub)
            node.destroy_node()
            context.shutdown()

    @staticmethod
    def _wait_for_connections(executor, odom_pub, trajectory_pub):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if (
                odom_pub.get_subscription_count() > 0
                and trajectory_pub.get_subscription_count() > 0
            ):
                return
            executor.spin_once(timeout_sec=0.05)
        raise AssertionError("控制器订阅连接未在超时时间内建立")

    @staticmethod
    def _wait_for_command(
        node,
        executor,
        odom_pub,
        odom,
        commands,
        predicate,
    ):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            odom.header.stamp = node.get_clock().now().to_msg()
            odom_pub.publish(odom)
            executor.spin_once(timeout_sec=0.05)
            for command in reversed(commands):
                if predicate(command):
                    return command
        raise AssertionError("未在超时时间内收到符合条件的速度命令")

    @staticmethod
    def _make_diagonal_trajectory():
        trajectory = Bspline()
        trajectory.order = 3
        trajectory.traj_id = 1
        trajectory.knots = [
            -0.3,
            -0.2,
            -0.1,
            0.0,
            0.1,
            0.2,
            0.3,
            0.4,
            0.5,
            0.6,
            0.7,
        ]
        trajectory.pos_pts = [
            Point(x=0.1 * index, y=0.1 * index, z=0.3)
            for index in range(7)
        ]
        return trajectory

    @staticmethod
    def _make_odometry(yaw):
        odom = Odometry()
        odom.pose.pose.position.z = 0.3
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return odom

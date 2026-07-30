"""Publish YAML-defined waypoints to the integrated global planner one at a time."""

from __future__ import annotations

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String

from navigation_bringup.waypoint_sequence import WaypointSequence, parse_waypoints


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class WaypointMissionManager(Node):
    """Turn a static waypoint list into a sequence of single-goal requests."""

    def __init__(self) -> None:
        super().__init__("waypoint_mission_manager")
        self.declare_parameter(
            "waypoints", descriptor=ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("odom_topic", "/fast_anchor/odom")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("status_topic", "/waypoint_mission/status")
        self.declare_parameter("xy_tolerance", 0.5)
        self.declare_parameter("z_tolerance", -1.0)
        self.declare_parameter("path_goal_tolerance", 0.75)
        self.declare_parameter("hold_time", 0.5)
        self.declare_parameter("loop", False)

        waypoints = parse_waypoints(self.get_parameter("waypoints").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._path_goal_tolerance = float(
            self.get_parameter("path_goal_tolerance").value
        )
        self._sequence = WaypointSequence(
            waypoints=waypoints,
            xy_tolerance=float(self.get_parameter("xy_tolerance").value),
            z_tolerance=float(self.get_parameter("z_tolerance").value),
            hold_time=float(self.get_parameter("hold_time").value),
            loop=bool(self.get_parameter("loop").value),
        )
        if not self._frame_id:
            raise ValueError("frame_id must not be empty")
        if not odom_topic or not goal_topic or not path_topic or not status_topic:
            raise ValueError(
                "odom_topic, goal_topic, path_topic and status_topic must not be empty"
            )
        if not math.isfinite(self._path_goal_tolerance) or self._path_goal_tolerance <= 0.0:
            raise ValueError("path_goal_tolerance must be greater than zero")

        self._goal_publisher = self.create_publisher(PoseStamped, goal_topic, latched_qos())
        self._status_publisher = self.create_publisher(String, status_topic, latched_qos())
        self._odom_subscription = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self._path_subscription = self.create_subscription(
            Path, path_topic, self._path_callback, latched_qos()
        )
        self._started = False
        self._goal_published_stamp_ns = 0
        self._publish_status("WAITING_FOR_ODOMETRY")
        self.get_logger().info(
            "Loaded %d waypoint(s); waiting for odometry on %s"
            % (len(waypoints), odom_topic)
        )

    def _publish_status(self, state: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                "current_index": (
                    None if self._sequence.completed else self._sequence.current_index
                ),
                "waypoint_count": len(self._sequence.waypoints),
            },
            separators=(",", ":"),
        )
        self._status_publisher.publish(message)

    def _publish_current_goal(self) -> None:
        waypoint = self._sequence.current_waypoint
        if waypoint is None:
            return
        goal = PoseStamped()
        goal_time = self.get_clock().now()
        goal.header.stamp = goal_time.to_msg()
        goal.header.frame_id = self._frame_id
        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.position.z = waypoint[2]
        goal.pose.orientation.w = 1.0
        self._goal_published_stamp_ns = goal_time.nanoseconds
        self._goal_publisher.publish(goal)
        self._publish_status("WAITING_FOR_PATH")
        self.get_logger().info(
            "Published waypoint %d/%d: [%.3f, %.3f, %.3f]"
            % (
                self._sequence.current_index + 1,
                len(self._sequence.waypoints),
                waypoint[0],
                waypoint[1],
                waypoint[2],
            )
        )

    def _path_callback(self, message: Path) -> None:
        waypoint = self._sequence.current_waypoint
        if not self._started or waypoint is None or not message.poses:
            return

        path_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if path_stamp_ns < self._goal_published_stamp_ns:
            return

        path_goal = message.poses[-1].pose.position
        goal_distance = (
            (float(path_goal.x) - waypoint[0]) ** 2
            + (float(path_goal.y) - waypoint[1]) ** 2
        ) ** 0.5
        if goal_distance > self._path_goal_tolerance:
            return

        if self._sequence.confirm_current_waypoint():
            self._publish_status("NAVIGATING")
            self.get_logger().info(
                "Confirmed FastPlanner path for waypoint %d/%d"
                % (self._sequence.current_index + 1, len(self._sequence.waypoints))
            )

    def _odom_callback(self, message: Odometry) -> None:
        if not self._started:
            self._started = True
            self._publish_current_goal()
            return

        position = message.pose.pose.position
        event = self._sequence.observe(
            (float(position.x), float(position.y), float(position.z)),
            self.get_clock().now().nanoseconds * 1.0e-9,
        )
        if event is None:
            return

        self.get_logger().info(
            "Reached waypoint %d/%d"
            % (event.reached_index + 1, len(self._sequence.waypoints))
        )
        if event.completed:
            self._publish_status("COMPLETED")
            self.get_logger().info("Waypoint mission completed")
            return

        if event.looped:
            self.get_logger().info("Restarting waypoint mission loop")
        self._publish_current_goal()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[WaypointMissionManager] = None
    try:
        node = WaypointMissionManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            get_logger("waypoint_mission_manager").fatal(str(error))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

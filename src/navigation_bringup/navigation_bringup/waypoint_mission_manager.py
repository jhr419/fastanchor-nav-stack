"""Dynamic waypoint mission manager with Action + pause/resume/cancel."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav_interfaces.action import FollowWaypoints
from nav_interfaces.srv import ControlMission
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool

from navigation_bringup.waypoint_sequence import WaypointSequence


PAUSE = 1
RESUME = 2


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class WaypointMissionManager(Node):

    def __init__(self) -> None:
        super().__init__("waypoint_mission_manager")

        # ------------------------------------------------------
        # Parameters
        # ------------------------------------------------------

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("odom_topic", "/fast_anchor/odom")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("status_topic", "/waypoint_mission/status")

        self.declare_parameter("action_name", "/follow_waypoints")
        self.declare_parameter(
            "control_service",
            "/waypoint_mission/control",
        )
        self.declare_parameter(
            "scan_enable_service",
            "/scan_planner/set_navigation_enabled",
        )

        self.declare_parameter("xy_tolerance", 1.5)
        self.declare_parameter("z_tolerance", -1.0)
        self.declare_parameter("path_goal_tolerance", 0.75)
        self.declare_parameter("hold_time", 0.1)

        self._configured_frame = str(
            self.get_parameter("frame_id").value
        )
        self._odom_topic = str(
            self.get_parameter("odom_topic").value
        )
        self._goal_topic = str(
            self.get_parameter("goal_topic").value
        )
        self._path_topic = str(
            self.get_parameter("path_topic").value
        )
        self._status_topic = str(
            self.get_parameter("status_topic").value
        )
        self._action_name = str(
            self.get_parameter("action_name").value
        )
        self._control_service_name = str(
            self.get_parameter("control_service").value
        )
        self._scan_service_name = str(
            self.get_parameter("scan_enable_service").value
        )

        self._xy_tolerance = float(
            self.get_parameter("xy_tolerance").value
        )
        self._z_tolerance = float(
            self.get_parameter("z_tolerance").value
        )
        self._path_goal_tolerance = float(
            self.get_parameter("path_goal_tolerance").value
        )
        self._hold_time = float(
            self.get_parameter("hold_time").value
        )

        if not self._configured_frame:
            raise ValueError("frame_id must not be empty")

        if (
            not math.isfinite(self._path_goal_tolerance)
            or self._path_goal_tolerance <= 0.0
        ):
            raise ValueError(
                "path_goal_tolerance must be greater than zero"
            )

        # ------------------------------------------------------
        # Concurrency
        # ------------------------------------------------------

        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        # ------------------------------------------------------
        # Mission state
        # ------------------------------------------------------

        self._sequence: Optional[WaypointSequence] = None
        self._mission_id = ""
        self._mission_frame = self._configured_frame

        self._state = "IDLE"

        self._goal_reserved = False
        self._active_goal_handle = None

        self._paused = False
        self._pause_in_progress = False
        self._resume_in_progress = False
        self._state_before_pause = "NAVIGATING"

        self._odom_received = False
        self._goal_published_stamp_ns = 0

        self._mission_done = threading.Event()
        self._mission_success = False
        self._mission_message = ""

        # ------------------------------------------------------
        # ROS I/O
        # ------------------------------------------------------

        self._goal_pub = self.create_publisher(
            PoseStamped,
            self._goal_topic,
            latched_qos(),
        )

        self._status_pub = self.create_publisher(
            String,
            self._status_topic,
            latched_qos(),
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )

        self._path_sub = self.create_subscription(
            Path,
            self._path_topic,
            self._path_callback,
            latched_qos(),
            callback_group=self._callback_group,
        )

        self._scan_client = self.create_client(
            SetBool,
            self._scan_service_name,
            callback_group=self._callback_group,
        )

        self._control_srv = self.create_service(
            ControlMission,
            self._control_service_name,
            self._control_callback,
            callback_group=self._callback_group,
        )

        self._action_server = ActionServer(
            self,
            FollowWaypoints,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._publish_status("IDLE")

        self.get_logger().info(
            "Waypoint mission manager ready"
        )
        self.get_logger().info(
            "  Action:  %s" % self._action_name
        )
        self.get_logger().info(
            "  Control: %s" % self._control_service_name
        )
        self.get_logger().info(
            "  SCAN:    %s" % self._scan_service_name
        )

    # =========================================================
    # Helpers
    # =========================================================

    def _progress_locked(self) -> float:
        if self._sequence is None:
            return 0.0

        total = len(self._sequence.waypoints)

        if total <= 0:
            return 0.0

        if self._sequence.completed:
            return 1.0

        return float(self._sequence.current_index) / float(total)

    def _publish_status(self, state: Optional[str] = None) -> None:
        with self._lock:
            if state is not None:
                self._state = state

            sequence = self._sequence

            current_index = None
            waypoint_count = 0
            progress = 0.0

            if sequence is not None:
                waypoint_count = len(sequence.waypoints)
                progress = self._progress_locked()

                if not sequence.completed:
                    current_index = sequence.current_index

            payload = {
                "mission_id": self._mission_id,
                "state": self._state,
                "current_index": current_index,
                "waypoint_count": waypoint_count,
                "progress": progress,
                "paused": self._paused,
            }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )
        self._status_pub.publish(msg)

    def _feedback_snapshot(self) -> FollowWaypoints.Feedback:
        feedback = FollowWaypoints.Feedback()

        with self._lock:
            if self._sequence is None:
                feedback.current_index = 0
                feedback.total_waypoints = 0
                feedback.progress = 0.0
            else:
                feedback.current_index = int(
                    self._sequence.current_index
                )
                feedback.total_waypoints = int(
                    len(self._sequence.waypoints)
                )
                feedback.progress = float(
                    self._progress_locked()
                )

            feedback.state = self._state

        return feedback

    def _validate_goal(
        self,
        request: FollowWaypoints.Goal,
    ) -> Tuple[bool, str]:

        if not request.waypoints:
            return False, "waypoints must not be empty"

        frame = request.frame_id.strip()

        if frame and frame != self._configured_frame:
            return (
                False,
                "frame_id '%s' is unsupported; expected '%s'"
                % (frame, self._configured_frame),
            )

        for index, point in enumerate(request.waypoints):
            values = (
                float(point.x),
                float(point.y),
                float(point.z),
            )

            if not all(math.isfinite(v) for v in values):
                return (
                    False,
                    "waypoint %d contains non-finite coordinates"
                    % index,
                )

        return True, ""

    # =========================================================
    # Action goal / cancel
    # =========================================================

    def _goal_callback(
        self,
        request: FollowWaypoints.Goal,
    ) -> GoalResponse:

        valid, reason = self._validate_goal(request)

        if not valid:
            self.get_logger().warning(
                "Reject waypoint mission: %s" % reason
            )
            return GoalResponse.REJECT

        with self._lock:
            if self._goal_reserved:
                self.get_logger().warning(
                    "Reject waypoint mission: another mission is active"
                )
                return GoalResponse.REJECT

            # Reserve immediately, before execute_callback begins,
            # to avoid two nearly simultaneous accepted goals.
            self._goal_reserved = True

        self.get_logger().info(
            "Accepted waypoint mission request with %d waypoint(s)"
            % len(request.waypoints)
        )

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        with self._lock:
            if (
                self._active_goal_handle is None
                or goal_handle != self._active_goal_handle
            ):
                return CancelResponse.REJECT

        self.get_logger().info(
            "Action cancel request accepted"
        )
        return CancelResponse.ACCEPT

    # =========================================================
    # SCAN service
    # =========================================================

    def _call_scan_and_wait(
        self,
        enabled: bool,
        timeout_sec: float = 4.0,
    ) -> Tuple[bool, str]:

        if not self._scan_client.wait_for_service(
            timeout_sec=1.0
        ):
            return (
                False,
                "SCAN navigation-enable service unavailable",
            )

        request = SetBool.Request()
        request.data = enabled

        future = self._scan_client.call_async(request)

        deadline = time.monotonic() + timeout_sec

        while not future.done():
            if time.monotonic() >= deadline:
                return (
                    False,
                    "Timed out waiting for SCAN service",
                )
            time.sleep(0.02)

        try:
            result = future.result()
        except Exception as error:
            return False, str(error)

        if result is None:
            return False, "SCAN returned no response"

        return bool(result.success), str(result.message)

    # =========================================================
    # Goal publication
    # =========================================================

    def _publish_current_goal(self) -> bool:

        with self._lock:
            if (
                self._sequence is None
                or self._sequence.completed
                or self._paused
            ):
                return False

            waypoint = self._sequence.current_waypoint

            if waypoint is None:
                return False

            index = self._sequence.current_index
            total = len(self._sequence.waypoints)
            frame = self._mission_frame

            now = self.get_clock().now()
            self._goal_published_stamp_ns = now.nanoseconds

            self._state = "WAITING_FOR_PATH"

        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = frame

        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.position.z = waypoint[2]
        goal.pose.orientation.w = 1.0

        self._goal_pub.publish(goal)
        self._publish_status()

        self.get_logger().info(
            "Published waypoint %d/%d: [%.3f, %.3f, %.3f]"
            % (
                index + 1,
                total,
                waypoint[0],
                waypoint[1],
                waypoint[2],
            )
        )

        return True

    # =========================================================
    # Path callback
    # =========================================================

    def _path_callback(self, message: Path) -> None:

        with self._lock:
            if (
                self._sequence is None
                or self._paused
                or self._sequence.completed
                or not message.poses
            ):
                return

            waypoint = self._sequence.current_waypoint

            if waypoint is None:
                return

            goal_stamp = self._goal_published_stamp_ns

        path_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

        if path_stamp_ns < goal_stamp:
            return

        path_goal = message.poses[-1].pose.position

        goal_distance = math.hypot(
            float(path_goal.x) - waypoint[0],
            float(path_goal.y) - waypoint[1],
        )

        if goal_distance > self._path_goal_tolerance:
            return

        with self._lock:
            if (
                self._sequence is None
                or self._paused
            ):
                return

            if self._sequence.confirm_current_waypoint():
                self._state = "NAVIGATING"
                index = self._sequence.current_index
                total = len(self._sequence.waypoints)
            else:
                return

        self._publish_status()

        self.get_logger().info(
            "Confirmed fresh FastPlanner path for waypoint %d/%d"
            % (index + 1, total)
        )

    # =========================================================
    # Odometry callback
    # =========================================================

    def _odom_callback(self, message: Odometry) -> None:

        should_publish_first_goal = False

        with self._lock:
            first_odom = not self._odom_received
            self._odom_received = True

            if (
                first_odom
                and self._sequence is not None
                and not self._paused
                and self._state == "WAITING_FOR_ODOMETRY"
            ):
                should_publish_first_goal = True

        if should_publish_first_goal:
            self._publish_current_goal()
            return

        position = message.pose.pose.position

        with self._lock:
            if (
                self._sequence is None
                or self._paused
                or self._sequence.completed
            ):
                return

            event = self._sequence.observe(
                (
                    float(position.x),
                    float(position.y),
                    float(position.z),
                ),
                self.get_clock().now().nanoseconds * 1.0e-9,
            )

            if event is None:
                return

            total = len(self._sequence.waypoints)

            if event.completed:
                self._mission_success = True
                self._mission_message = (
                    "Final waypoint reached; stopping navigation"
                )
                self._state = "COMPLETING"
                self._mission_done.set()

                reached_index = event.reached_index
                publish_next = False
            else:
                reached_index = event.reached_index
                publish_next = True

        self.get_logger().info(
            "Reached waypoint %d/%d"
            % (reached_index + 1, total)
        )

        self._publish_status()

        if publish_next:
            self._publish_current_goal()

    # =========================================================
    # Pause / Resume
    # =========================================================

    def _request_pause(self) -> Tuple[bool, str]:

        with self._lock:
            if self._sequence is None:
                return False, "No active mission"

            if self._pause_in_progress:
                return True, "Pause already in progress"

            if self._paused:
                return True, "Mission already paused"

            if self._resume_in_progress:
                return False, "Resume is in progress"

            self._state_before_pause = self._state
            self._paused = True
            self._pause_in_progress = True

            self._sequence.reset_current_confirmation()
            self._state = "PAUSING"

        self._publish_status()

        request = SetBool.Request()
        request.data = False

        if not self._scan_client.wait_for_service(
            timeout_sec=0.5
        ):
            with self._lock:
                self._pause_in_progress = False
                self._paused = False
                self._state = self._state_before_pause

            self._publish_status()
            return (
                False,
                "SCAN navigation-enable service unavailable",
            )

        future = self._scan_client.call_async(request)
        future.add_done_callback(self._pause_done)

        return True, "Pause requested"

    def _pause_done(self, future) -> None:

        try:
            result = future.result()
            success = result is not None and result.success
            message = (
                "no response"
                if result is None
                else result.message
            )
        except Exception as error:
            success = False
            message = str(error)

        with self._lock:
            self._pause_in_progress = False

            if success:
                self._paused = True
                self._state = "PAUSED"
            else:
                self._paused = False

                if (
                    self._sequence is not None
                    and self._state_before_pause == "NAVIGATING"
                ):
                    self._sequence.confirm_current_waypoint()

                self._state = self._state_before_pause

        self._publish_status()

        if success:
            self.get_logger().info(
                "Mission paused"
            )
        else:
            self.get_logger().error(
                "Pause failed: %s" % message
            )

    def _request_resume(self) -> Tuple[bool, str]:

        with self._lock:
            if self._sequence is None:
                return False, "No active mission"

            if self._resume_in_progress:
                return True, "Resume already in progress"

            if self._pause_in_progress:
                return False, "Pause is still in progress"

            if not self._paused:
                return True, "Mission already running"

            self._resume_in_progress = True
            self._state = "RESUMING"

        self._publish_status()

        request = SetBool.Request()
        request.data = True

        if not self._scan_client.wait_for_service(
            timeout_sec=0.5
        ):
            with self._lock:
                self._resume_in_progress = False
                self._state = "PAUSED"

            self._publish_status()
            return (
                False,
                "SCAN navigation-enable service unavailable",
            )

        future = self._scan_client.call_async(request)
        future.add_done_callback(self._resume_done)

        return True, "Resume requested"

    def _resume_done(self, future) -> None:

        try:
            result = future.result()
            success = result is not None and result.success
            message = (
                "no response"
                if result is None
                else result.message
            )
        except Exception as error:
            success = False
            message = str(error)

        with self._lock:
            self._resume_in_progress = False

            if not success:
                self._paused = True
                self._state = "PAUSED"
            else:
                self._paused = False

                if self._sequence is not None:
                    self._sequence.reset_current_confirmation()

                if self._odom_received:
                    publish_goal = True
                else:
                    publish_goal = False
                    self._state = "WAITING_FOR_ODOMETRY"

        if not success:
            self._publish_status()
            self.get_logger().error(
                "Resume failed: %s" % message
            )
            return

        self.get_logger().info(
            "Mission resumed; replanning current waypoint"
        )

        if publish_goal:
            self._publish_current_goal()
        else:
            self._publish_status()

    def _control_callback(
        self,
        request: ControlMission.Request,
        response: ControlMission.Response,
    ) -> ControlMission.Response:

        requested_id = request.mission_id.strip()

        with self._lock:
            current_id = self._mission_id
            current_state = self._state

        if requested_id and requested_id != current_id:
            response.success = False
            response.message = (
                "Mission ID mismatch; current mission is '%s'"
                % current_id
            )
            response.state = current_state
            return response

        command = int(request.command)

        if command == PAUSE:
            success, message = self._request_pause()
        elif command == RESUME:
            success, message = self._request_resume()
        else:
            success = False
            message = (
                "Unsupported command %d; PAUSE=1, RESUME=2"
                % command
            )

        with self._lock:
            state = self._state

        response.success = success
        response.message = message
        response.state = state

        return response

    # =========================================================
    # Action execution
    # =========================================================

    def _execute_callback(
        self,
        goal_handle,
    ) -> FollowWaypoints.Result:

        request = goal_handle.request

        points = tuple(
            (
                float(point.x),
                float(point.y),
                float(point.z),
            )
            for point in request.waypoints
        )

        mission_id = request.mission_id.strip()

        if not mission_id:
            mission_id = (
                "mission_%d"
                % int(time.time() * 1000.0)
            )

        mission_frame = (
            request.frame_id.strip()
            or self._configured_frame
        )

        sequence = WaypointSequence(
            waypoints=points,
            xy_tolerance=self._xy_tolerance,
            z_tolerance=self._z_tolerance,
            hold_time=self._hold_time,
            loop=False,
        )

        with self._lock:
            self._active_goal_handle = goal_handle
            self._sequence = sequence
            self._mission_id = mission_id
            self._mission_frame = mission_frame

            self._paused = False
            self._pause_in_progress = False
            self._resume_in_progress = False

            self._mission_done.clear()
            self._mission_success = False
            self._mission_message = ""

            self._state = "STARTING"

        self._publish_status()

        self.get_logger().info(
            "Starting mission '%s' with %d waypoint(s)"
            % (mission_id, len(points))
        )

        enabled, enable_message = self._call_scan_and_wait(
            True
        )

        if not enabled:
            result = FollowWaypoints.Result()
            result.success = False
            result.message = (
                "Cannot start mission: %s"
                % enable_message
            )

            goal_handle.abort()
            self._cleanup_after_goal("FAILED")
            return result

        with self._lock:
            odom_ready = self._odom_received

            if not odom_ready:
                self._state = "WAITING_FOR_ODOMETRY"

        if odom_ready:
            self._publish_current_goal()
        else:
            self._publish_status()

        # ------------------------------------------------------
        # Long-running Action lifecycle
        # ------------------------------------------------------

        while rclpy.ok():

            if goal_handle.is_cancel_requested:

                with self._lock:
                    self._paused = True
                    self._state = "CANCELING"

                    if self._sequence is not None:
                        self._sequence.reset_current_confirmation()

                self._publish_status()

                stopped, stop_message = (
                    self._call_scan_and_wait(False)
                )

                result = FollowWaypoints.Result()
                result.success = False

                if stopped:
                    result.message = "Mission canceled"
                else:
                    result.message = (
                        "Mission canceled, but SCAN stop failed: %s"
                        % stop_message
                    )

                goal_handle.canceled()

                self._cleanup_after_goal("CANCELED")
                return result

            if self._mission_done.is_set():
                break

            goal_handle.publish_feedback(
                self._feedback_snapshot()
            )

            time.sleep(0.1)

        with self._lock:
            success = self._mission_success
            message = self._mission_message or (
                "Mission terminated"
            )

        if success:
            stopped, stop_message = self._call_scan_and_wait(False)

            if stopped:
                message = "Waypoint mission completed"
                with self._lock:
                    self._state = "COMPLETED"
                    self._mission_message = message
            else:
                success = False
                message = (
                    "Final waypoint reached, but SCAN stop failed: %s"
                    % stop_message
                )
                with self._lock:
                    self._state = "FAILED"
                    self._mission_success = False
                    self._mission_message = message

        result = FollowWaypoints.Result()
        result.success = success
        result.message = message

        if success:
            goal_handle.succeed()
            final_state = "SUCCEEDED"
        else:
            goal_handle.abort()
            final_state = "FAILED"

        # Final feedback/status before resetting to IDLE.
        goal_handle.publish_feedback(
            self._feedback_snapshot()
        )

        self._cleanup_after_goal(final_state)

        return result

    def _cleanup_after_goal(self, terminal_state: str) -> None:

        with self._lock:
            self._state = terminal_state

        self._publish_status()

        with self._lock:
            self._sequence = None
            self._active_goal_handle = None
            self._goal_reserved = False

            self._paused = False
            self._pause_in_progress = False
            self._resume_in_progress = False

            self._mission_done.clear()
            self._mission_success = False
            self._mission_message = ""

            self._mission_id = ""
            self._mission_frame = self._configured_frame

            self._state = "IDLE"

        self._publish_status()

    def destroy_node(self):
        if hasattr(self, "_action_server"):
            self._action_server.destroy()

        return super().destroy_node()


def main(args=None) -> None:

    rclpy.init(args=args)

    node = WaypointMissionManager()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

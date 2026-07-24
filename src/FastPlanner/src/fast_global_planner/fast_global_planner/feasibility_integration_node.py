"""ROS 2 goal gate and path feasibility validator for every planner backend."""

import csv
import math
from pathlib import Path as FilePath
import threading
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, String
import tf2_ros
from visualization_msgs.msg import Marker
import yaml

from fast_planner_common.feasibility_checker import FeasibilityChecker, FeasibilityReport
from fast_planner_common.project_paths import resolve_fast_planner_path


Point3 = Tuple[float, float, float]


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class FastGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('fast_global_planner_node')
        self._declare_parameters()
        self._read_parameters()
        self._lock = threading.Lock()
        self.last_odom: Optional[Odometry] = None
        self.last_start_pose: Optional[PoseStamped] = None
        self.last_goal: Optional[Point3] = None
        self.last_pre_report: Optional[FeasibilityReport] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._create_interfaces()

        self.checker = FeasibilityChecker(
            tomogram_file=self.tomogram_file,
            pcd_file=self.pcd_file,
            octomap_file=self.octomap_file,
            traversable_cost_threshold=self.traversable_cost_threshold,
            clearance_threshold=self.feasibility_clearance_threshold,
            path_sample_resolution=self.path_check_resolution,
            max_path_segment_length=self.max_path_segment_length,
            max_ground_z_error=self.max_ground_z_error,
            obstacle_min_relative_z=self.obstacle_min_relative_z,
            obstacle_max_relative_z=self.obstacle_max_relative_z,
            clearance_search_radius=self.clearance_search_radius,
        )
        loaded = self.checker.load()
        self.get_logger().info(
            'Feasibility check: %s' % ('ENABLED' if self.enable_feasibility_check else 'DISABLED')
        )
        self.get_logger().info('Mode: %s' % self.feasibility_check_mode)
        if loaded:
            summary = self.checker.map_summary()
            tomo = summary.get('tomogram', {})
            self.get_logger().info(
                'Feasibility maps loaded: pcd_points=%d tomogram_shape=%s octomap=%s'
                % (
                    summary.get('pcd', {}).get('finite_point_count', 0),
                    tomo.get('shape', []),
                    'present' if summary.get('octomap_exists') else 'not used/missing',
                )
            )
            self._publish_status('READY feasibility_map_loaded=true')
        else:
            self.get_logger().error('Feasibility map load failed: %s' % self.checker.load_error)
            self._publish_status('MAP_ERROR reason=%s' % self.checker.load_error)

    def _declare_parameters(self):
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'start_source': 'tf',
            'start_pose_topic': '/fast_global_planner/start_pose',
            'odom_topic': '/odom',
            'tf_lookup_timeout': 0.25,
            'default_goal_z': 0.0,
            'goal_pose_topic': '/goal_pose_3d',
            'goal_point_topic': '/goal_point_3d',
            'rviz_2d_goal_topic': '/goal_pose',
            'validated_goal_pose_topic': '/fast_global_planner/validated_goal_pose_3d',
            'validated_goal_point_topic': '/fast_global_planner/validated_goal_point_3d',
            'validated_rviz_goal_topic': '/fast_global_planner/validated_goal_pose_2d',
            'backend_start_pose_topic': '/fast_global_planner/backend_start_pose',
            'raw_path_topic': '/fast_global_planner/raw_path',
            'publish_path_topic': '/planned_path',
            'publish_alias_path_topic': '/path',
            'publish_marker_topic': '/planned_path_marker',
            'feasibility_status_topic': '/fast_global_planner/feasibility_status',
            'feasibility_marker_topic': '/fast_global_planner/feasibility_marker',
            'enable_feasibility_check': True,
            'feasibility_check_mode': 'post',
            'feasibility_clearance_threshold': 0.2,
            'fail_on_infeasible_path': True,
            'publish_feasibility_debug': True,
            'tomogram_file': '',
            'pcd_file': '',
            'octomap_file': '',
            'traversable_cost_threshold': 25.0,
            'path_check_resolution': 0.10,
            'max_path_segment_length': 1.0,
            'max_ground_z_error': 0.50,
            'obstacle_min_relative_z': 0.20,
            'obstacle_max_relative_z': 1.40,
            'clearance_search_radius': 1.50,
            'debug_output_dir': 'debug/feasibility',
            'feasibility_report_file': 'feasibility_report.yaml',
            'feasibility_points_file': 'feasibility_points.csv',
            'marker_point_scale': 0.10,
            'path_marker_line_width': 0.08,
            'max_marker_points': 5000,
            'enforce_start_to_goal_order': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        for name in (
            'map_frame', 'base_frame', 'start_source', 'start_pose_topic', 'odom_topic',
            'goal_pose_topic', 'goal_point_topic', 'rviz_2d_goal_topic',
            'validated_goal_pose_topic', 'validated_goal_point_topic',
            'validated_rviz_goal_topic', 'raw_path_topic', 'publish_path_topic',
            'backend_start_pose_topic',
            'publish_alias_path_topic', 'publish_marker_topic', 'feasibility_status_topic',
            'feasibility_marker_topic', 'tomogram_file', 'pcd_file', 'octomap_file',
            'debug_output_dir', 'feasibility_report_file', 'feasibility_points_file',
        ):
            setattr(self, name, str(self.get_parameter(name).value))
        for name in (
            'tf_lookup_timeout', 'default_goal_z', 'feasibility_clearance_threshold',
            'traversable_cost_threshold', 'path_check_resolution', 'max_path_segment_length',
            'max_ground_z_error', 'obstacle_min_relative_z', 'obstacle_max_relative_z',
            'clearance_search_radius', 'marker_point_scale', 'path_marker_line_width',
        ):
            setattr(self, name, float(self.get_parameter(name).value))
        for name in (
            'enable_feasibility_check', 'fail_on_infeasible_path',
            'publish_feasibility_debug', 'enforce_start_to_goal_order',
        ):
            setattr(self, name, as_bool(self.get_parameter(name).value))
        self.max_marker_points = max(0, int(self.get_parameter('max_marker_points').value))
        self.start_source = self.start_source.strip().lower()
        self.feasibility_check_mode = str(
            self.get_parameter('feasibility_check_mode').value
        ).strip().lower()
        if self.feasibility_check_mode not in ('pre', 'post', 'both'):
            self.get_logger().warning(
                "Unknown feasibility_check_mode='%s'; using post." % self.feasibility_check_mode
            )
            self.feasibility_check_mode = 'post'
        self.debug_output_path = resolve_fast_planner_path(self.debug_output_dir)

    def _create_interfaces(self):
        qos = latched_qos()
        self.path_pub = self.create_publisher(Path, self.publish_path_topic, qos)
        self.alias_path_pub = (
            self.create_publisher(Path, self.publish_alias_path_topic, qos)
            if self.publish_alias_path_topic and self.publish_alias_path_topic != self.publish_path_topic
            else None
        )
        self.path_marker_pub = self.create_publisher(Marker, self.publish_marker_topic, qos)
        self.status_pub = self.create_publisher(String, self.feasibility_status_topic, qos)
        self.feasibility_marker_pub = self.create_publisher(
            Marker, self.feasibility_marker_topic, qos
        )
        self.validated_goal_pose_pub = self.create_publisher(
            PoseStamped, self.validated_goal_pose_topic, qos
        )
        self.validated_goal_point_pub = self.create_publisher(
            PointStamped, self.validated_goal_point_topic, qos
        )
        self.validated_rviz_goal_pub = self.create_publisher(
            PoseStamped, self.validated_rviz_goal_topic, qos
        )
        self.backend_start_pub = self.create_publisher(
            PoseStamped, self.backend_start_pose_topic, qos
        )
        self.raw_path_sub = self.create_subscription(
            Path, self.raw_path_topic, self._raw_path_callback, qos
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            lambda msg: self._pose_goal_callback(msg, False, self.validated_goal_pose_pub),
            qos,
        )
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            self.rviz_2d_goal_topic,
            lambda msg: self._pose_goal_callback(msg, True, self.validated_rviz_goal_pub),
            qos,
        )
        self.goal_point_sub = self.create_subscription(
            PointStamped, self.goal_point_topic, self._point_goal_callback, qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos
        )
        self.start_pose_sub = self.create_subscription(
            PoseStamped, self.start_pose_topic, self._start_pose_callback, qos
        )

    def _odom_callback(self, msg: Odometry):
        self.last_odom = msg
        if self.start_source != 'odom':
            return
        point = msg.pose.pose.position
        transformed = self._transform_xyz((point.x, point.y, point.z), msg.header.frame_id)
        if transformed is not None:
            self._publish_backend_start(transformed, msg.pose.pose.orientation)

    def _start_pose_callback(self, msg: PoseStamped):
        self.last_start_pose = msg
        if self.start_source != 'topic':
            return
        point = msg.pose.position
        transformed = self._transform_xyz((point.x, point.y, point.z), msg.header.frame_id)
        if transformed is not None:
            self._publish_backend_start(transformed, msg.pose.orientation)

    def _publish_backend_start(self, xyz: Point3, orientation):
        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.map_frame
        output.pose.position.x, output.pose.position.y, output.pose.position.z = xyz
        output.pose.orientation = orientation
        if (
            abs(orientation.x) + abs(orientation.y) + abs(orientation.z) + abs(orientation.w)
            < 1.0e-9
        ):
            output.pose.orientation.w = 1.0
        self.backend_start_pub.publish(output)

    def _pose_goal_callback(self, msg: PoseStamped, force_default_z: bool, publisher):
        xyz = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            self.default_goal_z if force_default_z else float(msg.pose.position.z),
        )
        transformed = self._transform_xyz(xyz, msg.header.frame_id)
        if transformed is None:
            self._publish_status('PRECHECK_TF_ERROR role=goal')
            return
        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.map_frame
        output.pose = msg.pose
        output.pose.position.x, output.pose.position.y, output.pose.position.z = transformed
        if self._precheck_and_accept(transformed):
            publisher.publish(output)

    def _point_goal_callback(self, msg: PointStamped):
        xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        transformed = self._transform_xyz(xyz, msg.header.frame_id)
        if transformed is None:
            self._publish_status('PRECHECK_TF_ERROR role=goal')
            return
        output = PointStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.map_frame
        output.point.x, output.point.y, output.point.z = transformed
        if self._precheck_and_accept(transformed):
            self.validated_goal_point_pub.publish(output)

    def _precheck_and_accept(self, goal: Point3) -> bool:
        self.last_goal = goal
        if not self.enable_feasibility_check or self.feasibility_check_mode == 'post':
            return True
        start = self._current_start()
        if start is None:
            self._publish_status('PRECHECK_FAILED reason=start_unavailable')
            if self.fail_on_infeasible_path:
                self._clear_path()
                return False
            return True
        report = self.checker.check_poses(start, goal)
        self.last_pre_report = report
        self._emit_report(report)
        if report.feasible:
            self._publish_status(self._status_text('PRECHECK_FEASIBLE', report))
            return True
        self._publish_status(self._status_text('PRECHECK_INFEASIBLE', report))
        if self.fail_on_infeasible_path:
            self._clear_path()
            return False
        return True

    def _raw_path_callback(self, msg: Path):
        with self._lock:
            path = self._normalize_path(msg)
            if not path.poses:
                self._publish_status('PATH_CLEARED reason=empty_raw_path')
                self._clear_path()
                return
            if not self.enable_feasibility_check or self.feasibility_check_mode == 'pre':
                self._publish_path(path)
                label = 'PATH_PUBLISHED_CHECK_DISABLED' if not self.enable_feasibility_check else (
                    'PATH_PUBLISHED_PRECHECK_ONLY'
                )
                self._publish_status(label)
                return
            points = [
                (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                for pose in path.poses
            ]
            report = self.checker.check(points, check_type='post', check_connectivity=True)
            self._emit_report(report)
            if report.feasible:
                self._publish_path(path)
                self._publish_status(self._status_text('PATH_FEASIBLE', report))
                return
            self._publish_status(self._status_text('PATH_INFEASIBLE', report))
            if self.fail_on_infeasible_path:
                self._clear_path()
            else:
                self._publish_path(path)

    def _normalize_path(self, msg: Path) -> Path:
        points = []
        for pose in msg.poses:
            transformed = self._transform_xyz(
                (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                pose.header.frame_id or msg.header.frame_id,
            )
            if transformed is None:
                continue
            points.append(transformed)
        if (
            self.enforce_start_to_goal_order
            and self.last_goal is not None
            and len(points) > 1
            and self._distance(points[0], self.last_goal) < self._distance(points[-1], self.last_goal)
        ):
            points.reverse()
        return self._path_message(points)

    def _path_message(self, points: Sequence[Point3]) -> Path:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            if index + 1 < len(points):
                neighbor = points[index + 1]
                yaw = math.atan2(neighbor[1] - point[1], neighbor[0] - point[0])
            elif index:
                neighbor = points[index - 1]
                yaw = math.atan2(point[1] - neighbor[1], point[0] - neighbor[0])
            else:
                yaw = 0.0
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            msg.poses.append(pose)
        return msg

    def _publish_path(self, path: Path):
        self.path_pub.publish(path)
        if self.alias_path_pub is not None:
            self.alias_path_pub.publish(path)
        marker = Marker()
        marker.header = path.header
        marker.ns = 'fast_global_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.path_marker_line_width
        marker.color = self._color(0.05, 0.85, 1.0, 1.0)
        marker.points = [pose.pose.position for pose in path.poses]
        self.path_marker_pub.publish(marker)

    def _clear_path(self):
        empty = self._path_message([])
        self.path_pub.publish(empty)
        if self.alias_path_pub is not None:
            self.alias_path_pub.publish(empty)
        marker = Marker()
        marker.header = empty.header
        marker.ns = 'fast_global_path'
        marker.id = 0
        marker.action = Marker.DELETE
        self.path_marker_pub.publish(marker)

    def _emit_report(self, report: FeasibilityReport):
        if self.publish_feasibility_debug:
            self._publish_feasibility_marker(report)
            self._write_report(report)

    def _publish_feasibility_marker(self, report: FeasibilityReport):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = 'fast_planner_feasibility'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_point_scale
        marker.scale.y = self.marker_point_scale
        points = report.points
        if self.max_marker_points and len(points) > self.max_marker_points:
            stride = int(math.ceil(len(points) / float(self.max_marker_points)))
            points = points[::stride]
        for result in points:
            point = Point()
            point.x, point.y, point.z = result.xyz
            marker.points.append(point)
            if result.collision:
                marker.colors.append(self._color(1.0, 0.02, 0.02, 0.98))
            elif result.feasible:
                marker.colors.append(self._color(0.0, 0.9, 0.15, 0.95))
            else:
                marker.colors.append(self._color(1.0, 0.65, 0.0, 0.98))
        self.feasibility_marker_pub.publish(marker)

    def _write_report(self, report: FeasibilityReport):
        try:
            self.debug_output_path.mkdir(parents=True, exist_ok=True)
            payload = {
                'frame_id': self.map_frame,
                'enable_feasibility_check': self.enable_feasibility_check,
                'feasibility_check_mode': self.feasibility_check_mode,
                'feasibility_clearance_threshold': self.feasibility_clearance_threshold,
                'fail_on_infeasible_path': self.fail_on_infeasible_path,
                'map': self.checker.map_summary(),
                'result': report.as_dict(),
            }
            report_path = self.debug_output_path / self.feasibility_report_file
            report_path.write_text(
                yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding='utf-8'
            )
            csv_path = self.debug_output_path / self.feasibility_points_file
            with csv_path.open('w', newline='', encoding='utf-8') as handle:
                fieldnames = [
                    'index', 'source_segment', 'x', 'y', 'z', 'feasible', 'collision',
                    'clearance', 'traversable', 'traversal_cost', 'ground_z',
                    'pcd_bounds_ok', 'tomogram_bounds_ok', 'reason',
                ]
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(point.as_dict() for point in report.points)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('Could not write feasibility debug files: %s' % exc)

    def _current_start(self) -> Optional[Point3]:
        if self.start_source == 'tf':
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout),
                )
                t = transform.transform.translation
                return (float(t.x), float(t.y), float(t.z))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning('Cannot obtain planner start from TF: %s' % exc)
                return None
        if self.start_source == 'odom' and self.last_odom is not None:
            pose = self.last_odom.pose.pose.position
            return self._transform_xyz(
                (pose.x, pose.y, pose.z), self.last_odom.header.frame_id
            )
        if self.start_source == 'topic' and self.last_start_pose is not None:
            pose = self.last_start_pose.pose.position
            return self._transform_xyz(
                (pose.x, pose.y, pose.z), self.last_start_pose.header.frame_id
            )
        return None

    def _transform_xyz(self, xyz: Point3, frame_id: str) -> Optional[Point3]:
        source = str(frame_id or self.map_frame).strip()
        if source == self.map_frame:
            return (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Cannot transform point from '%s' to '%s': %s" % (source, self.map_frame, exc)
            )
            return None
        q = transform.transform.rotation
        t = transform.transform.translation
        rotated = self._rotate((q.x, q.y, q.z, q.w), xyz)
        return (rotated[0] + t.x, rotated[1] + t.y, rotated[2] + t.z)

    @staticmethod
    def _rotate(q, value: Point3) -> Point3:
        qx, qy, qz, qw = q
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1.0e-12:
            return value
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        uv = (qy * value[2] - qz * value[1], qz * value[0] - qx * value[2], qx * value[1] - qy * value[0])
        uuv = (qy * uv[2] - qz * uv[1], qz * uv[0] - qx * uv[2], qx * uv[1] - qy * uv[0])
        return (
            value[0] + 2.0 * (qw * uv[0] + uuv[0]),
            value[1] + 2.0 * (qw * uv[1] + uuv[1]),
            value[2] + 2.0 * (qw * uv[2] + uuv[2]),
        )

    @staticmethod
    def _distance(left: Point3, right: Point3) -> float:
        return math.sqrt(sum((left[i] - right[i]) ** 2 for i in range(3)))

    @staticmethod
    def _color(red, green, blue, alpha) -> ColorRGBA:
        color = ColorRGBA()
        color.r, color.g, color.b, color.a = red, green, blue, alpha
        return color

    @staticmethod
    def _status_text(prefix: str, report: FeasibilityReport) -> str:
        reason = '|'.join(report.reasons) if report.reasons else 'none'
        clearance = (
            'inf' if not math.isfinite(report.minimum_clearance)
            else '%.3f' % report.minimum_clearance
        )
        return (
            '%s feasible=%s min_clearance=%s collision_points=%d '
            'infeasible_points=%d discontinuities=%d reason=%s'
            % (
                prefix,
                str(report.feasible).lower(),
                clearance,
                report.collision_point_count,
                report.infeasible_point_count,
                report.discontinuity_count,
                reason,
            )
        )

    def _publish_status(self, text: str):
        message = String()
        message.data = text
        self.status_pub.publish(message)
        if 'ERROR' in text or 'INFEASIBLE' in text or 'FAILED' in text:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FastGlobalPlannerNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

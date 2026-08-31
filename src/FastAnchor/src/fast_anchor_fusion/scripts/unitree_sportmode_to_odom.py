#!/usr/bin/env python3

import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

try:
    from unitree_go.msg import SportModeState
except ImportError as error:
    print(
        "unitree_go.msg is unavailable. Source the Unitree ROS 2 environment "
        "before enabling leg odometry fusion.",
        file=sys.stderr,
    )
    raise SystemExit(2) from error


def normalize_quaternion(w, x, y, z):
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def rotate_world_vector_to_body(quaternion, vector):
    w, x, y, z = quaternion
    vx, vy, vz = vector

    # 展开 R(world_from_body)^T * v_world，避免引入额外数学依赖。
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y + w * z) * vy
        + 2.0 * (x * z - w * y) * vz,
        2.0 * (x * y - w * z) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z + w * x) * vz,
        2.0 * (x * z + w * y) * vx
        + 2.0 * (y * z - w * x) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


def set_covariance_diagonal(covariance, diagonal):
    for index, value in enumerate(diagonal):
        covariance[index * 6 + index] = value


class UnitreeSportModeToOdom(Node):
    def __init__(self):
        super().__init__("unitree_sportmode_to_odom")
        self.input_topic = self.declare_parameter(
            "input_topic", "/lf/sportmodestate"
        ).value
        self.output_topic = self.declare_parameter("output_topic", "/leg_odom").value
        self.odom_frame = self.declare_parameter(
            "odom_frame", "unitree_odom"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.velocity_in_world_frame = self.declare_parameter(
            "velocity_in_world_frame", True
        ).value
        self.use_message_stamp = self.declare_parameter(
            "use_message_stamp", False
        ).value
        self.drop_on_error = self.declare_parameter("drop_on_error", False).value
        self.pose_covariance_diagonal = self.declare_parameter(
            "pose_covariance_diagonal",
            [0.04, 0.04, 0.09, 0.03, 0.03, 0.05],
        ).value
        self.twist_covariance_diagonal = self.declare_parameter(
            "twist_covariance_diagonal",
            [0.04, 0.04, 0.09, 0.04, 0.04, 0.05],
        ).value

        if len(self.pose_covariance_diagonal) != 6:
            raise ValueError("pose_covariance_diagonal must contain 6 values")
        if len(self.twist_covariance_diagonal) != 6:
            raise ValueError("twist_covariance_diagonal must contain 6 values")

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.subscription = self.create_subscription(
            SportModeState, self.input_topic, self.sportmode_callback, 20
        )
        self.get_logger().info(
            f"Converting Unitree {self.input_topic} to {self.output_topic} "
            f"({self.odom_frame} -> {self.base_frame})"
        )

    def sportmode_callback(self, message):
        if message.error_code != 0:
            if self.drop_on_error:
                self.get_logger().warning(
                    f"Dropping SportModeState with error_code={message.error_code}",
                    throttle_duration_sec=2.0,
                )
                return
            self.get_logger().warning(
                f"SportModeState error_code={message.error_code}; publishing because "
                "drop_on_error is false",
                throttle_duration_sec=2.0,
            )

        output = Odometry()
        if (
            self.use_message_stamp
            and message.stamp.sec >= 0
            and 0 <= message.stamp.nanosec < 1_000_000_000
        ):
            output.header.stamp.sec = message.stamp.sec
            output.header.stamp.nanosec = message.stamp.nanosec
        else:
            # 默认使用 ROS 接收时刻，避免 Unitree 设备时钟和 LiDAR/ROS 时钟不一致。
            output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame

        output.pose.pose.position.x = float(message.position[0])
        output.pose.pose.position.y = float(message.position[1])
        output.pose.pose.position.z = float(message.position[2])

        quaternion = normalize_quaternion(
            float(message.imu_state.quaternion[0]),
            float(message.imu_state.quaternion[1]),
            float(message.imu_state.quaternion[2]),
            float(message.imu_state.quaternion[3]),
        )
        output.pose.pose.orientation.w = quaternion[0]
        output.pose.pose.orientation.x = quaternion[1]
        output.pose.pose.orientation.y = quaternion[2]
        output.pose.pose.orientation.z = quaternion[3]

        velocity = tuple(float(component) for component in message.velocity)
        if self.velocity_in_world_frame:
            velocity = rotate_world_vector_to_body(quaternion, velocity)
        output.twist.twist.linear.x = velocity[0]
        output.twist.twist.linear.y = velocity[1]
        output.twist.twist.linear.z = velocity[2]
        output.twist.twist.angular.x = float(message.imu_state.gyroscope[0])
        output.twist.twist.angular.y = float(message.imu_state.gyroscope[1])
        output.twist.twist.angular.z = float(message.yaw_speed)

        set_covariance_diagonal(output.pose.covariance, self.pose_covariance_diagonal)
        set_covariance_diagonal(output.twist.covariance, self.twist_covariance_diagonal)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeSportModeToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

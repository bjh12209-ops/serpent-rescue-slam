#!/usr/bin/env python3
"""Publish app-friendly pose, path, and distance telemetry for online SLAM."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_yaw(x, y, z, w):
    """Return yaw in radians for a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_difference(first, second):
    """Return the smallest absolute difference between two angles."""
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def path_length(poses, dimensions=2):
    """Return the polyline length of PoseStamped-like messages."""
    total = 0.0
    previous = None
    for stamped_pose in poses:
        position = stamped_pose.pose.position
        current = (position.x, position.y, position.z)
        if previous is not None:
            if dimensions == 2:
                total += math.hypot(
                    current[0] - previous[0], current[1] - previous[1]
                )
            else:
                total += math.dist(current, previous)
        previous = current
    return total


class DistanceAccumulator:
    """Accumulate motion while rejecting pose jitter and impossible jumps."""

    def __init__(
        self,
        min_step=0.025,
        max_speed=1.0,
        max_jump=0.5,
        smoothing_alpha=0.35,
    ):
        self.min_step = max(0.0, float(min_step))
        self.max_speed = max(0.0, float(max_speed))
        self.max_jump = max(0.0, float(max_jump))
        self.alpha = min(max(float(smoothing_alpha), 0.0), 1.0)
        self.last_raw = None
        self.last_stamp = None
        self.filtered = None
        self.accepted = None
        self.distance_2d = 0.0
        self.distance_3d = 0.0

    def update(self, position, stamp):
        """Consume one XYZ sample and return whether motion was accepted."""
        current = tuple(float(value) for value in position)
        sample_stamp = float(stamp)
        if self.last_raw is None:
            self.last_raw = current
            self.last_stamp = sample_stamp
            self.filtered = current
            self.accepted = current
            return False

        raw_step = math.dist(current, self.last_raw)
        dt = sample_stamp - self.last_stamp
        impossible_speed = (
            self.max_speed > 0.0
            and dt > 0.0
            and raw_step >= self.min_step
            and raw_step / dt > self.max_speed
        )
        impossible_jump = self.max_jump > 0.0 and raw_step > self.max_jump
        self.last_stamp = sample_stamp
        if impossible_speed or impossible_jump:
            # Re-anchor without counting the rejected displacement. Keeping
            # last_raw at the old pose makes every later sample look like the
            # same jump and can permanently freeze distance at 0 m.
            self.last_raw = current
            self.filtered = current
            self.accepted = current
            return False

        self.last_raw = current
        self.filtered = tuple(
            self.alpha * value + (1.0 - self.alpha) * filtered
            for value, filtered in zip(current, self.filtered)
        )
        step_2d = math.hypot(
            self.filtered[0] - self.accepted[0],
            self.filtered[1] - self.accepted[1],
        )
        step_3d = math.dist(self.filtered, self.accepted)
        if max(step_2d, step_3d) < self.min_step:
            return False

        self.distance_2d += step_2d
        self.distance_3d += step_3d
        self.accepted = self.filtered
        return True


class SlamTelemetry(Node):
    """Convert SLAM TF and odometry into stable display telemetry topics."""

    def __init__(self):
        super().__init__("slam_telemetry")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("map_path_topic", "/mapPath")
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("path_min_distance", 0.02)
        self.declare_parameter("path_min_angle", 0.035)
        self.declare_parameter("max_path_points", 20000)
        self.declare_parameter("max_odom_step", 0.5)
        self.declare_parameter("distance_min_step", 0.025)
        self.declare_parameter("distance_max_speed", 2.5)
        self.declare_parameter("distance_smoothing_alpha", 0.35)

        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        odom_topic = self.get_parameter("odom_topic").value
        map_path_topic = self.get_parameter("map_path_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.path_min_distance = float(
            self.get_parameter("path_min_distance").value
        )
        self.path_min_angle = float(self.get_parameter("path_min_angle").value)
        self.max_path_points = int(self.get_parameter("max_path_points").value)
        self.max_odom_step = float(self.get_parameter("max_odom_step").value)
        self.distance_accumulator = DistanceAccumulator(
            min_step=self.get_parameter("distance_min_step").value,
            max_speed=self.get_parameter("distance_max_speed").value,
            max_jump=self.max_odom_step,
            smoothing_alpha=self.get_parameter(
                "distance_smoothing_alpha"
            ).value,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pose_publisher = self.create_publisher(
            PoseStamped, "/slam/current_pose", 10
        )
        self.start_pose_publisher = self.create_publisher(
            PoseStamped, "/slam/start_pose", latched_qos
        )
        self.path_publisher = self.create_publisher(
            Path, "/slam/path", latched_qos
        )
        self.distance_traveled_publisher = self.create_publisher(
            Float64, "/slam/distance_traveled", 10
        )
        self.distance_traveled_3d_publisher = self.create_publisher(
            Float64, "/slam/distance_traveled_3d", 10
        )
        self.distance_slam_corrected_publisher = self.create_publisher(
            Float64, "/slam/distance_slam_corrected", 10
        )
        self.distance_from_start_publisher = self.create_publisher(
            Float64, "/slam/distance_from_start", 10
        )
        self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(Path, map_path_topic, self.map_path_callback, 10)

        self.path = Path()
        self.path.header.frame_id = self.global_frame
        self.optimized_path = None
        self.start_position = None
        self.start_pose = None
        self.last_path_position = None
        self.last_path_yaw = None
        self.distance_slam_corrected = 0.0
        self.warned_tf = False

        timer_period = 1.0 / max(publish_rate, 0.1)
        self.create_timer(timer_period, self.publish_telemetry)
        self.get_logger().info(
            f"Publishing online SLAM telemetry from {self.global_frame} -> "
            f"{self.robot_frame}; distance source is {odom_topic}"
        )

    def map_path_callback(self, message):
        """Keep RTAB-Map's loop-closure-corrected trajectory when available."""
        if message.poses:
            self.optimized_path = message
            self.distance_slam_corrected = path_length(message.poses, dimensions=2)

    def odom_callback(self, message):
        """Accumulate traveled distance in the continuous odometry frame."""
        position = message.pose.pose.position
        current = (position.x, position.y, position.z)
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        previous_raw = self.distance_accumulator.last_raw
        accepted = self.distance_accumulator.update(current, stamp)
        if not accepted and previous_raw is not None:
            raw_step = math.dist(current, previous_raw)
            if raw_step > self.max_odom_step:
                self.get_logger().warning(
                    f"Ignoring odometry jump of {raw_step:.3f} m",
                    throttle_duration_sec=5.0,
                )

    def publish_telemetry(self):
        """Publish the latest map pose and append it to the displayed path."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException as error:
            if not self.warned_tf:
                self.get_logger().warning(
                    f"Waiting for {self.global_frame} -> {self.robot_frame}: {error}"
                )
                self.warned_tf = True
            return

        self.warned_tf = False
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position = (translation.x, translation.y, translation.z)

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation = rotation
        self.pose_publisher.publish(pose)

        if self.start_position is None:
            self.start_position = position
            self.start_pose = pose
        self.start_pose_publisher.publish(self.start_pose)

        yaw = quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        should_append = self.last_path_position is None
        if self.last_path_position is not None:
            should_append = (
                math.dist(position, self.last_path_position)
                >= self.path_min_distance
                or angle_difference(yaw, self.last_path_yaw) >= self.path_min_angle
            )

        if should_append:
            self.path.poses.append(pose)
            if len(self.path.poses) > self.max_path_points:
                self.path.poses = self.path.poses[-self.max_path_points:]
            self.last_path_position = position
            self.last_path_yaw = yaw

        self.path.header.stamp = self.get_clock().now().to_msg()
        if self.optimized_path is not None:
            self.path_publisher.publish(self.optimized_path)
        else:
            self.path_publisher.publish(self.path)
        self.distance_traveled_publisher.publish(
            Float64(data=self.distance_accumulator.distance_2d)
        )
        self.distance_traveled_3d_publisher.publish(
            Float64(data=self.distance_accumulator.distance_3d)
        )
        self.distance_slam_corrected_publisher.publish(
            Float64(data=self.distance_slam_corrected)
        )
        self.distance_from_start_publisher.publish(
            Float64(data=math.dist(position, self.start_position))
        )


def main(args=None):
    """Run the SLAM telemetry node."""
    rclpy.init(args=args)
    node = SlamTelemetry()
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

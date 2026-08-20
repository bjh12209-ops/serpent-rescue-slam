#!/usr/bin/env python3
"""Publish compact topic-rate diagnostics for SLAM development and apps."""

from collections import deque
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class RateTracker:
    """Track arrival rate and age without retaining message payloads."""

    def __init__(self, label, required, stale_after, window_size=120):
        self.label = label
        self.required = required
        self.stale_after = stale_after
        self.arrivals = deque(maxlen=window_size)

    def tick(self):
        """Record a message arrival using monotonic time."""
        self.arrivals.append(time.monotonic())

    def rate(self):
        """Return the measured arrival rate in hertz."""
        if len(self.arrivals) < 2:
            return 0.0
        duration = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / duration if duration > 0.0 else 0.0

    def age(self):
        """Return seconds since the last message, or infinity before startup."""
        if not self.arrivals:
            return float("inf")
        return time.monotonic() - self.arrivals[-1]

    def healthy(self):
        """Return whether a required stream is currently alive."""
        return not self.required or self.age() <= self.stale_after


class SlamHealthMonitor(Node):
    """Monitor small companion topics instead of copying full RGB-D images."""

    def __init__(self):
        super().__init__("slam_health_monitor")

        self.declare_parameter("expect_imu", True)
        self.declare_parameter("expect_ekf", True)
        self.declare_parameter("robot_frame", "base_link")
        expect_imu = bool(self.get_parameter("expect_imu").value)
        expect_ekf = bool(self.get_parameter("expect_ekf").value)
        self.robot_frame = self.get_parameter("robot_frame").value

        self.trackers = {
            "RGB": RateTracker("RGB", True, 0.5),
            "DEPTH": RateTracker("DEPTH", True, 0.5),
            "VO": RateTracker("VO", True, 0.5),
            "IMU": RateTracker("IMU", expect_imu, 0.5),
            "EKF": RateTracker("EKF", expect_ekf, 0.5),
            "SLAM": RateTracker("SLAM", True, 2.0),
        }

        qos = qos_profile_sensor_data
        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            lambda _: self.trackers["RGB"].tick(),
            qos,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/aligned_depth_to_color/camera_info",
            lambda _: self.trackers["DEPTH"].tick(),
            qos,
        )
        self.create_subscription(
            Odometry,
            "/visual_odom",
            lambda _: self.trackers["VO"].tick(),
            qos,
        )
        self.create_subscription(
            Imu,
            "/imu_52/data",
            lambda _: self.trackers["IMU"].tick(),
            qos,
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            lambda _: self.trackers["EKF"].tick(),
            qos,
        )
        self.create_subscription(
            Info,
            "/info",
            lambda _: self.trackers["SLAM"].tick(),
            qos,
        )

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/slam/diagnostics", 10
        )
        self.summary_publisher = self.create_publisher(
            String, "/slam/rate_summary", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/slam/status_marker", 10
        )
        self.create_timer(1.0, self.publish_status)

    def display_rate(self, name):
        """Format an optional or required stream for the operator display."""
        tracker = self.trackers[name]
        if not tracker.required and not tracker.arrivals:
            return "--"
        if tracker.age() > tracker.stale_after:
            return "LOST"
        return f"{tracker.rate():.1f}"

    def publish_status(self):
        """Publish diagnostics, an app string, and an RViz text marker."""
        now = self.get_clock().now().to_msg()
        unhealthy = [
            tracker.label
            for tracker in self.trackers.values()
            if not tracker.healthy()
        ]

        diagnostic = DiagnosticStatus()
        diagnostic.name = "online_slam/topic_rates"
        diagnostic.hardware_id = "snake_robot"
        diagnostic.level = (
            DiagnosticStatus.WARN if unhealthy else DiagnosticStatus.OK
        )
        diagnostic.message = (
            "Missing or stale: " + ", ".join(unhealthy)
            if unhealthy
            else "All required SLAM streams are alive"
        )
        for tracker in self.trackers.values():
            diagnostic.values.append(
                KeyValue(
                    key=f"{tracker.label}_hz",
                    value=f"{tracker.rate():.2f}",
                )
            )
            diagnostic.values.append(
                KeyValue(
                    key=f"{tracker.label}_age_sec",
                    value=(
                        f"{tracker.age():.2f}"
                        if tracker.age() != float("inf")
                        else "inf"
                    ),
                )
            )

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = now
        diagnostics.status.append(diagnostic)
        self.diagnostics_publisher.publish(diagnostics)

        lines = [
            f"RGB {self.display_rate('RGB')} | DEPTH {self.display_rate('DEPTH')}",
            f"VO {self.display_rate('VO')} | SLAM {self.display_rate('SLAM')}",
            f"IMU {self.display_rate('IMU')} | EKF {self.display_rate('EKF')} Hz",
        ]
        summary = "\n".join(lines)
        self.summary_publisher.publish(String(data=summary))

        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = now
        marker.ns = "slam_health"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.08
        marker.color.r = 1.0 if unhealthy else 0.2
        marker.color.g = 0.55 if unhealthy else 1.0
        marker.color.b = 0.1 if unhealthy else 0.3
        marker.color.a = 0.95
        marker.text = summary
        self.marker_publisher.publish(marker)


def main(args=None):
    """Run the SLAM health monitor."""
    rclpy.init(args=args)
    node = SlamHealthMonitor()
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

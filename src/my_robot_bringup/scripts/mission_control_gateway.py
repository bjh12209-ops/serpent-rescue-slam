#!/usr/bin/env python3
"""Read-only ROS 2 to WebSocket gateway for the mission-control browser UI."""

import asyncio
import base64
from contextlib import suppress
from copy import deepcopy
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import numpy as np
from pathlib import Path
import struct
import sys
import threading
import time
from urllib.parse import urlsplit

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rtabmap_msgs.msg import Info, MapGraph
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, String

try:
    import websockets
except ImportError as error:
    raise SystemExit(
        "python3-websockets is required. Install it with: "
        "sudo apt install python3-websockets"
    ) from error


def apply_websockets_9_asyncio_compatibility():
    """Ignore obsolete loop keywords used by Ubuntu Jammy websockets 9.1."""
    try:
        major_version = int(websockets.__version__.split(".", maxsplit=1)[0])
    except (AttributeError, ValueError):
        return
    if major_version >= 10 or sys.version_info < (3, 10):
        return

    original_lock = asyncio.Lock

    class CompatibleLock(original_lock):
        """asyncio.Lock accepting the removed Python 3.10 loop keyword."""

        def __init__(self, *args, **kwargs):
            kwargs.pop("loop", None)
            super().__init__(*args, **kwargs)

    asyncio.Lock = CompatibleLock

    for function_name in ("sleep", "wait", "wait_for"):
        original_function = getattr(asyncio, function_name)

        def without_loop(*args, _function=original_function, **kwargs):
            kwargs.pop("loop", None)
            return _function(*args, **kwargs)

        setattr(asyncio, function_name, without_loop)


apply_websockets_9_asyncio_compatibility()


EXPECTED_RATES = {
    "RGB": 30.0,
    "DEPTH": 30.0,
    "VO": 30.0,
    "SLAM": 2.0,
    "IMU50": 45.0,
    "IMU51": 45.0,
    "IMU52": 45.0,
    "EKF": 45.0,
}

STALE_LIMITS = {
    "RGB": 0.5,
    "DEPTH": 0.5,
    "VO": 0.5,
    "SLAM": 2.0,
    "IMU50": 0.5,
    "IMU51": 0.5,
    "IMU52": 0.5,
    "EKF": 0.5,
}

CLOUD_MAGIC = b"SPC1"
CLOUD_RECORD_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("red", "u1"), ("green", "u1"), ("blue", "u1"), ("padding", "u1"),
])


def cloud_replacement_decision(
    previous_count,
    candidate_count,
    sparse_streak,
    minimum_baseline=500,
    minimum_ratio=0.12,
):
    """
    Reject isolated, implausibly sparse replacements of a useful map.

    RTAB-Map may briefly publish a tiny cloud while its map products are being
    rebuilt. A verified /mapGraph reset clears the retained cloud separately,
    so a tiny product must never overwrite an accumulated map by itself.
    """
    previous = max(0, int(previous_count))
    candidate = max(0, int(candidate_count))
    if candidate == 0:
        return False, sparse_streak
    suspicious = (
        previous >= int(minimum_baseline)
        and candidate < previous * float(minimum_ratio)
    )
    if not suspicious:
        return True, 0
    return False, int(sparse_streak) + 1


def map_graph_restarted(previous_max_id, current_max_id):
    """Return true only when the complete RTAB graph restarts at lower IDs."""
    previous = max(0, int(previous_max_id))
    current = max(0, int(current_max_id))
    return (
        previous >= 2
        and current > 0
        and (current == 1 or (previous >= 8 and current <= previous // 4))
    )


def quaternion_yaw(quaternion):
    """Return REP-103 yaw in radians from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def normalized_quaternion(quaternion):
    """Return a finite, unit (x, y, z, w) quaternion."""
    values = tuple(
        float(getattr(quaternion, component))
        for component in ("x", "y", "z", "w")
    )
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in values)


def quaternion_to_euler_degrees(quaternion):
    """Convert a quaternion to display-only REP-103 roll/pitch/yaw degrees."""
    x, y, z, w = normalized_quaternion(quaternion)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def pose_to_dict(pose):
    """Convert a geometry_msgs pose to the compact browser contract."""
    return {
        "x": round(float(pose.position.x), 4),
        "y": round(float(pose.position.y), 4),
        "z": round(float(pose.position.z), 4),
        "yaw": round(quaternion_yaw(pose.orientation), 5),
    }


def point_cloud_binary_packet(message, maximum, min_z, max_z):
    """Vectorize and pack a bounded PointCloud2 as SPC1 binary records."""
    fields = {field.name: field for field in message.fields}
    color_name = "rgb" if "rgb" in fields else (
        "rgba" if "rgba" in fields else None
    )
    field_names = ["x", "y", "z"]
    if color_name:
        field_names.append(color_name)
    points = point_cloud2.read_points(
        message, field_names=field_names, skip_nans=False
    )
    if len(points) == 0:
        return None, 0

    x_values = np.asarray(points["x"])
    y_values = np.asarray(points["y"])
    z_values = np.asarray(points["z"])
    valid = (
        np.isfinite(x_values)
        & np.isfinite(y_values)
        & np.isfinite(z_values)
        & (z_values >= min_z)
        & (z_values <= max_z)
    )
    indices = np.flatnonzero(valid)
    if len(indices) == 0:
        return None, 0
    if len(indices) > maximum:
        sample_offsets = np.linspace(
            0, len(indices) - 1, num=maximum, dtype=np.int64
        )
        indices = indices[sample_offsets]

    records = np.zeros(len(indices), dtype=CLOUD_RECORD_DTYPE)
    records["x"] = x_values[indices]
    records["y"] = y_values[indices]
    records["z"] = z_values[indices]
    if color_name:
        packed_values = np.asarray(points[color_name])[indices]
        if fields[color_name].datatype == PointField.FLOAT32:
            packed = np.ascontiguousarray(
                packed_values, dtype=np.float32
            ).view(np.uint32)
        else:
            packed = packed_values.astype(np.uint32, copy=False)
        records["red"] = (packed >> 16) & 0xFF
        records["green"] = (packed >> 8) & 0xFF
        records["blue"] = packed & 0xFF
    else:
        tone = np.clip((records["z"] + 1.5) / 4.5, 0.0, 1.0)
        records["red"] = (61 + tone * 89).astype(np.uint8)
        records["green"] = (140 + tone * 71).astype(np.uint8)
        records["blue"] = 173

    packet = CLOUD_MAGIC + struct.pack("<I", len(records)) + records.tobytes()
    return packet, len(records)


class MissionHttpHandler(SimpleHTTPRequestHandler):
    """Serve static UI assets and connect the root page to this gateway."""

    protocol_version = "HTTP/1.1"

    def __init__(
        self,
        *args,
        websocket_port,
        camera_frame_callback,
        camera_stream_rate,
        **kwargs,
    ):
        self.websocket_port = websocket_port
        self.camera_frame_callback = camera_frame_callback
        self.camera_stream_rate = camera_stream_rate
        super().__init__(*args, **kwargs)

    def end_headers(self):
        """Prevent browsers from keeping an obsolete dashboard bundle."""
        if urlsplit(self.path).path != "/camera.mjpg":
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
        super().end_headers()

    def do_GET(self):
        parsed = urlsplit(self.path)
        if parsed.path == "/camera.mjpg":
            self._serve_camera_stream()
            return
        if parsed.path == "/" and not parsed.query:
            host_header = self.headers.get("Host", "localhost")
            host = host_header.rsplit(":", 1)[0]
            if ":" in host and not host.startswith("["):
                host = f"[{host}]"
            target = f"/?ws=ws://{host}:{self.websocket_port}/telemetry"
            self.send_response(302)
            self.send_header("Location", target)
            self.end_headers()
            return
        super().do_GET()

    def _serve_camera_stream(self):
        """Stream only the newest JPEG frame on an independent HTTP path."""
        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        last_sequence = -1
        period = 1.0 / max(float(self.camera_stream_rate), 1.0)
        try:
            while True:
                frame = self.camera_frame_callback()
                if frame is None or frame[0] == last_sequence:
                    time.sleep(period)
                    continue
                sequence, stamp, jpeg = frame
                last_sequence = sequence
                headers = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n".encode("ascii")
                    + f"X-Timestamp: {stamp:.6f}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(headers)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return

    def log_message(self, message_format, *args):
        """Keep routine browser requests out of ROS logs."""
        del message_format, args


class WebSocketHub:
    """Run a websockets server in its own asyncio thread."""

    def __init__(
        self, host, port, snapshot_callback, binary_snapshot_callback, logger
    ):
        self.host = host
        self.port = port
        self.snapshot_callback = snapshot_callback
        self.binary_snapshot_callback = binary_snapshot_callback
        self.logger = logger
        self.loop = None
        self.queue = None
        self.clients = set()
        self.thread = None
        self.ready = threading.Event()
        self.start_error = None
        self.stop_future = None

    def start(self):
        self.thread = threading.Thread(
            target=self._thread_main,
            name="mission-control-websocket",
            daemon=True,
        )
        self.thread.start()
        if not self.ready.wait(timeout=5.0):
            raise RuntimeError("WebSocket server did not start within 5 seconds")
        if self.start_error is not None:
            raise RuntimeError(str(self.start_error)) from self.start_error

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.queue = asyncio.Queue(maxsize=16)
        try:
            self.loop.run_until_complete(self._serve())
        except Exception as error:  # pragma: no cover - startup environment
            self.start_error = error
            self.ready.set()
        finally:
            self.loop.close()

    async def _serve(self):
        broadcaster = asyncio.create_task(self._broadcast_loop())
        self.stop_future = self.loop.create_future()
        async with websockets.serve(
            self._client_handler,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20,
            max_size=2**22,
            compression=None,
        ):
            self.ready.set()
            await self.stop_future
        broadcaster.cancel()
        with suppress(asyncio.CancelledError):
            await broadcaster

    async def _client_handler(self, websocket, path):
        if path != "/telemetry":
            await websocket.close(code=1008, reason="Use /telemetry")
            return
        self.clients.add(websocket)
        try:
            await websocket.send(self.snapshot_callback())
            binary_snapshot = self.binary_snapshot_callback()
            if binary_snapshot:
                await websocket.send(binary_snapshot)
            async for raw_message in websocket:
                try:
                    message = json.loads(raw_message)
                except (json.JSONDecodeError, TypeError):
                    continue
                if message.get("type") == "ping":
                    await websocket.send(json.dumps({
                        "type": "pong",
                        "sent": message.get("sent"),
                    }))
        finally:
            self.clients.discard(websocket)

    async def _broadcast_loop(self):
        while True:
            message = await self.queue.get()
            if not self.clients:
                continue
            targets = tuple(self.clients)
            results = await asyncio.gather(
                *(client.send(message) for client in targets),
                return_exceptions=True,
            )
            for client, result in zip(targets, results):
                if isinstance(result, Exception):
                    self.clients.discard(client)

    def publish(self, payload):
        if self.loop is None or self.queue is None:
            return
        encoded = payload if isinstance(payload, bytes) else json.dumps(
            payload, separators=(",", ":"), allow_nan=False
        )

        def enqueue():
            if self.queue.full():
                try:
                    self.queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            self.queue.put_nowait(encoded)

        self.loop.call_soon_threadsafe(enqueue)

    def stop(self):
        if self.loop is None or self.stop_future is None:
            return

        def request_stop():
            if not self.stop_future.done():
                self.stop_future.set_result(None)

        self.loop.call_soon_threadsafe(request_stop)
        if self.thread is not None:
            self.thread.join(timeout=3.0)


class MissionControlGateway(Node):
    """Translate selected ROS telemetry into bounded, read-only UI messages."""

    def __init__(self):
        super().__init__("mission_control_gateway")
        self._declare_parameters()

        self.state_lock = threading.Lock()
        self.started_at = time.monotonic()
        self.last_path_publish = 0.0
        self.last_map_publish = 0.0
        self.last_cloud_publish = 0.0
        self.last_camera_publish = 0.0
        self.last_yolo_input_publish = 0.0
        self.last_camera_frame = None
        self.camera_sequence = 0
        self.camera_lock = threading.Lock()
        self.camera_input_lock = threading.Lock()
        self.camera_input_event = threading.Event()
        self.latest_camera_message = None
        self.cloud_input_lock = threading.Lock()
        self.cloud_input_event = threading.Event()
        self.latest_cloud_message = None
        self.cloud_packet = None
        self.sparse_cloud_streak = 0
        self.last_graph_node_id = 0
        self.running = True
        self.cv_bridge = CvBridge()
        self.yolo_input_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("yolo_input_topic").value),
            qos_profile_sensor_data,
        )
        self.imu_layout = self._build_imu_layout()
        self.snake_model = self._build_snake_model()
        self.imu_last_arrival = {
            item["sensorId"]: None for item in self.imu_layout
        }
        self.imu_last_web_publish = {
            item["sensorId"]: 0.0 for item in self.imu_layout
        }
        initial_imus = {
            item["sensorId"]: self._empty_imu_state(item)
            for item in self.imu_layout
        }

        self.state = {
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "start": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "path": [],
            "distanceTraveled": 0.0,
            "distanceTraveled3d": 0.0,
            "distanceSlamCorrected": 0.0,
            "distanceFromStart": 0.0,
            "mapNodes": 0,
            "mapRate": 0.0,
            "cloudPointCount": 0,
            "battery": None,
            "latency": None,
            "rates": {
                name: {"value": 0.0, "expected": expected, "ok": False}
                for name, expected in EXPECTED_RATES.items()
            },
            "segmentPoses": [],
            "imus": initial_imus,
            "snakeModel": deepcopy(self.snake_model),
            "target": None,
            "personDetection": {
                "detected": False,
                "count": 0,
                "candidates": [],
                "modelReady": False,
            },
        }
        self.map_state = {
            "cellSize": float(self.get_parameter("map_cell_size").value),
            "known": [],
            "occupied": [],
        }
        websocket_host = str(self.get_parameter("websocket_host").value)
        websocket_port = int(self.get_parameter("websocket_port").value)
        self.websocket_hub = WebSocketHub(
            websocket_host,
            websocket_port,
            self.snapshot_json,
            self.cloud_binary_snapshot,
            self.get_logger(),
        )
        self.websocket_hub.start()

        self.http_server = None
        self.http_thread = None
        if bool(self.get_parameter("serve_ui").value):
            self._start_http_server(websocket_port)

        self._create_subscriptions()
        self.camera_worker = threading.Thread(
            target=self._camera_worker_main,
            name="mission-camera-encoder",
            daemon=True,
        )
        self.cloud_worker = threading.Thread(
            target=self._cloud_worker_main,
            name="mission-cloud-packer",
            daemon=True,
        )
        self.camera_worker.start()
        self.cloud_worker.start()
        self.create_timer(1.0, self.publish_heartbeat)
        self.get_logger().info(
            f"Read-only WebSocket telemetry: ws://{websocket_host}:"
            f"{websocket_port}/telemetry"
        )

    def _declare_parameters(self):
        self.declare_parameter("websocket_host", "127.0.0.1")
        self.declare_parameter("websocket_port", 8765)
        self.declare_parameter("serve_ui", True)
        self.declare_parameter("ui_host", "127.0.0.1")
        self.declare_parameter("ui_port", 8080)
        self.declare_parameter("ui_directory", "")
        self.declare_parameter("pose_topic", "/slam/current_pose")
        self.declare_parameter("start_pose_topic", "/slam/start_pose")
        self.declare_parameter("path_topic", "/slam/path")
        self.declare_parameter(
            "distance_traveled_topic", "/slam/distance_traveled"
        )
        self.declare_parameter(
            "distance_traveled_3d_topic", "/slam/distance_traveled_3d"
        )
        self.declare_parameter(
            "distance_slam_corrected_topic",
            "/slam/distance_slam_corrected",
        )
        self.declare_parameter(
            "distance_from_start_topic", "/slam/distance_from_start"
        )
        self.declare_parameter("diagnostics_topic", "/slam/diagnostics")
        # rtabmap publishes its 2D occupancy grid on /map by default.
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cloud_topic", "/cloud_map")
        self.declare_parameter("info_topic", "/info")
        self.declare_parameter("map_graph_topic", "/mapGraph")
        self.declare_parameter(
            "camera_topic", "/camera/camera/color/image_raw"
        )
        self.declare_parameter(
            "person_detection_topic", "/perception/person_detection"
        )
        self.declare_parameter("segment_poses_topic", "/snake/segment_poses")
        self.declare_parameter("imu_ids", ["50", "51", "52"])
        self.declare_parameter(
            "imu_topics", ["/imu_50/data", "/imu_51/data", "/imu_52/data"]
        )
        self.declare_parameter("imu_roles", ["HEAD", "MIDDLE", "TAIL"])
        self.declare_parameter("imu_module_indices", [1, 2, 3])
        self.declare_parameter("imu_translation_known", [True, False, False])
        self.declare_parameter("imu_translation_x", [-0.05, 0.0, 0.0])
        self.declare_parameter("imu_translation_y", [0.0, 0.0, 0.0])
        self.declare_parameter("imu_translation_z", [0.02, 0.0, 0.0])
        self.declare_parameter("imu_visual_lengths", [0.28, 0.28, 0.28])
        self.declare_parameter("robot_module_count", 7)
        self.declare_parameter("imu_stale_timeout_sec", 0.5)
        self.declare_parameter("imu_web_publish_rate", 15.0)
        self.declare_parameter("map_cell_size", 0.05)
        self.declare_parameter("map_publish_rate", 1.0)
        self.declare_parameter("cloud_publish_rate", 1.0)
        self.declare_parameter("camera_publish_rate", 20.0)
        self.declare_parameter("camera_stream_rate", 20.0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("camera_websocket_enabled", False)
        self.declare_parameter("yolo_input_topic", "/mission_control/yolo_input")
        self.declare_parameter("yolo_input_rate", 2.0)
        self.declare_parameter("yolo_input_width", 320)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("max_path_points", 4000)
        self.declare_parameter("max_cloud_points", 50000)
        self.declare_parameter("cloud_min_z", -1.5)
        self.declare_parameter("cloud_max_z", 3.0)

    def _build_imu_layout(self):
        """Build a validated, extensible active-module description."""
        fields = {
            "ids": list(self.get_parameter("imu_ids").value),
            "topics": list(self.get_parameter("imu_topics").value),
            "roles": list(self.get_parameter("imu_roles").value),
            "indices": list(self.get_parameter("imu_module_indices").value),
            "known": list(self.get_parameter("imu_translation_known").value),
            "x": list(self.get_parameter("imu_translation_x").value),
            "y": list(self.get_parameter("imu_translation_y").value),
            "z": list(self.get_parameter("imu_translation_z").value),
            "lengths": list(self.get_parameter("imu_visual_lengths").value),
        }
        count = len(fields["ids"])
        mismatched = [name for name, values in fields.items() if len(values) != count]
        if not count or mismatched:
            raise ValueError(
                "IMU layout arrays must be non-empty and equal length; bad: "
                + ", ".join(mismatched)
            )
        layout = []
        for index in range(count):
            sensor_id = str(fields["ids"][index]).lower().removeprefix("0x")
            known = bool(fields["known"][index])
            translation = None
            if known:
                translation = {
                    "x": float(fields["x"][index]),
                    "y": float(fields["y"][index]),
                    "z": float(fields["z"][index]),
                }
            layout.append({
                "sensorId": sensor_id,
                "displayId": f"0x{sensor_id.upper()}",
                "topic": str(fields["topics"][index]),
                "role": str(fields["roles"][index]),
                "moduleIndex": int(fields["indices"][index]),
                "translationKnown": known,
                "translation": translation,
                "visualLength": float(fields["lengths"][index]),
            })
        return layout

    @staticmethod
    def _empty_imu_state(layout):
        return {
            "sensorId": layout["sensorId"],
            "displayId": layout["displayId"],
            "topic": layout["topic"],
            "role": layout["role"],
            "moduleIndex": layout["moduleIndex"],
            "online": False,
            "ageSec": None,
            "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "eulerDeg": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }

    def _build_snake_model(self):
        """Represent all seven modules while activating only fitted IMUs."""
        requested_count = int(self.get_parameter("robot_module_count").value)
        highest_active = max(item["moduleIndex"] for item in self.imu_layout)
        module_count = max(requested_count, highest_active)
        active_by_index = {
            item["moduleIndex"]: item for item in self.imu_layout
        }
        modules = []
        for module_index in range(1, module_count + 1):
            active = active_by_index.get(module_index)
            modules.append(
                {**deepcopy(active), "active": True}
                if active is not None
                else {
                    "moduleIndex": module_index,
                    "active": False,
                    "sensorId": None,
                    "role": "PLACEHOLDER",
                }
            )
        return {
            "moduleCount": module_count,
            "modules": modules,
            "activeModules": deepcopy(self.imu_layout),
            "translationNote": (
                "Unknown inter-module translations use UI-only visual "
                "spacing and are not calibration or TF data."
            ),
        }

    def _start_http_server(self, websocket_port):
        configured = str(self.get_parameter("ui_directory").value)
        ui_directory = Path(configured).expanduser() if configured else Path(
            get_package_share_directory("my_robot_bringup")
        ) / "ui"
        if not (ui_directory / "index.html").is_file():
            raise RuntimeError(f"Mission-control UI not found: {ui_directory}")

        ui_host = str(self.get_parameter("ui_host").value)
        ui_port = int(self.get_parameter("ui_port").value)
        handler = partial(
            MissionHttpHandler,
            directory=str(ui_directory),
            websocket_port=websocket_port,
            camera_frame_callback=self.get_camera_frame,
            camera_stream_rate=float(
                self.get_parameter("camera_stream_rate").value
            ),
        )
        self.http_server = ThreadingHTTPServer((ui_host, ui_port), handler)
        self.http_server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="mission-control-http",
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f"Mission-control UI: http://{ui_host}:{ui_port}"
        )

    def _create_subscriptions(self):
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor_qos = qos_profile_sensor_data

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("start_pose_topic").value),
            self.start_pose_callback,
            latched_qos,
        )
        self.create_subscription(
            PathMessage,
            str(self.get_parameter("path_topic").value),
            self.path_callback,
            latched_qos,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("distance_traveled_topic").value),
            self.distance_traveled_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("distance_traveled_3d_topic").value),
            self.distance_traveled_3d_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("distance_slam_corrected_topic").value),
            self.distance_slam_corrected_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("distance_from_start_topic").value),
            self.distance_from_start_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            self.diagnostics_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            latched_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self.cloud_callback,
            # RTAB-Map publishes /cloud_map as a latched map product. Matching
            # that QoS also gives a newly started UI the latest cloud at once.
            latched_qos,
        )
        self.create_subscription(
            Info,
            str(self.get_parameter("info_topic").value),
            self.info_callback,
            sensor_qos,
        )
        self.create_subscription(
            MapGraph,
            str(self.get_parameter("map_graph_topic").value),
            self.map_graph_callback,
            latched_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self.camera_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("person_detection_topic").value),
            self.person_detection_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("segment_poses_topic").value),
            self.segment_poses_callback,
            sensor_qos,
        )
        for layout in self.imu_layout:
            self.create_subscription(
                Imu,
                layout["topic"],
                partial(self.imu_callback, layout["sensorId"]),
                sensor_qos,
            )

    def snapshot_json(self):
        with self.state_lock:
            self._refresh_imu_status_locked(time.monotonic())
            data = deepcopy(self.state)
            data["missionSeconds"] = time.monotonic() - self.started_at
            payload = {
                "type": "snapshot",
                "data": data,
                "map": deepcopy(self.map_state),
                # The cloud follows as an SPC1 binary frame. Keeping it out of
                # JSON avoids multi-megabyte parsing pauses in the browser.
                "cloudPoints": [],
            }
        return json.dumps(payload, separators=(",", ":"), allow_nan=False)

    def cloud_binary_snapshot(self):
        """Return the latest non-empty binary cloud for a new browser client."""
        with self.state_lock:
            return self.cloud_packet

    def emit(self, payload):
        self.websocket_hub.publish(payload)

    def pose_callback(self, message):
        pose = pose_to_dict(message.pose)
        with self.state_lock:
            self.state["pose"] = pose
            traveled = self.state["distanceTraveled"]
            traveled_3d = self.state["distanceTraveled3d"]
            slam_corrected = self.state["distanceSlamCorrected"]
            from_start = self.state["distanceFromStart"]
        self.emit({
            "type": "pose",
            "pose": pose,
            "distanceTraveled": traveled,
            "distanceTraveled3d": traveled_3d,
            "distanceSlamCorrected": slam_corrected,
            "distanceFromStart": from_start,
            "missionSeconds": time.monotonic() - self.started_at,
        })

    def start_pose_callback(self, message):
        start = pose_to_dict(message.pose)
        with self.state_lock:
            self.state["start"] = start
        self.emit({"type": "start", "pose": start})

    def path_callback(self, message):
        now = time.monotonic()
        if now - self.last_path_publish < 0.5:
            return
        self.last_path_publish = now
        max_points = max(2, int(self.get_parameter("max_path_points").value))
        stride = max(1, math.ceil(len(message.poses) / max_points))
        selected = message.poses[::stride]
        if message.poses and selected[-1] is not message.poses[-1]:
            selected.append(message.poses[-1])
        points = [pose_to_dict(item.pose) for item in selected]
        with self.state_lock:
            self.state["path"] = points
        self.emit({"type": "path", "points": points})

    def distance_traveled_callback(self, message):
        with self.state_lock:
            self.state["distanceTraveled"] = round(float(message.data), 3)

    def distance_traveled_3d_callback(self, message):
        with self.state_lock:
            self.state["distanceTraveled3d"] = round(float(message.data), 3)

    def distance_slam_corrected_callback(self, message):
        with self.state_lock:
            self.state["distanceSlamCorrected"] = round(
                float(message.data), 3
            )

    def distance_from_start_callback(self, message):
        with self.state_lock:
            self.state["distanceFromStart"] = round(float(message.data), 3)

    def diagnostics_callback(self, message):
        values = {}
        for status in message.status:
            if status.name == "online_slam/topic_rates":
                values.update({item.key: item.value for item in status.values})
        if not values:
            return

        rates = {}
        for name, expected in EXPECTED_RATES.items():
            try:
                rate = float(values.get(f"{name}_hz", 0.0))
                age = float(values.get(f"{name}_age_sec", "inf"))
            except ValueError:
                rate, age = 0.0, float("inf")
            rates[name] = {
                "value": round(rate, 2),
                "expected": expected,
                "ok": math.isfinite(age) and age <= STALE_LIMITS[name],
            }
        with self.state_lock:
            self.state["rates"] = rates
            self.state["mapRate"] = rates["SLAM"]["value"]
        self.emit({"type": "rates", "rates": rates})

    def info_callback(self, message):
        with self.state_lock:
            # wm_state is only RTAB-Map's active working set. It may shrink
            # during normal loop closure, so it must never reset the web map.
            if self.last_graph_node_id == 0:
                self.state["mapNodes"] = max(
                    self.state["mapNodes"], len(message.wm_state)
                )

    def map_graph_callback(self, message):
        """Track the complete optimized graph and detect a real DB restart."""
        node_ids = tuple(int(node_id) for node_id in message.poses_id)
        graph_node_id = max(node_ids, default=0)
        reset_cloud = False
        with self.state_lock:
            if map_graph_restarted(self.last_graph_node_id, graph_node_id):
                self.cloud_packet = None
                self.state["cloudPointCount"] = 0
                self.state["target"] = None
                self.sparse_cloud_streak = 0
                reset_cloud = True
            if graph_node_id > 0:
                self.last_graph_node_id = graph_node_id
            self.state["mapNodes"] = len(node_ids)
        if reset_cloud:
            self.emit({"type": "cloud_reset"})

    def map_callback(self, message):
        now = time.monotonic()
        rate = max(0.1, float(self.get_parameter("map_publish_rate").value))
        if now - self.last_map_publish < 1.0 / rate:
            return
        self.last_map_publish = now
        source_resolution = float(message.info.resolution)
        if source_resolution <= 0.0 or not message.data:
            return

        cell_size = max(
            source_resolution,
            float(self.get_parameter("map_cell_size").value),
        )
        threshold = int(self.get_parameter("occupied_threshold").value)
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)
        width = int(message.info.width)
        values = np.asarray(message.data, dtype=np.int16)
        valid_indices = np.flatnonzero(values >= 0)
        columns = valid_indices % width
        rows = valid_indices // width
        cell_x = np.floor(
            (origin_x + (columns + 0.5) * source_resolution) / cell_size
        ).astype(np.int32)
        cell_y = np.floor(
            (origin_y + (rows + 0.5) * source_resolution) / cell_size
        ).astype(np.int32)
        known_array = np.unique(np.column_stack((cell_x, cell_y)), axis=0)
        occupied_mask = values[valid_indices] >= threshold
        occupied_array = np.unique(
            np.column_stack((cell_x[occupied_mask], cell_y[occupied_mask])),
            axis=0,
        )
        map_state = {
            "cellSize": round(cell_size, 4),
            "known": known_array.tolist(),
            "occupied": occupied_array.tolist(),
        }
        with self.state_lock:
            self.map_state = map_state
            map_nodes = self.state["mapNodes"]
        self.emit({"type": "map", "map": map_state, "mapNodes": map_nodes})

    def cloud_callback(self, message):
        """Replace the pending cloud; conversion runs outside the ROS thread."""
        with self.cloud_input_lock:
            self.latest_cloud_message = message
            self.cloud_input_event.set()

    def _take_latest_cloud_message(self):
        with self.cloud_input_lock:
            message = self.latest_cloud_message
            self.latest_cloud_message = None
            self.cloud_input_event.clear()
            return message

    def _cloud_worker_main(self):
        """Vectorize only the newest cloud and keep the last valid map visible."""
        rate = max(0.1, float(self.get_parameter("cloud_publish_rate").value))
        period = 1.0 / rate
        while self.running:
            self.cloud_input_event.wait(timeout=0.25)
            if not self.running:
                break
            message = self._take_latest_cloud_message()
            if message is None:
                continue
            delay = self.last_cloud_publish + period - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
                message = self._take_latest_cloud_message() or message
            try:
                packet, count = point_cloud_binary_packet(
                    message,
                    max(100, int(self.get_parameter("max_cloud_points").value)),
                    float(self.get_parameter("cloud_min_z").value),
                    float(self.get_parameter("cloud_max_z").value),
                )
            except Exception as error:
                self.get_logger().warning(
                    f"Point-cloud packing failed: {error}",
                    throttle_duration_sec=5.0,
                )
                continue
            self.last_cloud_publish = time.monotonic()
            # RTAB-Map can briefly publish an empty product while rebuilding.
            # Never replace a useful accumulated map with that transient gap.
            if not packet or count == 0:
                continue
            with self.state_lock:
                replace, self.sparse_cloud_streak = cloud_replacement_decision(
                    self.state["cloudPointCount"],
                    count,
                    self.sparse_cloud_streak,
                )
                if not replace:
                    continue
                self.cloud_packet = packet
                self.state["cloudPointCount"] = count
            self.emit(packet)

    def camera_callback(self, message):
        """Replace the pending frame; JPEG encoding runs in its own thread."""
        with self.camera_input_lock:
            self.latest_camera_message = message
            self.camera_input_event.set()

    def _take_latest_camera_message(self):
        with self.camera_input_lock:
            message = self.latest_camera_message
            self.latest_camera_message = None
            self.camera_input_event.clear()
            return message

    def _camera_worker_main(self):
        """Encode only the newest camera image at a bounded output rate."""
        rate = max(0.1, float(self.get_parameter("camera_publish_rate").value))
        period = 1.0 / rate
        while self.running:
            self.camera_input_event.wait(timeout=0.25)
            if not self.running:
                break
            message = self._take_latest_camera_message()
            if message is None:
                continue
            delay = self.last_camera_publish + period - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
                message = self._take_latest_camera_message() or message
            self._encode_camera_message(message)
            self.last_camera_publish = time.monotonic()

    def _encode_camera_message(self, message):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            self._publish_yolo_input(frame, message.header)
            target_width = int(self.get_parameter("camera_width").value)
            if target_width > 0 and frame.shape[1] > target_width:
                ratio = target_width / frame.shape[1]
                frame = cv2.resize(
                    frame,
                    (target_width, round(frame.shape[0] * ratio)),
                    interpolation=cv2.INTER_AREA,
                )
            quality = int(self.get_parameter("jpeg_quality").value)
            success, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )
            if not success:
                return
        except Exception as error:  # cv_bridge reports varying exception types
            self.get_logger().warning(
                f"Camera frame conversion failed: {error}",
                throttle_duration_sec=5.0,
            )
            return
        stamp = time.time()
        jpeg = encoded.tobytes()
        with self.camera_lock:
            self.camera_sequence += 1
            self.last_camera_frame = (self.camera_sequence, stamp, jpeg)
        if bool(self.get_parameter("camera_websocket_enabled").value):
            data = base64.b64encode(jpeg).decode("ascii")
            self.emit({
                "type": "camera",
                "mime": "image/jpeg",
                "data": data,
                "stamp": stamp,
            })

    def _publish_yolo_input(self, frame, source_header):
        """
        Publish a small, throttled latest frame for CPU-only inference.

        The detector no longer subscribes to the full camera stream. This
        removes one large DDS image copy while keeping display encoding and
        person inference independent.
        """
        rate = max(0.1, float(self.get_parameter("yolo_input_rate").value))
        now = time.monotonic()
        if now - self.last_yolo_input_publish < 1.0 / rate:
            return
        if self.yolo_input_publisher.get_subscription_count() == 0:
            return
        target_width = max(
            64, int(self.get_parameter("yolo_input_width").value)
        )
        if frame.shape[1] > target_width:
            ratio = target_width / frame.shape[1]
            detector_frame = cv2.resize(
                frame,
                (target_width, round(frame.shape[0] * ratio)),
                interpolation=cv2.INTER_AREA,
            )
        else:
            detector_frame = frame
        detector_message = self.cv_bridge.cv2_to_imgmsg(
            detector_frame, encoding="bgr8"
        )
        detector_message.header = source_header
        self.yolo_input_publisher.publish(detector_message)
        self.last_yolo_input_publish = now

    def get_camera_frame(self):
        """Return the newest encoded frame without queueing older frames."""
        with self.camera_lock:
            return self.last_camera_frame

    def person_detection_callback(self, message):
        """Forward the bounded person-only JSON detection contract."""
        try:
            detection = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(
                "Ignoring malformed /perception/person_detection JSON",
                throttle_duration_sec=5.0,
            )
            return
        candidates = detection.get("candidates", [])
        if not isinstance(candidates, list):
            return
        try:
            bounded = {
                "detected": bool(detection.get("detected", False)),
                "count": min(max(int(detection.get("count", 0)), 0), 20),
                "candidates": candidates[:20],
                "modelReady": bool(detection.get("modelReady", False)),
                "inferenceMs": round(
                    float(detection.get("inferenceMs", 0.0)), 1
                ),
                "stamp": float(detection.get("stamp", time.time())),
                "error": str(detection.get("error", ""))[:200],
            }
        except (TypeError, ValueError):
            return
        with self.state_lock:
            self.state["personDetection"] = bounded
            if bounded["detected"]:
                for candidate in bounded["candidates"]:
                    position = candidate.get("mapPosition", {})
                    try:
                        coordinates = {
                            axis: float(position[axis])
                            for axis in ("x", "y", "z")
                        }
                    except (KeyError, TypeError, ValueError):
                        continue
                    if not all(math.isfinite(value) for value in coordinates.values()):
                        continue
                    distance = candidate.get("distanceMeters")
                    try:
                        distance = float(distance)
                    except (TypeError, ValueError):
                        distance = None
                    try:
                        confidence = float(candidate.get("confidence", 0.0))
                    except (TypeError, ValueError):
                        confidence = 0.0
                    self.state["target"] = {
                        "id": str(candidate.get("id", "P-01"))[:32],
                        **coordinates,
                        "confidence": confidence if math.isfinite(confidence) else 0.0,
                        "distanceMeters": (
                            distance if distance is not None and math.isfinite(distance)
                            else None
                        ),
                        "localizedAt": bounded["stamp"],
                    }
                    break
        self.emit({"type": "person_detection", "data": bounded})

    def segment_poses_callback(self, message):
        poses = [pose_to_dict(pose) for pose in message.poses]
        with self.state_lock:
            self.state["segmentPoses"] = poses
        self.emit({"type": "segment_poses", "poses": poses})

    def imu_callback(self, sensor_id, message):
        """Forward normalized orientation while keeping each segment separate."""
        x, y, z, w = normalized_quaternion(message.orientation)
        roll, pitch, yaw = quaternion_to_euler_degrees(message.orientation)
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        arrival = time.monotonic()
        with self.state_lock:
            previous = self.state["imus"][sensor_id]
            imu_state = {
                **previous,
                "online": True,
                "ageSec": 0.0,
                "stamp": stamp,
                "frameId": str(message.header.frame_id),
                "quaternion": {
                    "x": round(x, 6), "y": round(y, 6),
                    "z": round(z, 6), "w": round(w, 6),
                },
                "eulerDeg": {
                    "roll": round(roll, 2),
                    "pitch": round(pitch, 2),
                    "yaw": round(yaw, 2),
                },
            }
            self.imu_last_arrival[sensor_id] = arrival
            self.state["imus"][sensor_id] = imu_state
        web_rate = max(
            1.0, float(self.get_parameter("imu_web_publish_rate").value)
        )
        if arrival - self.imu_last_web_publish[sensor_id] < 1.0 / web_rate:
            return
        self.imu_last_web_publish[sensor_id] = arrival
        self.emit({"type": "imu", "imu": imu_state})

    def _refresh_imu_status_locked(self, now):
        timeout = max(
            0.05, float(self.get_parameter("imu_stale_timeout_sec").value)
        )
        for sensor_id, last_arrival in self.imu_last_arrival.items():
            current = self.state["imus"][sensor_id]
            age = None if last_arrival is None else max(0.0, now - last_arrival)
            current["ageSec"] = None if age is None else round(age, 3)
            current["online"] = age is not None and age <= timeout

    def publish_heartbeat(self):
        with self.state_lock:
            self._refresh_imu_status_locked(time.monotonic())
            imus = deepcopy(self.state["imus"])
        self.emit({
            "type": "status",
            "missionSeconds": time.monotonic() - self.started_at,
            "imus": imus,
        })

    def destroy_node(self):
        self.running = False
        self.camera_input_event.set()
        self.cloud_input_event.set()
        for worker in (self.camera_worker, self.cloud_worker):
            if worker.is_alive():
                try:
                    worker.join(timeout=3.0)
                except KeyboardInterrupt:
                    break
        if self.http_server is not None:
            try:
                self.http_server.shutdown()
            except KeyboardInterrupt:
                pass
            finally:
                self.http_server.server_close()
        self.websocket_hub.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlGateway()
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

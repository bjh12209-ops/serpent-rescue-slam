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
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import Image, PointCloud2, PointField
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
    "SLAM": 3.0,
    "IMU": 60.0,
    "EKF": 60.0,
}

STALE_LIMITS = {
    "RGB": 0.5,
    "DEPTH": 0.5,
    "VO": 0.5,
    "SLAM": 2.0,
    "IMU": 0.5,
    "EKF": 0.5,
}


def quaternion_yaw(quaternion):
    """Return REP-103 yaw in radians from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def pose_to_dict(pose):
    """Convert a geometry_msgs pose to the compact browser contract."""
    return {
        "x": round(float(pose.position.x), 4),
        "y": round(float(pose.position.y), 4),
        "z": round(float(pose.position.z), 4),
        "yaw": round(quaternion_yaw(pose.orientation), 5),
    }


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

    def __init__(self, host, port, snapshot_callback, logger):
        self.host = host
        self.port = port
        self.snapshot_callback = snapshot_callback
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
        encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)

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
        self.last_camera_frame = None
        self.camera_sequence = 0
        self.camera_lock = threading.Lock()
        self.cv_bridge = CvBridge()

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
            "battery": None,
            "latency": None,
            "rates": {
                name: {"value": 0.0, "expected": expected, "ok": False}
                for name, expected in EXPECTED_RATES.items()
            },
            "segmentPoses": [],
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
        self.cloud_points = []

        websocket_host = str(self.get_parameter("websocket_host").value)
        websocket_port = int(self.get_parameter("websocket_port").value)
        self.websocket_hub = WebSocketHub(
            websocket_host,
            websocket_port,
            self.snapshot_json,
            self.get_logger(),
        )
        self.websocket_hub.start()

        self.http_server = None
        self.http_thread = None
        if bool(self.get_parameter("serve_ui").value):
            self._start_http_server(websocket_port)

        self._create_subscriptions()
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
        self.declare_parameter(
            "camera_topic", "/camera/camera/color/image_raw"
        )
        self.declare_parameter(
            "person_detection_topic", "/perception/person_detection"
        )
        self.declare_parameter("segment_poses_topic", "/snake/segment_poses")
        self.declare_parameter("map_cell_size", 0.05)
        self.declare_parameter("map_publish_rate", 1.0)
        self.declare_parameter("cloud_publish_rate", 1.0)
        self.declare_parameter("camera_publish_rate", 20.0)
        self.declare_parameter("camera_stream_rate", 20.0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("camera_websocket_enabled", False)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("max_path_points", 4000)
        self.declare_parameter("max_cloud_points", 50000)
        self.declare_parameter("cloud_min_z", -1.5)
        self.declare_parameter("cloud_max_z", 3.0)

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

    def snapshot_json(self):
        with self.state_lock:
            data = deepcopy(self.state)
            data["missionSeconds"] = time.monotonic() - self.started_at
            payload = {
                "type": "snapshot",
                "data": data,
                "map": deepcopy(self.map_state),
                "cloudPoints": deepcopy(self.cloud_points),
            }
        return json.dumps(payload, separators=(",", ":"), allow_nan=False)

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
            current = self.state["mapNodes"]
            self.state["mapNodes"] = max(
                current,
                int(message.ref_id),
                len(message.wm_state),
            )

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
        known = set()
        occupied = set()
        for index, value in enumerate(message.data):
            if value < 0:
                continue
            row, column = divmod(index, width)
            world_x = origin_x + (column + 0.5) * source_resolution
            world_y = origin_y + (row + 0.5) * source_resolution
            cell = (
                math.floor(world_x / cell_size),
                math.floor(world_y / cell_size),
            )
            known.add(cell)
            if value >= threshold:
                occupied.add(cell)
        map_state = {
            "cellSize": round(cell_size, 4),
            "known": [list(cell) for cell in sorted(known)],
            "occupied": [list(cell) for cell in sorted(occupied)],
        }
        with self.state_lock:
            self.map_state = map_state
            map_nodes = self.state["mapNodes"]
        self.emit({"type": "map", "map": map_state, "mapNodes": map_nodes})

    def cloud_callback(self, message):
        now = time.monotonic()
        rate = max(0.1, float(self.get_parameter("cloud_publish_rate").value))
        if now - self.last_cloud_publish < 1.0 / rate:
            return
        self.last_cloud_publish = now
        maximum = max(100, int(self.get_parameter("max_cloud_points").value))
        min_z = float(self.get_parameter("cloud_min_z").value)
        max_z = float(self.get_parameter("cloud_max_z").value)
        fields = {field.name: field for field in message.fields}
        color_name = "rgb" if "rgb" in fields else (
            "rgba" if "rgba" in fields else None
        )
        field_names = ("x", "y", "z", color_name) if color_name else (
            "x", "y", "z"
        )
        color_is_float = bool(
            color_name and fields[color_name].datatype == PointField.FLOAT32
        )
        valid_points = []
        for point in point_cloud2.read_points(
            message, field_names=field_names, skip_nans=True
        ):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not (min_z <= z <= max_z):
                continue
            output = [round(x, 3), round(y, 3), round(z, 3)]
            if color_name:
                if color_is_float:
                    packed = struct.unpack(
                        "<I", struct.pack("<f", float(point[3]))
                    )[0]
                else:
                    packed = int(point[3])
                output.extend([
                    (packed >> 16) & 0xFF,
                    (packed >> 8) & 0xFF,
                    packed & 0xFF,
                ])
            valid_points.append(output)

        # Count valid, height-filtered samples before decimating. Using the
        # organized cloud size here can severely under-fill the browser cloud
        # when most depth pixels are NaN.
        stride = max(1, math.ceil(len(valid_points) / maximum))
        points = valid_points[::stride][:maximum]
        with self.state_lock:
            self.cloud_points = points
        self.emit({"type": "cloud", "points": points})

    def camera_callback(self, message):
        now = time.monotonic()
        rate = max(0.1, float(self.get_parameter("camera_publish_rate").value))
        if now - self.last_camera_publish < 1.0 / rate:
            return
        self.last_camera_publish = now
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
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
        self.emit({"type": "person_detection", "data": bounded})

    def segment_poses_callback(self, message):
        poses = [pose_to_dict(pose) for pose in message.poses]
        with self.state_lock:
            self.state["segmentPoses"] = poses
        self.emit({"type": "segment_poses", "poses": poses})

    def publish_heartbeat(self):
        self.emit({
            "type": "status",
            "missionSeconds": time.monotonic() - self.started_at,
        })

    def destroy_node(self):
        if self.http_server is not None:
            self.http_server.shutdown()
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

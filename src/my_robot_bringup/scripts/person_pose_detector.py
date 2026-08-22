#!/usr/bin/env python3
"""Detect people and localize them with aligned D435 depth and ROS TF."""

from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


COCO_EXTREMITIES = {
    9: "왼손목",
    10: "오른손목",
    15: "왼발목",
    16: "오른발목",
}


def project_root():
    """Return the repository root for source and symlink installs."""
    return Path(__file__).resolve().parents[3]


def yolo_site_package_candidates(configured=""):
    """Return portable virtualenv package paths without hard-coding one PC."""
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    roots = []
    if configured:
        configured_path = Path(configured).expanduser()
        if configured_path.name == "site-packages":
            yield configured_path
        else:
            roots.append(configured_path)
    roots.extend([
        project_root() / ".venv-yolo",
        Path.cwd() / ".venv-yolo",
        Path.home() / "ros2_ws" / ".venv-yolo",
    ])
    seen = set()
    for root in roots:
        candidate = root / "lib" / version / "site-packages"
        resolved = str(candidate.resolve())
        if resolved not in seen:
            seen.add(resolved)
            yield candidate


def import_yolo(configured_site_packages=""):
    """Import Ultralytics, falling back to the project's YOLO virtualenv."""
    try:
        from ultralytics import YOLO
        return YOLO
    except ImportError as original_error:
        for candidate in yolo_site_package_candidates(configured_site_packages):
            if not candidate.is_dir():
                continue
            sys.path.insert(0, str(candidate))
            try:
                from ultralytics import YOLO
                return YOLO
            except ImportError:
                continue
        raise original_error


def resolve_model_path(configured_model):
    """Find a local model in common workspace locations before downloading."""
    expanded = Path(configured_model).expanduser()
    if expanded.is_file() or expanded.parent != Path("."):
        return str(expanded)
    candidates = [
        Path.cwd() / configured_model,
        project_root() / configured_model,
        Path.home() / "ros2_ws" / configured_model,
        Path.home() / configured_model,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return configured_model


def clamp(value, minimum=0.0, maximum=1.0):
    """Clamp a numeric value to a closed interval."""
    return min(max(float(value), minimum), maximum)


def class_name(names, class_id):
    """Resolve an Ultralytics names dict/list without importing torch."""
    index = int(class_id)
    if isinstance(names, dict):
        return str(names.get(index, names.get(str(index), index)))
    if isinstance(names, (list, tuple)) and 0 <= index < len(names):
        return str(names[index])
    return str(index)


def build_person_candidates(
    boxes,
    keypoints,
    names,
    image_width,
    image_height,
    keypoint_confidence=0.3,
    require_extremity=True,
    custom_extremity_classes=(),
):
    """Convert model arrays to the bounded person-only browser contract."""
    width = max(float(image_width), 1.0)
    height = max(float(image_height), 1.0)
    custom_names = {name.strip().lower() for name in custom_extremity_classes}
    candidates = []
    for index, box in enumerate(boxes):
        if len(box) < 6:
            continue
        x1, y1, x2, y2, confidence, class_id = box[:6]
        name = class_name(names, class_id).strip().lower()
        is_person = name == "person"
        is_custom_extremity = name in custom_names
        if not is_person and not is_custom_extremity:
            continue

        extremities = []
        if index < len(keypoints):
            for keypoint_index, point_name in COCO_EXTREMITIES.items():
                if keypoint_index >= len(keypoints[index]):
                    continue
                point = keypoints[index][keypoint_index]
                if len(point) < 2:
                    continue
                point_confidence = float(point[2]) if len(point) > 2 else 1.0
                if point_confidence < keypoint_confidence:
                    continue
                extremities.append({
                    "name": point_name,
                    "x": round(clamp(point[0] / width), 5),
                    "y": round(clamp(point[1] / height), 5),
                    "confidence": round(clamp(point_confidence), 4),
                })
        elif is_custom_extremity:
            extremities.append({
                "name": name,
                "x": round(clamp((float(x1) + float(x2)) / (2.0 * width)), 5),
                "y": round(clamp((float(y1) + float(y2)) / (2.0 * height)), 5),
                "confidence": round(clamp(confidence), 4),
            })

        if require_extremity and not extremities:
            continue
        candidates.append({
            "id": f"P-{len(candidates) + 1:02d}",
            "confidence": round(clamp(confidence), 4),
            "bbox": [
                round(clamp(float(x1) / width), 5),
                round(clamp(float(y1) / height), 5),
                round(clamp(float(x2) / width), 5),
                round(clamp(float(y2) / height), 5),
            ],
            "extremities": extremities,
        })
    return candidates


def estimate_optical_position(
    candidate,
    depth_image,
    camera_matrix,
    depth_scale=0.001,
    minimum_depth=0.2,
    maximum_depth=6.0,
):
    """
    Estimate a robust person center in the camera optical frame.

    The central torso portion of the YOLO box is used instead of a single
    pixel, which rejects depth holes and most background around limbs.
    """
    if depth_image is None or depth_image.ndim != 2:
        return None
    bbox = candidate.get("bbox", [])
    if len(bbox) != 4 or len(camera_matrix) < 9:
        return None
    height, width = depth_image.shape
    x1, y1, x2, y2 = [clamp(value) for value in bbox]
    if x2 <= x1 or y2 <= y1:
        return None
    box_width = x2 - x1
    box_height = y2 - y1
    roi_x1 = max(0, int((x1 + 0.25 * box_width) * width))
    roi_x2 = min(width, int((x2 - 0.25 * box_width) * width) + 1)
    roi_y1 = max(0, int((y1 + 0.20 * box_height) * height))
    roi_y2 = min(height, int((y2 - 0.20 * box_height) * height) + 1)
    if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
        return None
    roi_meters = depth_image[roi_y1:roi_y2, roi_x1:roi_x2].astype(
        np.float32, copy=False
    ) * float(depth_scale)
    valid = roi_meters[
        np.isfinite(roi_meters)
        & (roi_meters >= float(minimum_depth))
        & (roi_meters <= float(maximum_depth))
    ]
    if valid.size < 12:
        return None
    depth = float(np.median(valid))
    u = (x1 + x2) * 0.5 * width
    v = (y1 + y2) * 0.5 * height
    fx = float(camera_matrix[0])
    fy = float(camera_matrix[4])
    cx = float(camera_matrix[2])
    cy = float(camera_matrix[5])
    if fx <= 0.0 or fy <= 0.0:
        return None
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    return {
        "x": x,
        "y": y,
        "z": depth,
        "distance": math.sqrt(x * x + y * y + depth * depth),
        "samples": int(valid.size),
    }


class DetectionDebouncer:
    """Require consecutive frames to confirm and clear person detections."""

    def __init__(self, confirm_frames=2, clear_frames=3):
        self.confirm_frames = max(1, int(confirm_frames))
        self.clear_frames = max(1, int(clear_frames))
        self.positive_count = 0
        self.negative_count = 0
        self.detected = False

    def update(self, has_candidate):
        """Update stable detection state from one raw inference result."""
        if has_candidate:
            self.positive_count += 1
            self.negative_count = 0
            if self.positive_count >= self.confirm_frames:
                self.detected = True
        else:
            self.positive_count = 0
            self.negative_count += 1
            if self.negative_count >= self.clear_frames:
                self.detected = False
        return self.detected


class PersonPoseDetector(Node):
    """Run latest-frame-only YOLO inference without blocking ROS callbacks."""

    def __init__(self):
        super().__init__("person_pose_detector")
        self.declare_parameter("image_topic", "/mission_control/yolo_input")
        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("minimum_depth", 0.2)
        self.declare_parameter("maximum_depth", 6.0)
        self.declare_parameter("maximum_depth_time_error", 0.12)
        self.declare_parameter("model", "yolo26n-pose.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("image_size", 416)
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("keypoint_confidence", 0.25)
        self.declare_parameter("max_inference_rate", 2.0)
        self.declare_parameter("require_extremity", False)
        self.declare_parameter("python_site_packages", "")
        self.declare_parameter("confirm_frames", 2)
        self.declare_parameter("clear_frames", 3)
        self.declare_parameter(
            "custom_extremity_classes", ["hand", "finger", "foot"]
        )

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.latest_message = None
        self.depth_lock = threading.Lock()
        self.depth_messages = deque(maxlen=8)
        self.camera_info = None
        self.running = True
        self.model_ready = False
        self.model_error = "YOLO model is loading"
        self.last_candidates = []
        self.last_inference_ms = 0.0
        self.last_publish_stamp = 0.0
        self.debouncer = DetectionDebouncer(
            self.get_parameter("confirm_frames").value,
            self.get_parameter("clear_frames").value,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            String, "/perception/person_detection", 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(2.0, self.publish_status_if_idle)
        self.worker = threading.Thread(
            target=self.worker_main,
            name="yolo-person-pose",
            daemon=True,
        )
        self.worker.start()

    def image_callback(self, message):
        """Replace any queued image so inference can never build a backlog."""
        with self.frame_lock:
            self.latest_message = message
            self.frame_event.set()

    def depth_callback(self, message):
        """Keep a short depth history for timestamp matching after inference."""
        with self.depth_lock:
            self.depth_messages.append(message)

    def camera_info_callback(self, message):
        """Keep the color intrinsics used by aligned depth."""
        with self.depth_lock:
            self.camera_info = message

    @staticmethod
    def stamp_seconds(stamp):
        """Convert a ROS builtin time message to floating-point seconds."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def closest_depth(self, stamp):
        """Return depth and intrinsics closest to the inferred RGB frame."""
        requested = self.stamp_seconds(stamp)
        with self.depth_lock:
            messages = tuple(self.depth_messages)
            camera_info = self.camera_info
        if not messages or camera_info is None:
            return None, None
        message = min(
            messages,
            key=lambda item: abs(
                self.stamp_seconds(item.header.stamp) - requested
            ),
        )
        time_error = abs(
            self.stamp_seconds(message.header.stamp) - requested
        )
        maximum_error = float(
            self.get_parameter("maximum_depth_time_error").value
        )
        if time_error > maximum_error:
            return None, camera_info
        return message, camera_info

    def localize_candidates(self, candidates, rgb_stamp):
        """Add camera distance and map-frame position to detected people."""
        if not candidates:
            return candidates
        depth_message, camera_info = self.closest_depth(rgb_stamp)
        if depth_message is None or camera_info is None:
            return candidates
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
        except Exception as error:
            self.get_logger().warning(
                f"Person depth conversion failed: {error}",
                throttle_duration_sec=5.0,
            )
            return candidates

        map_frame = str(self.get_parameter("map_frame").value)
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                depth_message.header.frame_id,
                Time.from_msg(depth_message.header.stamp),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Person map transform unavailable: {error}",
                throttle_duration_sec=5.0,
            )

        for candidate in candidates:
            position = estimate_optical_position(
                candidate,
                np.asarray(depth_image),
                camera_info.k,
                depth_scale=float(self.get_parameter("depth_scale").value),
                minimum_depth=float(
                    self.get_parameter("minimum_depth").value
                ),
                maximum_depth=float(
                    self.get_parameter("maximum_depth").value
                ),
            )
            if position is None:
                candidate["depthValid"] = False
                continue
            candidate["depthValid"] = True
            candidate["distanceMeters"] = round(position["distance"], 2)
            candidate["cameraPosition"] = {
                axis: round(position[axis], 3) for axis in ("x", "y", "z")
            }
            if transform is None:
                continue
            camera_point = PointStamped()
            camera_point.header = depth_message.header
            camera_point.point.x = position["x"]
            camera_point.point.y = position["y"]
            camera_point.point.z = position["z"]
            map_point = do_transform_point(camera_point, transform)
            candidate["mapPosition"] = {
                "x": round(float(map_point.point.x), 3),
                "y": round(float(map_point.point.y), 3),
                "z": round(float(map_point.point.z), 3),
                "frameId": map_frame,
            }
        return candidates

    def take_latest_message(self):
        """Atomically consume the newest camera message."""
        with self.frame_lock:
            message = self.latest_message
            self.latest_message = None
            self.frame_event.clear()
        return message

    def worker_main(self):
        """Load YOLO and infer in a non-ROS worker thread."""
        try:
            YOLO = import_yolo(
                str(self.get_parameter("python_site_packages").value)
            )
        except ImportError:
            self.model_error = (
                "ultralytics is not installed; see my_robot_bringup README"
            )
            self.get_logger().error(self.model_error)
            return

        configured_model = str(self.get_parameter("model").value)
        model_path = resolve_model_path(configured_model)
        try:
            model = YOLO(model_path)
        except Exception as error:  # model loaders raise backend-specific errors
            self.model_error = f"Failed to load YOLO model: {error}"
            self.get_logger().error(self.model_error)
            return

        self.model_ready = True
        self.model_error = ""
        self.get_logger().info(
            f"Person-only YOLO ready: {model_path}; latest-frame mode"
        )
        inference_period = 1.0 / max(
            float(self.get_parameter("max_inference_rate").value), 0.1
        )
        next_inference = 0.0
        while self.running:
            self.frame_event.wait(timeout=0.25)
            if not self.running:
                break
            message = self.take_latest_message()
            if message is None:
                continue
            delay = next_inference - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
                message = self.take_latest_message() or message
            started = time.perf_counter()
            try:
                frame = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding="bgr8"
                )
                result = model.predict(
                    source=frame,
                    imgsz=int(self.get_parameter("image_size").value),
                    conf=float(self.get_parameter("confidence").value),
                    device=str(self.get_parameter("device").value),
                    verbose=False,
                )[0]
                boxes = []
                if result.boxes is not None:
                    xyxy = result.boxes.xyxy.cpu().tolist()
                    confidence = result.boxes.conf.cpu().tolist()
                    classes = result.boxes.cls.cpu().tolist()
                    boxes = [
                        [*coordinates, score, class_id]
                        for coordinates, score, class_id in zip(
                            xyxy, confidence, classes
                        )
                    ]
                keypoints = []
                if result.keypoints is not None:
                    keypoints = result.keypoints.data.cpu().tolist()
                candidates = build_person_candidates(
                    boxes=boxes,
                    keypoints=keypoints,
                    names=result.names,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                    keypoint_confidence=float(
                        self.get_parameter("keypoint_confidence").value
                    ),
                    require_extremity=bool(
                        self.get_parameter("require_extremity").value
                    ),
                    custom_extremity_classes=self.get_parameter(
                        "custom_extremity_classes"
                    ).value,
                )
                candidates = self.localize_candidates(
                    candidates, message.header.stamp
                )
            except Exception as error:  # inference backends vary by platform
                self.get_logger().error(
                    f"YOLO inference failed: {error}",
                    throttle_duration_sec=5.0,
                )
                candidates = []

            self.last_inference_ms = (time.perf_counter() - started) * 1000.0
            detected = self.debouncer.update(bool(candidates))
            if candidates:
                self.last_candidates = candidates
            elif not detected:
                self.last_candidates = []
            self.publish_detection(detected, self.last_candidates)
            next_inference = time.monotonic() + inference_period

    def publish_detection(self, detected, candidates):
        """Publish compact normalized boxes and extremity keypoints as JSON."""
        stamp = time.time()
        payload = {
            "stamp": stamp,
            "detected": bool(detected),
            "count": len(candidates) if detected else 0,
            "candidates": candidates if detected else [],
            "modelReady": self.model_ready,
            "inferenceMs": round(self.last_inference_ms, 1),
        }
        self.publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self.last_publish_stamp = stamp

    def publish_status_if_idle(self):
        """Keep the UI informed when the model or camera is unavailable."""
        if time.time() - self.last_publish_stamp < 2.0:
            return
        payload = {
            "stamp": time.time(),
            "detected": False,
            "count": 0,
            "candidates": [],
            "modelReady": self.model_ready,
            "inferenceMs": round(self.last_inference_ms, 1),
            "error": self.model_error,
        }
        self.publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def destroy_node(self):
        """Stop the inference worker before destroying ROS resources."""
        self.running = False
        self.frame_event.set()
        if self.worker.is_alive():
            try:
                self.worker.join(timeout=3.0)
            except KeyboardInterrupt:
                pass
        super().destroy_node()


def main(args=None):
    """Run the person-only YOLO pose detector."""
    rclpy.init(args=args)
    node = PersonPoseDetector()
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

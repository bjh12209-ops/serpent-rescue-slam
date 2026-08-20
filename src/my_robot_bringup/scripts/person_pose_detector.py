#!/usr/bin/env python3
"""Detect only people with visible extremities using an Ultralytics YOLO model."""

import json
from pathlib import Path
import threading
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


COCO_EXTREMITIES = {
    9: "왼손목",
    10: "오른손목",
    15: "왼발목",
    16: "오른발목",
}


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
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("model", "yolo26n-pose.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("image_size", 416)
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("keypoint_confidence", 0.30)
        self.declare_parameter("max_inference_rate", 5.0)
        self.declare_parameter("require_extremity", True)
        self.declare_parameter("confirm_frames", 2)
        self.declare_parameter("clear_frames", 3)
        self.declare_parameter(
            "custom_extremity_classes", ["hand", "finger", "foot"]
        )

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.latest_message = None
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

        self.publisher = self.create_publisher(
            String, "/perception/person_detection", 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
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
            from ultralytics import YOLO
        except ImportError:
            self.model_error = (
                "ultralytics is not installed; see my_robot_bringup README"
            )
            self.get_logger().error(self.model_error)
            return

        configured_model = str(self.get_parameter("model").value)
        model_path = str(Path(configured_model).expanduser())
        if "/" not in configured_model and not configured_model.startswith("."):
            model_path = configured_model
        try:
            model = YOLO(model_path)
        except Exception as error:  # model loaders raise backend-specific errors
            self.model_error = f"Failed to load YOLO model: {error}"
            self.get_logger().error(self.model_error)
            return

        self.model_ready = True
        self.model_error = ""
        self.get_logger().info(
            f"Person-only YOLO ready: {configured_model}; latest-frame mode"
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
            self.worker.join(timeout=3.0)
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

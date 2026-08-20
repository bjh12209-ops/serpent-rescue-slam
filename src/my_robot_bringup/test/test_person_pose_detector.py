"""Tests for the person-only YOLO result filter."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "person_pose_detector.py"
SPEC = spec_from_file_location("person_pose_detector", SCRIPT)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def pose_with_left_wrist(confidence=0.9):
    """Return one COCO pose with only the left wrist visible."""
    points = [[0.0, 0.0, 0.0] for _ in range(17)]
    points[9] = [40.0, 50.0, confidence]
    return points


def test_only_person_class_is_accepted():
    boxes = [
        [10, 10, 90, 90, 0.8, 0],
        [20, 20, 80, 80, 0.99, 1],
    ]
    keypoints = [pose_with_left_wrist(), pose_with_left_wrist()]
    result = MODULE.build_person_candidates(
        boxes, keypoints, {0: "person", 1: "car"}, 100, 100
    )
    assert len(result) == 1
    assert result[0]["extremities"][0]["name"] == "왼손목"


def test_person_without_extremity_is_rejected():
    result = MODULE.build_person_candidates(
        [[0, 0, 100, 100, 0.8, 0]],
        [pose_with_left_wrist(confidence=0.1)],
        {0: "person"},
        100,
        100,
        keypoint_confidence=0.3,
        require_extremity=True,
    )
    assert result == []


def test_custom_finger_model_is_supported():
    result = MODULE.build_person_candidates(
        [[20, 30, 40, 50, 0.91, 0]],
        [],
        {0: "finger"},
        100,
        100,
        custom_extremity_classes=["finger"],
    )
    assert len(result) == 1
    assert result[0]["extremities"][0]["name"] == "finger"
    assert result[0]["extremities"][0]["x"] == 0.3


def test_detection_debouncer_prevents_flicker():
    debounce = MODULE.DetectionDebouncer(confirm_frames=2, clear_frames=3)
    assert debounce.update(True) is False
    assert debounce.update(True) is True
    assert debounce.update(False) is True
    assert debounce.update(False) is True
    assert debounce.update(False) is False

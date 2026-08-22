"""Tests for jitter-resistant SLAM distance telemetry."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).parents[1] / "scripts" / "slam_telemetry.py"
SPEC = spec_from_file_location("slam_telemetry", SCRIPT)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def stamped_pose(x, y, z=0.0):
    """Create the position shape consumed by path_length()."""
    position = SimpleNamespace(x=x, y=y, z=z)
    return SimpleNamespace(pose=SimpleNamespace(position=position))


def test_stationary_jitter_does_not_accumulate():
    accumulator = MODULE.DistanceAccumulator(min_step=0.025)
    for index in range(600):
        jitter = 0.004 if index % 2 else -0.004
        accumulator.update((jitter, -jitter, jitter / 2), index / 60.0)
    assert accumulator.distance_2d == 0.0
    assert accumulator.distance_3d == 0.0


def test_slow_motion_crosses_deadband_without_being_lost():
    accumulator = MODULE.DistanceAccumulator(min_step=0.025)
    for index in range(301):
        accumulator.update((index * 0.001, 0.0, 0.0), index / 60.0)
    assert 0.25 < accumulator.distance_2d <= 0.31
    assert accumulator.distance_3d == accumulator.distance_2d


def test_impossible_jump_is_rejected():
    accumulator = MODULE.DistanceAccumulator(
        min_step=0.01, max_speed=1.0, max_jump=0.5
    )
    accumulator.update((0.0, 0.0, 0.0), 0.0)
    accumulator.update((1.0, 0.0, 0.0), 0.1)
    accumulator.update((0.02, 0.0, 0.0), 0.2)
    assert accumulator.distance_2d < 0.03


def test_distance_recovers_after_rejected_odometry_jump():
    accumulator = MODULE.DistanceAccumulator(
        min_step=0.01,
        max_speed=1.0,
        max_jump=0.5,
        smoothing_alpha=1.0,
    )
    accumulator.update((0.0, 0.0, 0.0), 0.0)
    accumulator.update((1.0, 0.0, 0.0), 0.1)  # rejected reset/outlier
    for index in range(1, 7):
        accumulator.update((1.0 + index * 0.02, 0.0, 0.0), 0.1 + index * 0.1)
    assert 0.09 < accumulator.distance_2d < 0.13


def test_optimized_path_length_can_be_xy_or_3d():
    poses = [stamped_pose(0, 0, 0), stamped_pose(3, 4, 0), stamped_pose(3, 4, 12)]
    assert MODULE.path_length(poses, dimensions=2) == 5.0
    assert MODULE.path_length(poses, dimensions=3) == 17.0

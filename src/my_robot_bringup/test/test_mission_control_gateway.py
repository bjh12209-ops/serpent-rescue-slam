"""Tests for the browser gateway's IMU orientation contract."""

from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
import struct
from types import SimpleNamespace

from sensor_msgs.msg import PointCloud2, PointField


SCRIPT = Path(__file__).parents[1] / "scripts" / "mission_control_gateway.py"
SPEC = spec_from_file_location("mission_control_gateway", SCRIPT)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    return SimpleNamespace(x=x, y=y, z=z, w=w)


def test_zero_quaternion_falls_back_to_identity():
    assert MODULE.normalized_quaternion(quaternion(w=0.0)) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_is_normalized_before_forwarding():
    x, y, z, w = MODULE.normalized_quaternion(quaternion(z=2.0, w=2.0))
    assert math.isclose(math.sqrt(x * x + y * y + z * z + w * w), 1.0)


def test_yaw_is_only_converted_for_display():
    half_angle = math.pi / 4
    roll, pitch, yaw = MODULE.quaternion_to_euler_degrees(
        quaternion(z=math.sin(half_angle), w=math.cos(half_angle))
    )
    assert math.isclose(roll, 0.0, abs_tol=1e-8)
    assert math.isclose(pitch, 0.0, abs_tol=1e-8)
    assert math.isclose(yaw, 90.0, abs_tol=1e-8)


def packed_cloud(points):
    """Create a minimal XYZ/RGB PointCloud2 for the binary encoder."""
    message = PointCloud2()
    message.height = 1
    message.width = len(points)
    message.is_bigendian = False
    message.is_dense = False
    message.point_step = 16
    message.row_step = message.point_step * message.width
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    message.data = b"".join(
        struct.pack("<fffI", x, y, z, rgb) for x, y, z, rgb in points
    )
    return message


def test_cloud_packet_is_bounded_colored_and_filters_invalid_height():
    message = packed_cloud([
        (0.0, 0.0, 0.0, 0x112233),
        (1.0, 0.0, 0.2, 0x445566),
        (2.0, 0.0, 9.0, 0xFFFFFF),
        (3.0, 0.0, 0.4, 0x778899),
    ])
    packet, count = MODULE.point_cloud_binary_packet(
        message, maximum=2, min_z=-1.0, max_z=1.0
    )
    assert packet[:4] == b"SPC1"
    assert struct.unpack_from("<I", packet, 4)[0] == 2
    assert count == 2
    assert len(packet) == 8 + 2 * 16
    assert tuple(packet[20:23]) == (0x11, 0x22, 0x33)


def test_isolated_sparse_cloud_does_not_replace_accumulated_map():
    replace, streak = MODULE.cloud_replacement_decision(30000, 16, 0)
    assert replace is False
    assert streak == 1
    replace, streak = MODULE.cloud_replacement_decision(30000, 28000, streak)
    assert replace is True
    assert streak == 0


def test_persistent_sparse_cloud_never_erases_the_accumulated_map():
    streak = 0
    for expected_streak in range(1, 101):
        replace, streak = MODULE.cloud_replacement_decision(
            30000, 16, streak
        )
        assert replace is False
        assert streak == expected_streak


def test_only_complete_graph_id_rewind_is_a_map_restart():
    assert MODULE.map_graph_restarted(120, 1) is True
    assert MODULE.map_graph_restarted(120, 20) is True
    assert MODULE.map_graph_restarted(120, 119) is False
    assert MODULE.map_graph_restarted(120, 121) is False
    assert MODULE.map_graph_restarted(1, 1) is False
    assert MODULE.map_graph_restarted(120, 0) is False

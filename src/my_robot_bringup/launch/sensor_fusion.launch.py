from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("my_robot_bringup"))
    sensor_launch = package_share / "launch" / "sensor_bringup.launch.py"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sensor_launch)),
        ),
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[str(package_share / "config" / "rgbd_odometry.yaml")],
            arguments=["--ros-args", "--log-level", "warn"],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
                ("imu", "/imu_50/data"),
                ("odom", "/visual_odom"),
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[str(package_share / "config" / "ekf.yaml")],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),
    ])

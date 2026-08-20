from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_launch = Path(
        get_package_share_directory("wt901c485_driver")
    ) / "launch" / "wt901c485.launch.py"

    extrinsic_arguments = [
        # Temporary sensor-box convention: base_link is colocated with the IMU.
        # The D435 is mounted 5 cm forward with matching REP-103 axes.
        DeclareLaunchArgument("camera_x", default_value="0.05"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_serial",
            default_value="254622073455",
            description="D435 serial number.",
        ),
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            description="Start the WT901 driver and publish its static TF.",
        ),
        *extrinsic_arguments,
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="camera",
            name="camera",
            output="screen",
            parameters=[{
                "serial_no": ParameterValue(
                    LaunchConfiguration("camera_serial"), value_type=str
                ),
                "enable_color": True,
                "enable_depth": True,
                "enable_infra": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_sync": True,
                "align_depth.enable": True,
                "pointcloud.enable": False,
                "enable_gyro": False,
                "enable_accel": False,
                "rgb_camera.color_profile": "640,480,30",
                "depth_module.depth_profile": "640,480,30",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(imu_launch)),
            condition=IfCondition(LaunchConfiguration("use_imu")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_tf",
            arguments=[
                "--x", LaunchConfiguration("camera_x"),
                "--y", LaunchConfiguration("camera_y"),
                "--z", LaunchConfiguration("camera_z"),
                "--roll", LaunchConfiguration("camera_roll"),
                "--pitch", LaunchConfiguration("camera_pitch"),
                "--yaw", LaunchConfiguration("camera_yaw"),
                "--frame-id", "base_link",
                "--child-frame-id", "camera_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_tf",
            condition=IfCondition(LaunchConfiguration("use_imu")),
            arguments=[
                "--x", LaunchConfiguration("imu_x"),
                "--y", LaunchConfiguration("imu_y"),
                "--z", LaunchConfiguration("imu_z"),
                "--roll", LaunchConfiguration("imu_roll"),
                "--pitch", LaunchConfiguration("imu_pitch"),
                "--yaw", LaunchConfiguration("imu_yaw"),
                "--frame-id", "base_link",
                "--child-frame-id", "imu_52_link",
            ],
        ),
    ])

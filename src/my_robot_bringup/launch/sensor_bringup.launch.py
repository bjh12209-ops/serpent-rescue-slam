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
        # Camera is the head/base reference. IMU 0x50 is measured 5 cm behind
        # and 2 cm above it, with matching REP-103 axes.
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("head_imu_x", default_value="-0.05"),
        DeclareLaunchArgument("head_imu_y", default_value="0.0"),
        DeclareLaunchArgument("head_imu_z", default_value="0.02"),
        DeclareLaunchArgument("head_imu_roll", default_value="0.0"),
        DeclareLaunchArgument("head_imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("head_imu_yaw", default_value="0.0"),
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
                # RealSense recommends 848x480@30 for D435 depth quality.
                # High Accuracy rejects low-confidence stereo matches instead
                # of spraying them into every later SLAM cloud snapshot.
                "rgb_camera.color_profile": "848,480,30",
                "depth_module.depth_profile": "848,480,30",
                "depth_module.visual_preset": 3,
                "depth_module.enable_auto_exposure": True,
                "depth_module.emitter_enabled": 1,
                "depth_module.laser_power": 360.0,
                "depth_module.frames_queue_size": 2,
                # Spatial disparity filtering improves near edges without the
                # motion trails a temporal filter can create on a shaking head.
                "disparity_filter.enable": True,
                "spatial_filter.enable": True,
                "spatial_filter.filter_magnitude": 2,
                "spatial_filter.filter_smooth_alpha": 0.5,
                "spatial_filter.filter_smooth_delta": 20.0,
                "spatial_filter.holes_fill": 1,
                "temporal_filter.enable": False,
                "hole_filling_filter.enable": False,
                "disparity_to_depth.enable": True,
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
            name="camera_to_head_imu_tf",
            condition=IfCondition(LaunchConfiguration("use_imu")),
            arguments=[
                "--x", LaunchConfiguration("head_imu_x"),
                "--y", LaunchConfiguration("head_imu_y"),
                "--z", LaunchConfiguration("head_imu_z"),
                "--roll", LaunchConfiguration("head_imu_roll"),
                "--pitch", LaunchConfiguration("head_imu_pitch"),
                "--yaw", LaunchConfiguration("head_imu_yaw"),
                "--frame-id", "camera_link",
                "--child-frame-id", "imu_50_link",
            ],
        ),
    ])

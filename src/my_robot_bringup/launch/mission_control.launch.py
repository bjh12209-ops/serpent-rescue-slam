"""Launch the read-only ROS telemetry gateway and browser UI server."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_layout = Path(
        get_package_share_directory("my_robot_bringup")
    ) / "config" / "snake_imu_layout.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("websocket_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("websocket_port", default_value="8765"),
        DeclareLaunchArgument("ui_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("ui_port", default_value="8080"),
        DeclareLaunchArgument("serve_ui", default_value="true"),
        DeclareLaunchArgument("map_cell_size", default_value="0.05"),
        DeclareLaunchArgument("max_cloud_points", default_value="30000"),
        DeclareLaunchArgument("cloud_min_z", default_value="-1.5"),
        DeclareLaunchArgument("cloud_max_z", default_value="3.0"),
        DeclareLaunchArgument("camera_publish_rate", default_value="20.0"),
        DeclareLaunchArgument("camera_stream_rate", default_value="20.0"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("jpeg_quality", default_value="65"),
        DeclareLaunchArgument("yolo_input_rate", default_value="2.0"),
        DeclareLaunchArgument("yolo_input_width", default_value="320"),
        DeclareLaunchArgument("enable_person_detection", default_value="true"),
        DeclareLaunchArgument("yolo_model", default_value="yolo26n-pose.pt"),
        DeclareLaunchArgument("yolo_device", default_value="cpu"),
        DeclareLaunchArgument("yolo_image_size", default_value="320"),
        DeclareLaunchArgument("yolo_rate", default_value="2.0"),
        DeclareLaunchArgument("yolo_confidence", default_value="0.25"),
        DeclareLaunchArgument("yolo_require_extremity", default_value="false"),
        Node(
            package="my_robot_bringup",
            executable="mission_control_gateway.py",
            name="mission_control_gateway",
            output="screen",
            parameters=[str(imu_layout), {
                "websocket_host": LaunchConfiguration("websocket_host"),
                "websocket_port": ParameterValue(
                    LaunchConfiguration("websocket_port"), value_type=int
                ),
                "ui_host": LaunchConfiguration("ui_host"),
                "ui_port": ParameterValue(
                    LaunchConfiguration("ui_port"), value_type=int
                ),
                "serve_ui": ParameterValue(
                    LaunchConfiguration("serve_ui"), value_type=bool
                ),
                "map_cell_size": ParameterValue(
                    LaunchConfiguration("map_cell_size"), value_type=float
                ),
                "max_cloud_points": ParameterValue(
                    LaunchConfiguration("max_cloud_points"), value_type=int
                ),
                "cloud_min_z": ParameterValue(
                    LaunchConfiguration("cloud_min_z"), value_type=float
                ),
                "cloud_max_z": ParameterValue(
                    LaunchConfiguration("cloud_max_z"), value_type=float
                ),
                "camera_publish_rate": ParameterValue(
                    LaunchConfiguration("camera_publish_rate"), value_type=float
                ),
                "camera_stream_rate": ParameterValue(
                    LaunchConfiguration("camera_stream_rate"), value_type=float
                ),
                "camera_width": ParameterValue(
                    LaunchConfiguration("camera_width"), value_type=int
                ),
                "jpeg_quality": ParameterValue(
                    LaunchConfiguration("jpeg_quality"), value_type=int
                ),
                "yolo_input_rate": ParameterValue(
                    LaunchConfiguration("yolo_input_rate"), value_type=float
                ),
                "yolo_input_width": ParameterValue(
                    LaunchConfiguration("yolo_input_width"), value_type=int
                ),
            }],
        ),
        Node(
            package="my_robot_bringup",
            executable="person_pose_detector.py",
            name="person_pose_detector",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_person_detection")),
            parameters=[{
                "model": LaunchConfiguration("yolo_model"),
                "device": LaunchConfiguration("yolo_device"),
                "image_size": ParameterValue(
                    LaunchConfiguration("yolo_image_size"), value_type=int
                ),
                "max_inference_rate": ParameterValue(
                    LaunchConfiguration("yolo_rate"), value_type=float
                ),
                "confidence": ParameterValue(
                    LaunchConfiguration("yolo_confidence"), value_type=float
                ),
                "require_extremity": ParameterValue(
                    LaunchConfiguration("yolo_require_extremity"), value_type=bool
                ),
            }],
        ),
    ])

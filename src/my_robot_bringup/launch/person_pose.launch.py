"""Launch latest-frame-only person extremity perception."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="yolo26n-pose.pt",
            description="Official pose model or custom hand/finger YOLO model.",
        ),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("image_size", default_value="416"),
        DeclareLaunchArgument("max_inference_rate", default_value="5.0"),
        DeclareLaunchArgument("require_extremity", default_value="true"),
        Node(
            package="my_robot_bringup",
            executable="person_pose_detector.py",
            name="person_pose_detector",
            output="screen",
            parameters=[{
                "model": LaunchConfiguration("model"),
                "device": LaunchConfiguration("device"),
                "image_size": ParameterValue(
                    LaunchConfiguration("image_size"), value_type=int
                ),
                "max_inference_rate": ParameterValue(
                    LaunchConfiguration("max_inference_rate"), value_type=float
                ),
                "require_extremity": ParameterValue(
                    LaunchConfiguration("require_extremity"), value_type=bool
                ),
            }],
        ),
    ])

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("my_robot_bringup"))
    fusion_launch = package_share / "launch" / "sensor_fusion.launch.py"
    rtabmap_params = str(package_share / "config" / "rtabmap.yaml")

    common_remappings = [
        ("rgb/image", "/camera/camera/color/image_raw"),
        ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
        ("rgb/camera_info", "/camera/camera/color/camera_info"),
        ("imu", "/imu_50/data"),
        ("odom", "/odometry/filtered"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/online_slam.db",
            description="Existing RTAB-Map database used for localization.",
        ),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(fusion_launch)),
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                rtabmap_params,
                {
                    "database_path": LaunchConfiguration("database_path"),
                    "Mem/IncrementalMemory": "false",
                    "Mem/InitWMWithAllNodes": "true",
                },
            ],
            remappings=common_remappings,
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[rtabmap_params],
            remappings=common_remappings,
        ),
    ])

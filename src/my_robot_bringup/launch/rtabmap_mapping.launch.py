from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("my_robot_bringup"))
    fusion_launch = package_share / "launch" / "sensor_fusion.launch.py"
    rtabmap_params = str(package_share / "config" / "rtabmap.yaml")
    rviz_config = str(package_share / "config" / "online_slam.rviz")
    topdown_rviz_config = str(
        package_share / "config" / "online_slam_topdown.rviz"
    )

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
            # Keep the previous default file intact. A malformed SQLite DB
            # should not make every no-argument mapping launch crash.
            default_value="~/.ros/online_slam_optimized.db",
            description="RTAB-Map database to create or continue.",
        ),
        DeclareLaunchArgument(
            "reset_database",
            default_value="false",
            description="Delete database_path at startup before beginning a fresh map.",
        ),
        DeclareLaunchArgument(
            "rtabmap_viz",
            default_value="false",
            description="Run the RTAB-Map GUI (costly on the Intel N97).",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Show the online 3-D map, robot pose, path, TF, and RGB.",
        ),
        DeclareLaunchArgument(
            "topdown",
            default_value="false",
            description="Use the robot-centered top-down game-map RViz view.",
        ),
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
                    "delete_db_on_start": ParameterValue(
                        LaunchConfiguration("reset_database"), value_type=bool
                    ),
                    "Mem/IncrementalMemory": "true",
                    "Mem/InitWMWithAllNodes": "false",
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
        Node(
            package="my_robot_bringup",
            executable="slam_telemetry.py",
            name="slam_telemetry",
            output="screen",
            # Distance comes from visual translation. The EKF still provides
            # the robot TF, but its 60 Hz prediction jitter is not distance.
            parameters=[{"odom_topic": "/visual_odom"}],
        ),
        Node(
            package="my_robot_bringup",
            executable="slam_health_monitor.py",
            name="slam_health_monitor",
            output="screen",
            parameters=[{"expect_imu": True, "expect_ekf": True}],
        ),
        GroupAction(
            condition=IfCondition(LaunchConfiguration("rviz")),
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="online_slam_rviz",
                    output="screen",
                    condition=UnlessCondition(LaunchConfiguration("topdown")),
                    arguments=["-d", rviz_config],
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="online_slam_topdown_rviz",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("topdown")),
                    arguments=["-d", topdown_rviz_config],
                ),
            ],
        ),
    ])

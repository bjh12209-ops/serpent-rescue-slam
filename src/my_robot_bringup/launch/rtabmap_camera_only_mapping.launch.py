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
    sensor_launch = package_share / "launch" / "sensor_bringup.launch.py"
    odom_params = str(
        package_share / "config" / "rgbd_odometry_camera_only.yaml"
    )
    rtabmap_params = str(package_share / "config" / "rtabmap.yaml")
    rviz_config = str(package_share / "config" / "online_slam.rviz")
    topdown_rviz_config = str(
        package_share / "config" / "online_slam_topdown.rviz"
    )

    rgbd_remappings = [
        ("rgb/image", "/camera/camera/color/image_raw"),
        ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
        ("rgb/camera_info", "/camera/camera/color/camera_info"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/online_slam_camera_only.db",
            description="Camera-only RTAB-Map database to create or continue.",
        ),
        DeclareLaunchArgument(
            "reset_database",
            default_value="false",
            description="Delete database_path before starting a fresh map.",
        ),
        DeclareLaunchArgument(
            "rtabmap_viz",
            default_value="false",
            description="Run the heavier RTAB-Map database/debug GUI.",
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
            PythonLaunchDescriptionSource(str(sensor_launch)),
            launch_arguments={"use_imu": "false"}.items(),
        ),
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[odom_params],
            remappings=[*rgbd_remappings, ("odom", "/visual_odom")],
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
                    # The Intel N97 needs about 0.24-0.32 s for an undecimated
                    # 3 cm map update. Three updates/s prevents queue growth
                    # while RGB-D odometry continues at the camera rate.
                    "Rtabmap/DetectionRate": "3.0",
                    "Optimizer/GravitySigma": "0.0",
                },
            ],
            remappings=rgbd_remappings,
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[rtabmap_params],
            remappings=[*rgbd_remappings, ("odom", "/visual_odom")],
        ),
        Node(
            package="my_robot_bringup",
            executable="slam_telemetry.py",
            name="slam_telemetry",
            output="screen",
            parameters=[{"odom_topic": "/visual_odom"}],
        ),
        Node(
            package="my_robot_bringup",
            executable="slam_health_monitor.py",
            name="slam_health_monitor",
            output="screen",
            parameters=[{"expect_imu": False, "expect_ekf": False}],
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

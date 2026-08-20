from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(
        get_package_share_directory("wt901c485_driver")
    ) / "config" / "wt901c485.yaml"

    return LaunchDescription([
        Node(
            package="wt901c485_driver",
            executable="wt901c485_node",
            name="wt901c485_node",
            output="screen",
            parameters=[str(config)],
        )
    ])

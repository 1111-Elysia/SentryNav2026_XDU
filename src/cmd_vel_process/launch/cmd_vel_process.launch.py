from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("cmd_vel_process")
    params_file = os.path.join(pkg_share, "config", "slope_process.yaml")

    return LaunchDescription([
        Node(
            package="cmd_vel_process",
            executable="slope_process",
            name="slope_process",
            output="screen",
            parameters=[params_file],
        ),
    ])

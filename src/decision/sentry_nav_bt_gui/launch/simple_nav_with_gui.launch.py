import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bt_launch = os.path.join(
        get_package_share_directory("sentry_nav_bt"),
        "launch",
        "simple_nav.launch.py",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bt_launch)),
        Node(
            package="rqt_gui",
            executable="rqt_gui",
            arguments=["--force-discover", "--standalone", "sentry_nav_bt_gui/SimpleNavControl"],
            output="screen",
        ),
    ])

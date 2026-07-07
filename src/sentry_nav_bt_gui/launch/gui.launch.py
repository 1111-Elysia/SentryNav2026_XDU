from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rqt_gui",
            executable="rqt_gui",
            arguments=["--standalone", "sentry_nav_bt_gui/SimpleNavControl"],
            output="screen",
        )
    ])

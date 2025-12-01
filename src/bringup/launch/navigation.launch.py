from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_nav = FindPackageShare('sentry_navigation').find('sentry_navigation')
    map_file = os.path.join(FindPackageShare('bringup').find('bringup'), 'map', 'map.yaml')

    nav = ExecuteProcess(
        cmd=[
            "ros2", "launch", "sentry_navigation", "navigation_launch.py",
            f"map:={map_file}",
            "use_rviz:=true"
        ],
        output="screen"
    )

    return LaunchDescription([nav])

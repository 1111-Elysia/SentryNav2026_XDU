import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("sentry_nav_bt_test")
    tree_file = os.path.join(package_dir, "config", "bt", "simple_nav.xml")
    subtree_dir = os.path.join(package_dir, "config", "bt", "modules")
    waypoints_file = os.path.join(package_dir, "config", "waypoints.json")

    validate_arg = DeclareLaunchArgument("validate_bt_only", default_value="false")

    node = Node(
        package="sentry_nav_bt_test",
        executable="navigate_bt_node",
        name="sentry_nav_bt_test",
        output="screen",
        parameters=[{
            "bt_xml_filename": tree_file,
            "bt_main_tree_id": "MainTree",
            "bt_subtree_dir": subtree_dir,
            "waypoints_file": waypoints_file,
            "validate_bt_only": LaunchConfiguration("validate_bt_only"),
            "use_sim_time": False,
        }],
    )

    return LaunchDescription([validate_arg, node])

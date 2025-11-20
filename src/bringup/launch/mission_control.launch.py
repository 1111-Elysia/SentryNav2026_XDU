from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('bringup')
    config_file = os.path.join(pkg_dir, 'config', 'target_pose.yaml')
    
    return LaunchDescription([
        Node(
            package='bringup',
            executable='mission_control_node',
            name='mission_control_node',
            output='screen',
            parameters=[config_file]
        )
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    config_file = os.path.join(pkg_share, 'config', 'serial_params.yaml')
    topics_file = os.path.join(pkg_share, 'config', 'topic_names.yaml')  

    return LaunchDescription([
        Node(
            package='can_comm',
            executable='can_comm_node',
            name='can_comm_node',
            output='screen',
            parameters=[config_file, topics_file]  
        )
    ])

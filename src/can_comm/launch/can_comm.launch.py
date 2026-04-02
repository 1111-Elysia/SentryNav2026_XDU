from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    config_file = os.path.join(pkg_share, 'config', 'can_params.yaml')
    topics_file = os.path.join(pkg_share, 'config', 'can_topic_names.yaml')  

    can_comm_share = get_package_share_directory('can_comm')
    vyaw_config_file = os.path.join(can_comm_share, 'config', 'vyaw_tf_yaw_controller.yaml')

    return LaunchDescription([
        Node(
            package='can_comm',
            executable='can_comm_node',
            name='can_comm_node',
            output='screen',
            parameters=[config_file, topics_file]  
        ),
        Node(
            package='can_comm',
            executable='vyaw_tf_yaw_controller_node',
            name='vyaw_tf_yaw_controller',
            output='screen',
            parameters=[vyaw_config_file]
        )
    ])

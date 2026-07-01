import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ground_pos_relay')
    default_tf_config = os.path.join(pkg_share, 'config', 'field_to_odom.yaml')

    tf_config = LaunchConfiguration('tf_config')
    field_frame = LaunchConfiguration('field_frame')
    robot_frame = LaunchConfiguration('robot_frame')
    use_tf_self_position = LaunchConfiguration('use_tf_self_position')

    return LaunchDescription([
        DeclareLaunchArgument('tf_config', default_value=default_tf_config),
        DeclareLaunchArgument('field_frame', default_value='rm_field'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('use_tf_self_position', default_value='true'),

        Node(
            package='ground_pos_relay',
            executable='field_to_odom_tf_node',
            name='field_to_odom_tf_node',
            output='screen',
            parameters=[tf_config],
        ),

        Node(
            package='ground_pos_relay',
            executable='ground_pos_relay_node',
            name='ground_pos_relay_node',
            output='screen',
            parameters=[{
                'use_tf_self_position': use_tf_self_position,
                'field_frame': field_frame,
                'robot_frame': robot_frame,
            }],
        ),
    ])

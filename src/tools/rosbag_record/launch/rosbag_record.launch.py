from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    default_params_file = (
        get_package_share_directory('rosbag_record') + '/config/rosbag_record.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Path to rosbag_record parameters YAML file',
            ),
            Node(
                package='rosbag_record',
                executable='rosbag_record_node.py',
                name='rosbag_record',
                output='screen',
                # Always load the package default params, then optionally overlay a user-provided file.
                # This prevents a missing/invalid override path from resulting in empty parameters.
                parameters=[default_params_file, params_file],
            ),
        ]
    )

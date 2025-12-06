from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('pub_point_bt')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='pub_point_bt',
            executable='pub_point_bt_node',
            name='pub_point_bt_node',
            output='screen',
            parameters=[params_file, {
                "use_sim_time": False,    
                "bt_xml": os.path.join(pkg_share, 'config', 'waypoints_bt.xml'),
            }],
        )
    ])

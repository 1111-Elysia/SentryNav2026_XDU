import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- Paths ---
    # Path to the map server config file (contains path to the actual map yaml)
    params_file = '/media/gc/Software/workplace/Sentry/Test_workplace/kiss-icp/src/pcd_to_nav_map/config/test_map.yaml'

    # --- Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # --- Nodes ---
    # Lifecycle Manager to manage map_server AND global_costmap
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap', # Give a descriptive name
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': ['map_server',
                                    ]}])

    # Map Server Node (loads the static map)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server', # Name must match one in lifecycle_manager's node_names list
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}])

    # Global Costmap Node
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap', # Name must match one in lifecycle_manager's node_names list
        namespace='global',  # 命名空间
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}])
        # Remappings can be added here if needed, e.g., to change output topic names

    # Static TF Publisher
    # Publishes a static transform from 'map' frame to 'base_link' frame.
    # Required by the costmap node. We place base_link at the map origin (0,0,0).
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_base',
        output='screen',
        # Arguments: x y z yaw pitch roll frame_id child_frame_id
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'])


    return LaunchDescription([
        declare_use_sim_time_cmd,
        lifecycle_manager_node,
        map_server_node,
        global_costmap_node,    # Add the global costmap node
        static_tf_pub_node,     # Add the static transform publisher
    ])
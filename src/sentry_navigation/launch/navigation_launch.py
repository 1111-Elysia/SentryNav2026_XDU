import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    LogInfo
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # ---------------------------
    # Package paths
    # ---------------------------
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('bringup')
    
    # ---------------------------
    # Launch Configuration Variables (Fix for NameError)
    # 必须显式定义这些变量，才能在后面的 Node 中使用
    # ---------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # ---------------------------
    # Static (non-substitution) defaults
    # ---------------------------
    default_map_yaml = os.path.join(bringup_dir, 'map', 'map.yaml')
    default_params = os.path.join(bringup_dir, 'config', 'singlenav2_params.yaml')
    default_lidar_params = os.path.join(bringup_dir, 'config', 'lidar.yaml')
    default_rviz_config = os.path.join(bringup_dir, 'config', 'nav2_default_view.rviz')

    # ---------------------------
    # Declare launch arguments
    # ---------------------------
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz if true',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to map yaml file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to nav2 params file',
        ),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=default_lidar_params,
            description='Full path to lidar / TF params file',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to RViz config file',
        ),
    ]

    # ...existing code...
    # (保留之前的 map_server_node, map_lifecycle_manager, tf_odom_node, nav2_navigation 定义)
    
    # ---------------------------
    # Map server (NO lifecycle here)
    # ---------------------------
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
        ],
    )

    # ---------------------------
    # Lifecycle manager for map_server
    # ---------------------------
    map_lifecycle_manager = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']},
                ],
            )
        ],
    )
    # ---------------------------
    # Nav2 navigation stack
    # ---------------------------
    nav2_navigation = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            )
        ],
    )

    # ---------------------------
    # RViz
    # ---------------------------
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ---------------------------
    # TF Monitor Node
    # ---------------------------
    tf_monitor_node = Node(
        package='sentry_navigation',
        executable='tf_monitor.py', 
        name='tf_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------------------------
    # Event Handler: Wait for TF Monitor to finish
    # ---------------------------
    wait_for_tf_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=tf_monitor_node,
            on_exit=[
                LogInfo(msg="TF Chain Ready (map->odom->base_link->livox_frame). Starting Navigation Stack..."),
                map_server_node,
                map_lifecycle_manager,
                nav2_navigation
            ]
        )
    )

    # ---------------------------
    # LaunchDescription
    # ---------------------------
    return LaunchDescription(
        declare_args
        + [
            rviz_node,
            tf_monitor_node,
            wait_for_tf_handler
        ]
    )
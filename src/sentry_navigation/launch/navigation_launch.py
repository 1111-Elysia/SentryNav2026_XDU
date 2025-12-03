import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 获取包路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # sentry_nav_dir = get_package_share_directory('sentry_navigation')
    bringup_dir = get_package_share_directory('bringup')
    
    # 参数配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    map_yaml_file = LaunchConfiguration('map', 
        default=os.path.join('./data/new_map', 'map.yaml'))
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(bringup_dir, 'config', 'navigation_params.yaml'))
    lidar_params_file = LaunchConfiguration('lidar_params_file',
        default=os.path.join(bringup_dir, 'config', 'lidar.yaml'))
    rviz_config_file = LaunchConfiguration('rviz_config',
        default=os.path.join(bringup_dir, 'config', 'nav2_default_view.rviz'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz if true'),
        
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map yaml file'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to param file'),
        
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=lidar_params_file,
            description='Full path to lidar/TF params file'),
        
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='Full path to RViz config file'),

        # 启动地图服务器
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                       {'yaml_filename': map_yaml_file}]),

        # 启动生命周期管理器
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                       {'autostart': True},
                       {'node_names': ['map_server']}]),

        # 启动TF和Odom发布节点（加载 lidar.yaml 配置）
        Node(
            package='sentry_navigation',
            executable='tf_odom_publisher',
            name='tf_odom_publisher',
            output='screen',
            parameters=[lidar_params_file]),

        # 延迟2秒启动Nav2（等待TF树建立）
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': params_file
                    }.items()),
            ]),
        
        # 启动RViz（可选）
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
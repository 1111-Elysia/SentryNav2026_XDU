from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    
    # 1. 启动原始系统
    start_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'start.launch.py')
        )
    )
    
    # 2. 延迟启动监控器（等待更长的时间）
    monitor_timer = TimerAction(
        period=30.0,  # 增加到30秒，确保所有节点完全启动
        actions=[
            # 栅格地图监控节点
            Node(
                package='bringup',
                executable='map_monitor.py',
                name='map_monitor',
                output='screen',
                parameters=[{
                    'map_topic': '/map',
                    'timeout_seconds': 3.0,  
                    'check_interval': 1.0,     # 每1秒检查一次
                    'max_empty_counts': 2  # 连续2次空数据触发
                }]
            ),
            # 代价地图监控节点
            Node(
                package='bringup',
                executable='costmap_monitor.py',
                name='costmap_monitor',
                output='screen',
                parameters=[{
                    'global_costmap_topic': '/global_costmap/costmap',
                    'local_costmap_topic': '/local_costmap/costmap',
                    'timeout_seconds': 3.0,  
                    'check_interval': 1.0,     # 每1秒检查一次
                    'max_consecutive_timeouts': 2  # 连续2次超时触发
                }]
            ),
            # scan监控节点
            Node(
                package='bringup',
                executable='scan_monitor.py',
                name='scan_monitor',
                output='screen',
                parameters=[{
                    'scan_topic': '/scan',
                    'timeout_seconds': 3.0,  
                    'check_interval': 1.0,     # 每1秒检查一次
                    'max_consecutive_timeouts': 2  # 连续2次超时触发
                }],
            ),
            # 重启管理器
            Node(
                package='bringup',
                executable='restart_manager.py',
                name='restart_manager',
                output='screen',
                parameters=[{
                    'restart_delay': 5.0,
                    'max_restarts_per_hour': 5,
                    'restart_in_terminal': True  # 新参数，控制重启方式
                }]
            )
        ]
    )
    
    return LaunchDescription([
        start_launch,
        monitor_timer
    ])
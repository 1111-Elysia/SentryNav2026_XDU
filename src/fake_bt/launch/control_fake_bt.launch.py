from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="启动青春版行为树管理器 fake_bt_manager..."),
        Node(
            package='fake_bt',
            executable='fake_bt_manager.py',
            output='screen',
            emulate_tty=True
        )
    ])

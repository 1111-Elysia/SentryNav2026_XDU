from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    manager_script = PathJoinSubstitution(
        [FindPackageShare('fake_bt'), 'scripts', 'fake_bt_manager.py']
    )

    return LaunchDescription([
        LogInfo(msg=['Starting fake_bt manager (will spawn/stop pub_point and pub_vw based on match_stage)...']),
        ExecuteProcess(
            cmd=['python3', manager_script],
            output='screen',
            shell=False
        ),
    ])

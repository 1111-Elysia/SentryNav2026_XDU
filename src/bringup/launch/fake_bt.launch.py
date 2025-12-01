# fake_bt.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    bt = ExecuteProcess(
        cmd=["ros2", "launch", "fake_bt", "control_fake_bt.launch.py"],
        output="screen"
    )
    return LaunchDescription([bt])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rl_robot',
            executable='train_dqn.py',
            output='screen'
        )
    ])
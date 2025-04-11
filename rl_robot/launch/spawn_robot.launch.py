from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'rl_robot', '-file', os.path.join(
                get_package_share_directory('rl_robot'),
                'urdf/robot.urdf.xacro'
            )],
            output='screen'
        )
    ])
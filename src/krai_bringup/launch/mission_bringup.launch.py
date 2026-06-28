from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='krai_mission',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
        ),
    ])

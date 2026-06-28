from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='krai_gui',
            executable='krai_gui_node',
            name='krai_gui_node',
            output='screen',
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),

        Node(
            package='krai_s3_bridge',
            executable='s3_bridge_node',
            name='s3_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': baud_rate,
            }],
        ),

        Node(
            package='krai_mission',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
        ),

        Node(
            package='krai_gui',
            executable='krai_gui_node',
            name='krai_gui_node',
            output='screen',
        ),
    ])

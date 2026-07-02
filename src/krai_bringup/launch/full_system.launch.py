from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    camera_source = LaunchConfiguration('camera_source')
    model_path = LaunchConfiguration('model_path')
    enable_rak_grid = LaunchConfiguration('enable_rak_grid')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('camera_source', default_value='0',
                              description='Camera device index (0, 1, 2, ...)'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('enable_rak_grid', default_value='false'),

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

        Node(
            package='krai_camera',
            executable='krai_camera_node',
            name='krai_camera_node',
            output='screen',
            parameters=[{
                'camera_source': camera_source,
                'model_path': model_path,
                'enable_rak_grid': enable_rak_grid,
            }],
        ),
    ])

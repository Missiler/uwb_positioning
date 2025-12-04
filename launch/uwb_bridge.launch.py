from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('scale_m', default_value='1.0'),
        DeclareLaunchArgument('frame_id', default_value='map'),

        Node(
            package='uwb_positioning',
            executable='uwb_read_only_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'scale_m': LaunchConfiguration('scale_m'),
                'frame_id': LaunchConfiguration('frame_id'),
                'read_crlf': True,
            }]
        )
    ])

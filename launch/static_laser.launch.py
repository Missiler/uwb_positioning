from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('parent_frame', default_value='map'),
        DeclareLaunchArgument('child_frame', default_value='laser'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_laser_tf',
            output='screen',
            arguments=[
                LaunchConfiguration('x'),
                LaunchConfiguration('y'),
                LaunchConfiguration('z'),
                LaunchConfiguration('roll'),
                LaunchConfiguration('pitch'),
                LaunchConfiguration('yaw'),
                LaunchConfiguration('parent_frame'),
                LaunchConfiguration('child_frame'),
            ]
        )
    ])

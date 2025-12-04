#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----- Launch arguments -----
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )
    )

    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baud',
        default_value='115200'
    )

    uwb_port_arg = DeclareLaunchArgument(
        'uwb_port',
        default_value='/dev/serial/by-id/usb-SEGGER_J-Link_000760192731-if00'
    )

    uwb_baud_arg = DeclareLaunchArgument(
        'uwb_baud',
        default_value='115200'
    )


    # ----- Lidar driver (your Terminal A) -----
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baud'),
            'frame_id': 'laser',
            'scan_mode': 'Standard',
            'publish_tf': 'false'
        }.items()
    )

    # ----- UWB bridge (your Terminal B) -----
    uwb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uwb_ro_bridge'),
                'launch',
                'uwb_ro_bridge.launch.py'
            ])
        ),
        launch_arguments={
            'port': LaunchConfiguration('uwb_port'),
            'baud': LaunchConfiguration('uwb_baud'),
            'frame_id': 'odom',   # UWB publishes in odom frame
            'rviz': 'false',
        }.items()
    )

    # ----- UWB -> TF broadcaster (your Terminal C) -----
    uwb_odom_broadcaster_node = Node(
        package='uwb_ro_bridge',
        executable='uwb_odom_broadcaster',
        name='uwb_odom_broadcaster',
        output='screen'
    )

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        uwb_port_arg,
        uwb_baud_arg,
        lidar_launch,
        uwb_launch,
        uwb_odom_broadcaster_node,
    ])


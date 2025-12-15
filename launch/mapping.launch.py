#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

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
    publish_static_laser_tf_arg = DeclareLaunchArgument(
        'publish_static_laser_tf',
        default_value='true'
    )
    static_laser_x_arg = DeclareLaunchArgument('static_laser_x', default_value='0.0')
    static_laser_y_arg = DeclareLaunchArgument('static_laser_y', default_value='0.0')
    static_laser_z_arg = DeclareLaunchArgument('static_laser_z', default_value='0.0')
    static_laser_roll_arg = DeclareLaunchArgument('static_laser_roll', default_value='0.0')
    static_laser_pitch_arg = DeclareLaunchArgument('static_laser_pitch', default_value='0.0')
    static_laser_yaw_arg = DeclareLaunchArgument('static_laser_yaw', default_value='0.0')
    laser_frame_arg = DeclareLaunchArgument('laser_frame', default_value='laser')

    # ----- Lidar driver (your Terminal A) -----


    # ----- UWB bridge (your Terminal B) -----
    uwb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uwb_positioning'),
                'launch',
                'uwb_bridge.launch.py'
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
    uwb_publisher_node = Node(
        package='uwb_positioning',
        executable='uwb_publisher',
        name='uwb_publisher',
        output='screen'
    )
    
    imu_publisher_node = Node(
        package='uwb_positioning',
        executable='imu_publish',
        name='imu_publisher',
        output='screen'
    )
    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        output='screen',
        arguments=[
            LaunchConfiguration('static_laser_x'),
            LaunchConfiguration('static_laser_y'),
            LaunchConfiguration('static_laser_z'),
            LaunchConfiguration('static_laser_roll'),
            LaunchConfiguration('static_laser_pitch'),
            LaunchConfiguration('static_laser_yaw'),
            LaunchConfiguration('parent_frame') if 'parent_frame' in globals() else LaunchConfiguration('map'),
            LaunchConfiguration('laser_frame')
        ],
        condition=IfCondition(LaunchConfiguration('publish_static_laser_tf'))
    )
    
    

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        uwb_port_arg,
        uwb_baud_arg,
        uwb_launch,
        uwb_publisher_node,
        imu_publisher_node,
        static_laser_tf,
    ])

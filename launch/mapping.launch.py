#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
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
    
    
    namespace = LaunchConfiguration('namespace')

    # Declares an action to allow users to pass the robot namespace from the
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace')


    # ----- Lidar driver (your Terminal A) -----
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[
            get_package_share_directory("uwb_positioning") + '/config/rplidar_node.yaml'
            ],
        namespace=namespace
    )


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
    
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.012', '0', '0.144', '0', '0', '0', 'base_footprint', 'laser'],

        # Remaps topics used by the 'tf2_ros' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'tf2_ros'.
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace
    )
    
    

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        uwb_port_arg,
        uwb_baud_arg,
        rplidar_node,
        uwb_launch,
        uwb_publisher_node,
        imu_publisher_node,
    ])

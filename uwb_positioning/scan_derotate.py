#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

import tf2_ros
from tf2_ros import TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class ScanDerotate(Node):
    def __init__(self):
        super().__init__('scan_derotate')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('cloud_topic', '/scan_map')

        self.scan_topic = self.get_parameter('scan_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.cloud_topic = self.get_parameter('cloud_topic').value

        # Laser projection
        self.projector = LaserProjection()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS I/O
        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            self.cloud_topic,
            10
        )

        self.get_logger().info(
            f'De-rotating {self.scan_topic} → {self.cloud_topic} in frame "{self.target_frame}"'
        )

    def scan_cb(self, scan: LaserScan):
        try:
            # 1) Project LaserScan → PointCloud2 (laser frame)
            cloud_laser = self.projector.projectLaser(scan)

            # 2) Lookup transform laser → target_frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                rclpy.time.Time()
            )

            # 3) Transform cloud → target_frame
            cloud_map = do_transform_cloud(cloud_laser, transform)

            # 4) Fix header
            cloud_map.header.stamp = scan.header.stamp
            cloud_map.header.frame_id = self.target_frame

            self.pub.publish(cloud_map)

        except TransformException as ex:
            self.get_logger().warn(f'TF error: {ex}')


def main():
    rclpy.init()
    node = ScanDerotate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

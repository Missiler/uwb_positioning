#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

import tf2_ros
import tf2_py as tf2
from tf2_ros import TransformException


class ScanToMapCloud(Node):
    def __init__(self):
        super().__init__('scan_to_map_cloud')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('cloud_topic', '/scan_map')

        scan_topic = self.get_parameter('scan_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        cloud_topic = self.get_parameter('cloud_topic').value

        self.projector = LaserProjection()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_cb, 10
        )

        self.pub = self.create_publisher(
            PointCloud2, cloud_topic, 10
        )

        self.get_logger().info(
            f'Transforming {scan_topic} → {cloud_topic} in frame {self.target_frame}'
        )

    def scan_cb(self, scan: LaserScan):
        try:
            # Transform laser scan into map frame
            cloud = self.projector.transformLaserScanToPointCloud(
                self.target_frame,
                scan,
                self.tf_buffer
            )
            self.pub.publish(cloud)

        except TransformException as ex:
            self.get_logger().warn(f'TF error: {ex}')


def main():
    rclpy.init()
    node = ScanToMapCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

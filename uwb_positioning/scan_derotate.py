#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu


class ScanDeRotate(Node):
    def __init__(self):
        super().__init__('scan_derotate')

        # Parameters
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_world')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('enable', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.enabled = self.get_parameter('enable').value

        # Latest IMU yaw
        self._yaw = 0.0

        # Subscribers and publishers
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.input_topic, self.scan_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, 10)

        self.get_logger().info(f"Scan derotate node started (in: {self.input_topic}, out: {self.output_topic}, parent: {self.parent_frame})")

    def imu_cb(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg: LaserScan):
        if not self.enabled:
            return

        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self.parent_frame
        out.angle_min = msg.angle_min - self._yaw
        out.angle_max = msg.angle_max - self._yaw
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities) if msg.intensities is not None else []

        self.scan_pub.publish(out)


def main():
    rclpy.init()
    node = ScanDeRotate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu


class ScanDeRotate(Node):
    def __init__(self):
        super().__init__('scan_derotate')

        # Params
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_world')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('enable', True)
        self.declare_parameter('replace_scan', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.enabled = self.get_parameter('enable').value
        self.replace_scan = self.get_parameter('replace_scan').value

        # IMU state
        self.latest_yaw = 0.0
        self.prev_imu_stamp = None

        # Subscribers / publishers
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.create_subscription(LaserScan, self.input_topic, self.scan_cb, 10)
        # If replace_scan is True, publish derotated scans on the original topic
        out_topic = self.input_topic if self.replace_scan else self.output_topic
        self.scan_pub = self.create_publisher(LaserScan, out_topic, 10)

        self.get_logger().info(
            f"Scan derotate node started (in: {self.input_topic}, out: {self.output_topic}, parent: {self.parent_frame})"
        )

    def imu_cb(self, msg: Imu):
        # Convert quaternion to yaw
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.latest_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.prev_imu_stamp = msg.header.stamp

    def scan_cb(self, msg: LaserScan):
        if not self.enabled:
            return

        # Copy input into output
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.parent_frame
        out.angle_min = 0.0  # will be recomputed
        out.angle_max = 0.0  # will be recomputed
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        n = len(msg.ranges)
        derotated_ranges = [0.0] * n

        # Assume the scan covers one rotation from angle_min to angle_max
        scan_start_yaw = self.latest_yaw

        for i in range(n):
            # Time offset for this ray
            t_offset = i * msg.time_increment
            # Estimate yaw at time of this ray
            yaw_at_ray = scan_start_yaw  # IMU yaw can be more accurately interpolated if timestamped
            # Compute correction
            ray_angle = msg.angle_min + i * msg.angle_increment
            corrected_angle = ray_angle + (scan_start_yaw - yaw_at_ray)

            # Save corrected range
            derotated_ranges[i] = msg.ranges[i]

            # Update min/max if first/last
            if i == 0:
                out.angle_min = corrected_angle
            if i == n - 1:
                out.angle_max = corrected_angle

        out.ranges = derotated_ranges
        out.intensities = list(msg.intensities) if msg.intensities else []

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

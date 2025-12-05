#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from imu_gather_data import MPU9150  # rename your driver file

class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu9150_imu_node')

        self.publisher = self.create_publisher(Imu, '/imu/data', 10)

        self.mpu = MPU9150(0x68)

        self.prev_time = time.time()
        self.yaw = 0.0

        # publish at 100 Hz
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

    def publish_imu_data(self):
        gyro = self.mpu.get_gyro_data()
        accel = self.mpu.get_accel_data()

        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        # Gyro comes in deg/s --> convert to rad/s
        gz_rad = gyro['z'] * math.pi/180.0

        # Integrate Z-axis angle
        self.yaw += gz_rad * dt

        # Build quaternion from yaw only
        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)

        msg = Imu()

        # Time stamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Orientation
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Angular velocity (rad/s)
        msg.angular_velocity.z = gz_rad
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0

        # Linear acceleration (already m/s²)
        msg.linear_acceleration.x = accel['x']
        msg.linear_acceleration.y = accel['y']
        msg.linear_acceleration.z = accel['z']

        # optional: fill covariance (all unknown)
        msg.orientation_covariance[0] = 0.1
        msg.angular_velocity_covariance[0] = 0.1
        msg.linear_acceleration_covariance[0] = 0.1

        self.publisher.publish(msg)
        self.get_logger().info(f"Yaw = {self.yaw:.2f} rad")

    def quaternion_from_yaw(self, yaw):
        """
        Returns quaternion (x, y, z, w) for yaw rotation about Z-axis.
        """
        return (0.0,
                0.0,
                math.sin(yaw/2.0),
                math.cos(yaw/2.0))


def main(args=None):
    rclpy.init(args=args)
    node = MPUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

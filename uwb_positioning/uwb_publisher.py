#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import tf2_ros


def quat_inverse(q):
    """Inverse of a unit quaternion."""
    x, y, z, w = q
    n = (x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (-x / n, -y / n, -z / n,  w / n)


class UwbOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('uwb_odom_broadcaster')

        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Publishing controls
        self.declare_parameter('publish_tf', False)  # IMPORTANT: avoid TF fights with UKF if UKF publish_tf:=true
        self.publish_tf = self.get_parameter('publish_tf').value

        # Laser behavior
        # If True, publish base_link->laser rotation as inverse IMU, so laser stays world-aligned (virtual frame).
        self.declare_parameter('static_laser_in_world', False)
        self.static_laser_in_world = self.get_parameter('static_laser_in_world').value

        # Laser mounting offset relative to base_link (set to real values if needed)
        self.declare_parameter('laser_xyz', [0.0, 0.0, 0.0])
        self.laser_xyz = self.get_parameter('laser_xyz').value

        # Latest sensor data
        self.latest_pose = None  # PoseStamped from UWB
        self.latest_imu = None   # Imu message

        # Velocity estimation
        self._prev_pos = None
        self._prev_time = None

        # Subscriptions
        self.create_subscription(PoseStamped, '/uwb/pose', self.pose_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publishers / TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_timer(0.02, self.publish)  # 50 Hz
        self.get_logger().info(
            f'UWB->Odometry node started. publish_tf={self.publish_tf}, '
            f'static_laser_in_world={self.static_laser_in_world}'
        )

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def publish(self):
        if self.latest_pose is None:
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()

        # Always publish odometry in odom_frame -> base_link
        parent_frame = self.odom_frame

        px = self.latest_pose.pose.position.x
        py = self.latest_pose.pose.position.y
        pz = self.latest_pose.pose.position.z

        # Orientation (IMU preferred)
        if self.latest_imu is not None:
            q = self.latest_imu.orientation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        # --- TF: odom -> base_link (ONLY if enabled) ---
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = parent_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = pz
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

            # --- TF: base_link -> laser (keeps TF tree valid) ---
            tl = TransformStamped()
            tl.header.stamp = now_msg
            tl.header.frame_id = self.base_frame
            tl.child_frame_id = "laser"

            tl.transform.translation.x = float(self.laser_xyz[0])
            tl.transform.translation.y = float(self.laser_xyz[1])
            tl.transform.translation.z = float(self.laser_xyz[2])

            if self.static_laser_in_world and self.latest_imu is not None:
                # Counter-rotate the laser relative to base_link
                ix, iy, iz, iw = quat_inverse((qx, qy, qz, qw))
                tl.transform.rotation.x = ix
                tl.transform.rotation.y = iy
                tl.transform.rotation.z = iz
                tl.transform.rotation.w = iw
            else:
                # Normal mounted sensor: fixed rotation
                tl.transform.rotation.x = 0.0
                tl.transform.rotation.y = 0.0
                tl.transform.rotation.z = 0.0
                tl.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(tl)

        # --- Publish nav_msgs/Odometry ---
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = parent_frame          # odom
        odom.child_frame_id = self.base_frame        # base_link

        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Finite-difference velocity
        vx = vy = vz = 0.0
        t_now = now.nanoseconds * 1e-9
        if self._prev_pos is not None and self._prev_time is not None:
            dt = t_now - self._prev_time
            if dt > 1e-6:
                vx = (px - self._prev_pos[0]) / dt
                vy = (py - self._prev_pos[1]) / dt
                vz = (pz - self._prev_pos[2]) / dt

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz

        # Save prev
        self._prev_pos = (px, py, pz)
        self._prev_time = t_now

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = UwbOdomBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros


class UwbOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('uwb_odom_broadcaster')

        # ---------------- Parameters ----------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        self.declare_parameter('publish_laser_tf', True)
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_z', 0.0)
        self.declare_parameter('laser_yaw', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.publish_laser_tf = self.get_parameter('publish_laser_tf').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)
        self.laser_z = float(self.get_parameter('laser_z').value)
        self.laser_yaw = float(self.get_parameter('laser_yaw').value)

        # ---------------- State ----------------
        self.latest_pose = None   # PoseStamped from UWB (world position)
        self.latest_imu = None    # Imu (orientation)

        self.prev_pos = None
        self.prev_time = None

        # ---------------- ROS I/O ----------------
        self.create_subscription(PoseStamped, '/uwb/pose', self.pose_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.timer = self.create_timer(0.02, self.publish)  # 50 Hz

        self.get_logger().info(
            f'UWB broadcaster started. map_frame="{self.map_frame}", base_frame="{self.base_frame}"'
        )

    # ---------------- Callbacks ----------------
    def pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

    def imu_cb(self, msg: Imu):
        self.latest_imu = msg

    # ---------------- Main Loop ----------------
    def publish(self):
        if self.latest_pose is None:
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()
        t_now = now.nanoseconds * 1e-9

        px = self.latest_pose.pose.position.x
        py = self.latest_pose.pose.position.y
        pz = self.latest_pose.pose.position.z

        # ==========================================================
        # TF 1: map -> base_link
        # ==========================================================
        t_base = TransformStamped()
        t_base.header.stamp = now_msg
        t_base.header.frame_id = self.map_frame
        t_base.child_frame_id = self.base_frame

        t_base.transform.translation.x = px
        t_base.transform.translation.y = py
        t_base.transform.translation.z = pz

        if self.latest_imu is not None:
            t_base.transform.rotation = self.latest_imu.orientation
        else:
            t_base.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t_base)

        # ==========================================================
        # TF 2: base_link -> imu_link (STATIC)
        # ==========================================================
        t_imu = TransformStamped()
        t_imu.header.stamp = now_msg
        t_imu.header.frame_id = self.base_frame
        t_imu.child_frame_id = self.imu_frame
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.0
        t_imu.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t_imu)

        # ==========================================================
        # TF 3: base_link -> laser (STATIC)
        # ==========================================================
        if self.publish_laser_tf:
            t_laser = TransformStamped()
            t_laser.header.stamp = now_msg
            t_laser.header.frame_id = self.base_frame
            t_laser.child_frame_id = self.laser_frame

            t_laser.transform.translation.x = self.laser_x
            t_laser.transform.translation.y = self.laser_y
            t_laser.transform.translation.z = self.laser_z

            sy = math.sin(self.laser_yaw * 0.5)
            cy = math.cos(self.laser_yaw * 0.5)
            t_laser.transform.rotation.z = sy
            t_laser.transform.rotation.w = cy

            self.tf_broadcaster.sendTransform(t_laser)

        # ==========================================================
        # Odometry message (map -> base_link)
        # ==========================================================
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz

        if self.latest_imu is not None:
            odom.pose.pose.orientation = self.latest_imu.orientation
        else:
            odom.pose.pose.orientation.w = 1.0

        vx = vy = vz = 0.0
        if self.prev_pos is not None and self.prev_time is not None:
            dt = t_now - self.prev_time
            if dt > 1e-6:
                vx = (px - self.prev_pos[0]) / dt
                vy = (py - self.prev_pos[1]) / dt
                vz = (pz - self.prev_pos[2]) / dt

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz

        if self.latest_imu is not None:
            odom.twist.twist.angular.z = self.latest_imu.angular_velocity.z

        self.odom_pub.publish(odom)

        self.prev_pos = (px, py, pz)
        self.prev_time = t_now


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

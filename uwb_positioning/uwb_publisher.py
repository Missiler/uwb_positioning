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

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Latest sensor data
        self.latest_pose = None        # PoseStamped from UWB
        self.latest_imu = None         # Imu message

        # For simple velocity estimation
        self._prev_pos = None
        self._prev_time = None

        # Subscribers
        self.create_subscription(PoseStamped, '/uwb/pose', self.pose_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publishers and TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Publish at 50 Hz
        self.timer = self.create_timer(0.02, self.publish)

        self.get_logger().info('UWB->Odometry broadcaster started')

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def publish(self):
        if self.latest_pose is None:
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()

        # Build transform
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Position from UWB pose
        px = self.latest_pose.pose.position.x
        py = self.latest_pose.pose.position.y
        pz = self.latest_pose.pose.position.z
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz

        # Orientation from IMU if available, otherwise identity
        if self.latest_imu is not None:
            t.transform.rotation = self.latest_imu.orientation
        else:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

        # Broadcast TF for base_link (car body with IMU rotation)
        self.tf_broadcaster.sendTransform(t)

        # Build LiDAR transform (mounted on car, but with inverse IMU rotation)
        # This de-rotates the scan to world frame despite car turning
        t_laser = TransformStamped()
        t_laser.header.stamp = now_msg
        t_laser.header.frame_id = self.odom_frame
        t_laser.child_frame_id = "laser"
        t_laser.transform.translation.x = px
        t_laser.transform.translation.y = py
        t_laser.transform.translation.z = pz
        
        if self.latest_imu is not None:
            # Apply inverse (conjugate) of IMU orientation plus 90-degree Z rotation
            imu_quat = self.latest_imu.orientation
            # Conjugate: (x, y, z, w) -> (-x, -y, -z, w)
            imu_inv_x = -imu_quat.x
            imu_inv_y = -imu_quat.y
            imu_inv_z = -imu_quat.z
            imu_inv_w = imu_quat.w
            
            # Compose with 90-degree Z rotation: quat_result = q_inv * q_90z
            # q_90z = (0, 0, sin(pi/4), cos(pi/4)) = (0, 0, ~0.707, ~0.707)
            q90_x, q90_y, q90_z, q90_w = 0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)
            
            # Quaternion multiplication: q1 * q2
            t_laser.transform.rotation.x = imu_inv_w * q90_x + imu_inv_x * q90_w + imu_inv_y * q90_z - imu_inv_z * q90_y
            t_laser.transform.rotation.y = imu_inv_w * q90_y - imu_inv_x * q90_z + imu_inv_y * q90_w + imu_inv_z * q90_x
            t_laser.transform.rotation.z = imu_inv_w * q90_z + imu_inv_x * q90_y - imu_inv_y * q90_x + imu_inv_z * q90_w
            t_laser.transform.rotation.w = imu_inv_w * q90_w - imu_inv_x * q90_x - imu_inv_y * q90_y - imu_inv_z * q90_z
        else:
            # No IMU: just apply 90-degree Z rotation
            t_laser.transform.rotation.x = 0.0
            t_laser.transform.rotation.y = 0.0
            t_laser.transform.rotation.z = math.sin(math.pi / 4.0)
            t_laser.transform.rotation.w = math.cos(math.pi / 4.0)
        
        self.tf_broadcaster.sendTransform(t_laser)

        # Publish nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pose
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        if self.latest_imu is not None:
            odom.pose.pose.orientation = self.latest_imu.orientation
        else:
            odom.pose.pose.orientation.w = 1.0

        # Estimate linear velocity via finite difference
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

        # Simple covariances (small defaults). If IMU provides covariance, copy it.
        try:
            if self.latest_imu is not None:
                # copy orientation covariance into pose.covariance rotational blocks
                ocov = self.latest_imu.orientation_covariance
                # place orientation covariances (xx) at indices 21..26 region (yaw)
                # nav_msgs/Odometry.pose.covariance is 6x6 row-major
                odom.pose.covariance[0] = 0.1
                odom.pose.covariance[7] = 0.1
                odom.pose.covariance[14] = 0.1
                odom.pose.covariance[21] = ocov[0] if len(ocov) > 0 else 0.1
                odom.pose.covariance[28] = ocov[4] if len(ocov) > 4 else 0.1
                odom.pose.covariance[35] = ocov[8] if len(ocov) > 8 else 0.1
        except Exception:
            pass

        self.odom_pub.publish(odom)

        # Save previous for velocity estimation
        self._prev_pos = (px, py, pz)
        self._prev_time = t_now


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

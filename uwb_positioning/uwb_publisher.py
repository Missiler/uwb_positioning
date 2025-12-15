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
        # Laser TF configuration (publish base_link -> laser frame)
        self.declare_parameter('publish_laser_tf', True)
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_z', 0.0)
        self.declare_parameter('laser_yaw', 0.0)
        self.publish_laser_tf = self.get_parameter('publish_laser_tf').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)
        self.laser_z = float(self.get_parameter('laser_z').value)
        self.laser_yaw = float(self.get_parameter('laser_yaw').value)
        # Parent frame for published TFs (use 'map' to keep scans world-fixed)
        self.declare_parameter('parent_frame', 'map')
        self.parent_frame = self.get_parameter('parent_frame').value

        # Latest sensor data
        self.latest_pose = None        # PoseStamped from UWB
        self.latest_imu = None         # Imu message

        # For simple velocity estimation
        self._prev_pos = None
        self._prev_time = None
        self._prev_yaw = None

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

    def cmd_vel_callback(self, msg):
        # Store last cmd_vel; this node doesn't actuate motors but we keep it for integration
        self.latest_cmd = msg

    def publish(self):
        if self.latest_pose is None:
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()

        # Build transform: publish odom->base_link (odom frame should be local odometry)
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

        # Publish a fixed transform from base_link -> laser (robot frame)
        if self.publish_laser_tf:
            t_laser = TransformStamped()
            t_laser.header.stamp = now_msg
            t_laser.header.frame_id = self.base_frame
            t_laser.child_frame_id = self.laser_frame
            # translation is relative to base_link
            t_laser.transform.translation.x = self.laser_x
            t_laser.transform.translation.y = self.laser_y
            t_laser.transform.translation.z = self.laser_z
            # rotation from yaw offset
            sy = math.sin(self.laser_yaw * 0.5)
            cy = math.cos(self.laser_yaw * 0.5)
            t_laser.transform.rotation.x = 0.0
            t_laser.transform.rotation.y = 0.0
            t_laser.transform.rotation.z = sy
            t_laser.transform.rotation.w = cy
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

        # Convert linear velocity to base_link frame using IMU yaw (if available)
        if self.latest_imu is not None:
            q = self.latest_imu.orientation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            vbx = math.cos(yaw) * vx + math.sin(yaw) * vy
            vby = -math.sin(yaw) * vx + math.cos(yaw) * vy
            odom.twist.twist.linear.x = vbx
            odom.twist.twist.linear.y = vby
            odom.twist.twist.linear.z = vz
            # Angular velocity from IMU if present
            try:
                odom.twist.twist.angular.z = self.latest_imu.angular_velocity.z
            except Exception:
                odom.twist.twist.angular.z = 0.0
            self._prev_yaw = yaw
        else:
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.linear.z = vz
            # Fallback: estimate yaw rate from motion direction when IMU not available
            try:
                if self._prev_pos is not None and self._prev_time is not None:
                    dx = px - self._prev_pos[0]
                    dy = py - self._prev_pos[1]
                    if abs(dx) + abs(dy) > 1e-6:
                        cur_yaw = math.atan2(dy, dx)
                        if self._prev_yaw is not None and (t_now - self._prev_time) > 1e-6:
                            diff = cur_yaw - self._prev_yaw
                            while diff > math.pi:
                                diff -= 2.0 * math.pi
                            while diff < -math.pi:
                                diff += 2.0 * math.pi
                            odom.twist.twist.angular.z = diff / (t_now - self._prev_time)
                            self._prev_yaw = cur_yaw
                        else:
                            self._prev_yaw = cur_yaw
            except Exception:
                odom.twist.twist.angular.z = 0.0

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
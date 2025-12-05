import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros

class UwbOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('uwb_odom_broadcaster')

        self.latest_pose = None

        # Subscribe to /uwb/pose  (PoseStamped)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/uwb/pose',
            self.pose_callback,
            10
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publish TF at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_tf)

    def pose_callback(self, msg):
        self.latest_pose = msg

    def publish_tf(self):
        if self.latest_pose is None:
            return

        # Use ROS time to keep scan and TF timestamps aligned
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # Position from PoseStamped.pose
        t.transform.translation.x = self.latest_pose.pose.position.x
        t.transform.translation.y = self.latest_pose.pose.position.y
        t.transform.translation.z = self.latest_pose.pose.position.z

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = UwbOdomBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

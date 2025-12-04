import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class UwbOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('uwb_odom_broadcaster')

        self.latest_odom = None

        # Subscribe to /uwb/odom
        self.subscription = self.create_subscription(
            Odometry,
            '/uwb/odom',
            self.odom_callback,
            10
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publish TF at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_tf)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def publish_tf(self):
        if self.latest_odom is None:
            return


        # Use ROS time to keep scan and TF timestamps aligned
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.latest_odom.pose.pose.position.x
        t.transform.translation.y = self.latest_odom.pose.pose.position.y
        t.transform.translation.z = self.latest_odom.pose.pose.position.z

        t.transform.rotation = self.latest_odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = UwbOdomBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


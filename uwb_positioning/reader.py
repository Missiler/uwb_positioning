#!/usr/bin/env python3
import re, time, serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped

POS = re.compile(r"POS[:,\s]*(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)[,\s]+(\d+)")

class UWBReadOnly(Node):
    def __init__(self):
        super().__init__('dwm1001_bridge')

        # Declare ROS parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('scale_m', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('read_crlf', True)

        # Load parameter values
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.scale_m = self.get_parameter('scale_m').value
        self.frame_id = self.get_parameter('frame_id').value
        self.read_crlf = self.get_parameter('read_crlf').value

        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_raw  = self.create_publisher(String, 'uwb/raw', qos)
        self.pub_pose = self.create_publisher(PoseStamped, 'uwb/pose', qos)
        self.pub_qf   = self.create_publisher(Int32, 'uwb/qf', qos)

        # Serial port
        self.ser = serial.Serial(
            port,
            baudrate=baud,
            timeout=0.5,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )

        self.buf = bytearray()

        # Position low-pass filter to smooth UWB noise
        self.pos_alpha = 0.20  # Filter strength (0..1): higher = faster response, less smoothing
        self.filtered_x = None
        self.filtered_y = None
        self.filtered_z = None

        self.get_logger().info(f"Opened {port} @ {baud}; read_only mode (CRLF={self.read_crlf})")

        # Timer loop (fast)
        self.create_timer(0.0, self._tick)

    def _tick(self):
        try:
            if self.read_crlf:
                b = self.ser.readline()
                if not b:
                    return
                self._handle_line_bytes(b)
            else:
                chunk = self.ser.read(256)
                if not chunk:
                    return
                self.buf.extend(chunk)
                while True:
                    i = self.buf.find(b'\r')
                    if i == -1:
                        break
                    line = bytes(self.buf[:i])
                    drop = 1
                    if i + 1 < len(self.buf) and self.buf[i+1] == 10:
                        drop = 2
                    self.buf = self.buf[i+drop:]
                    self._handle_line_bytes(line + b'\n')

        except Exception as e:
            self.get_logger().warn(f"serial read error: {e}")
            time.sleep(0.2)

    def _handle_line_bytes(self, b):
        s = b.decode('utf-8', errors='ignore').strip()
        if not s:
            return

        # Publish raw string
        self.pub_raw.publish(String(data=s))

        # Parse POS packet
        m = POS.search(s)
        if m:
            try:
                x = float(m.group(1)) * self.scale_m
                y = float(m.group(2)) * self.scale_m
                z = float(m.group(3)) * self.scale_m
                qf = int(m.group(4))
            except Exception:
                return

            # Apply low-pass filter to smooth position
            if self.filtered_x is None:
                self.filtered_x = x
                self.filtered_y = y
                self.filtered_z = z
            else:
                self.filtered_x = (1.0 - self.pos_alpha) * self.filtered_x + self.pos_alpha * x
                self.filtered_y = (1.0 - self.pos_alpha) * self.filtered_y + self.pos_alpha * y
                self.filtered_z = (1.0 - self.pos_alpha) * self.filtered_z + self.pos_alpha * z

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = self.filtered_x
            msg.pose.position.y = self.filtered_y
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0
            self.pub_pose.publish(msg)

            iqf = Int32()
            iqf.data = qf
            self.pub_qf.publish(iqf)


def main():
    rclpy.init()
    node = UWBReadOnly()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#!/usr/bin/env python3
import re, time, serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped

POS = re.compile(r"POS[:,\s]*(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)[,\s]+(\d+)")

class UWBReadOnly(Node):
    def __init__(self, port, baud, scale_m, frame_id, read_crlf=True):
        super().__init__('dwm1001_bridge')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_raw  = self.create_publisher(String, 'uwb/raw', qos)
        self.pub_pose = self.create_publisher(PoseStamped, 'uwb/pose', qos)
        self.pub_qf   = self.create_publisher(Int32, 'uwb/qf', qos)

        self.scale_m = scale_m
        self.frame_id = frame_id
        # open serial in read-only spirit (we won’t send anything)
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.5,
                                 xonxoff=False, rtscts=False, dsrdtr=False)
        # reading: try readline() (LF). if firmware emits CR-only, we’ll split manually
        self.read_crlf = read_crlf
        self.buf = bytearray()
        self.get_logger().info(f"Opened {port} @ {baud}; read_only mode")

        # fast loop timer: read & publish
        self.create_timer(0.0, self._tick)


    def _tick(self):
        try:
            if self.read_crlf:
                b = self.ser.readline()       # waits for \n
                if not b:
                    return
                self._handle_line_bytes(b)
            else:
                # CR splitter path
                chunk = self.ser.read(256)
                if not chunk:
                    return
                self.buf.extend(chunk)
                while True:
                    i = self.buf.find(b'\r')
                    if i == -1: break
                    line = bytes(self.buf[:i])
                    # drop CR and optional paired LF
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
        self.pub_raw.publish(String(data=s))
        m = POS.search(s)
        if m:
            try:
                x = float(m.group(1)) * self.scale_m
                y = float(m.group(2)) * self.scale_m
                z = float(m.group(3)) * self.scale_m
                qf = int(m.group(4))
            except Exception:
                return
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0
            self.pub_pose.publish(msg)
            iqf = Int32(); iqf.data = qf
            self.pub_qf.publish(iqf)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--scale-m', type=float, default=1.0)
    ap.add_argument('--frame-id', default='map')
    ap.add_argument('--read-crlf', action='store_true', default=True,
                    help='use .readline() (LF). leave as-is unless nothing appears')
    ap.add_argument('--read-cr', dest='read_crlf', action='store_false',
                    help='switch to CR-splitting if needed')
    args = ap.parse_args()

    rclpy.init()
    node = UWBReadOnly(args.port, args.baud, args.scale_m, args.frame_id, args.read_crlf)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser: node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()



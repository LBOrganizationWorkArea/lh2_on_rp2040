import json
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .common import POSITIONING_ROOT


class Lh2PositionNode(Node):
    def __init__(self):
        super().__init__("lh2_position_node")
        self.declare_parameter("geometry", str(POSITIONING_ROOT / "config" / "lighthouse_geometry.json"))
        self.declare_parameter("publish_coord", True)

        self.geometry_path = Path(str(self.get_parameter("geometry").value))
        self.publish_coord = bool(self.get_parameter("publish_coord").value)
        self.recent = deque(maxlen=200)

        self.position_pub = self.create_publisher(PointStamped, "/lh2/position", 10)
        self.coord_pub = self.create_publisher(Float32MultiArray, "/coord", 10)
        self.status_pub = self.create_publisher(String, "/lh2/calibration_status", 10)
        self.create_subscription(String, "/lh2/parsed", self.on_parsed, 100)

        if self.geometry_path.is_file():
            self.publish_status("ready", f"Geometry found: {self.geometry_path}")
        else:
            self.publish_status("waiting_geometry", f"Geometry not found yet: {self.geometry_path}")

    def on_parsed(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.recent.append(payload)
        # Position solving stays in tools/05_live_position.py for now.
        # This node owns the future ROS-facing output topics without changing calibration math yet.

    def publish_zero_for_transport_test(self):
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "lh2_world"
        self.position_pub.publish(point)
        if self.publish_coord:
            coord = Float32MultiArray()
            coord.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.coord_pub.publish(coord)

    def publish_status(self, state, message):
        msg = String()
        msg.data = json.dumps({
            "stamp_unix_time_s": time.time(),
            "state": state,
            "message": message,
            "geometry": str(self.geometry_path),
        }, separators=(",", ":"), sort_keys=True)
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = Lh2PositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

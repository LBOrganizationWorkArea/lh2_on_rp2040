import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import frame_to_json_payload, json_dumps, load_factory_calibs_for_ros


class Lh2ParserNode(Node):
    def __init__(self):
        super().__init__("lh2_parser_node")
        self.declare_parameter("factory_calibs", "auto")
        self.factory_calibs = load_factory_calibs_for_ros(self.get_parameter("factory_calibs").value)

        self.publisher = self.create_publisher(String, "/lh2/parsed", 50)
        self.subscription = self.create_subscription(String, "/lh2/raw_line", self.on_raw_line, 50)
        self.get_logger().info("Parsing /lh2/raw_line into JSON on /lh2/parsed")

    def on_raw_line(self, msg):
        payload = frame_to_json_payload(msg.data, self.factory_calibs)
        if payload is None:
            return
        out = String()
        out.data = json_dumps(payload)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Lh2ParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

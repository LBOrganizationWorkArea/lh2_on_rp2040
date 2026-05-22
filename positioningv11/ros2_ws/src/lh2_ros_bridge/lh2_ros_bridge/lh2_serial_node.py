try:
    import serial
except ImportError:
    serial = None

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Lh2SerialNode(Node):
    def __init__(self):
        super().__init__("lh2_serial_node")
        self.declare_parameter("port", "COM3")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout_s", 0.02)
        self.declare_parameter("reconnect_s", 2.0)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.reconnect_s = float(self.get_parameter("reconnect_s").value)
        self.publisher = self.create_publisher(String, "/lh2/raw_line", 50)
        self.serial = None
        self.last_open_attempt_s = 0.0

        if serial is None:
            raise RuntimeError("pyserial is required: pip install pyserial")

        self.open_serial()
        self.timer = self.create_timer(0.001, self.poll_serial)

    def open_serial(self):
        self.last_open_attempt_s = self.get_clock().now().nanoseconds / 1e9
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
        except serial.SerialException as exc:
            self.serial = None
            self.get_logger().warn(
                f"Cannot open/configure serial port {self.port}: {exc}. "
                f"Retrying every {self.reconnect_s:.1f}s."
            )
            return
        self.get_logger().info(f"Reading LH2P serial lines from {self.port} at {self.baudrate} baud")

    def poll_serial(self):
        if self.serial is None or not self.serial.is_open:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - self.last_open_attempt_s >= self.reconnect_s:
                self.open_serial()
            return

        try:
            raw = self.serial.readline()
        except serial.SerialException as exc:
            self.get_logger().warn(f"Serial error on {self.port}: {exc}. Reconnecting.")
            try:
                self.serial.close()
            except serial.SerialException:
                pass
            self.serial = None
            return

        if not raw:
            return
        line = raw.decode(errors="ignore").strip()
        if not line.startswith("LH2P;"):
            return

        msg = String()
        msg.data = line
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Lh2SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

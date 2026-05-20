import json
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import (
    DEFAULT_CALIBRATION_POSES,
    POSITIONING_ROOT,
    aggregate_observations,
    finite_float,
    json_dumps,
    missing_channels,
    parse_basestations,
)


class Lh2CalibrationRecorderNode(Node):
    def __init__(self):
        super().__init__("lh2_calibration_recorder_node")
        self.declare_parameter("output", str(POSITIONING_ROOT / "config" / "calibration_poses_2d_ros.json"))
        self.declare_parameter("duration_s", 4.0)
        self.declare_parameter("basestations", "4,10")
        self.declare_parameter("max_buffer_s", 30.0)
        self.declare_parameter("resume", True)

        self.output = Path(str(self.get_parameter("output").value))
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.basestations = parse_basestations(self.get_parameter("basestations").value)
        self.max_buffer_s = float(self.get_parameter("max_buffer_s").value)
        self.resume = bool(self.get_parameter("resume").value)
        self.buffer = deque()

        self.status_pub = self.create_publisher(String, "/lh2/calibration_status", 20)
        self.create_subscription(String, "/lh2/parsed", self.on_parsed, 100)
        self.create_subscription(String, "/lh2/calibration_command", self.on_command, 10)

        self.calibration = self.load_or_create_calibration()
        self.publish_status("ready", f"Recorder ready: {self.output}")

    def load_or_create_calibration(self):
        if self.resume and self.output.is_file():
            with self.output.open("r") as f:
                data = json.load(f)
            if isinstance(data, dict) and "poses" in data:
                return data

        return {
            "description": "Known 2D calibration poses for estimating Lighthouse geometry.",
            "created_unix_time_s": time.time(),
            "basestations": self.basestations,
            "duration_s_per_pose": self.duration_s,
            "frame": {
                "origin": "P0_center, drone center",
                "x_positive": "right from initial drone orientation",
                "y_positive": "front from initial drone orientation",
                "yaw": "kept fixed at 0 deg during calibration",
            },
            "poses": [],
        }

    def on_parsed(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        now = time.time()
        self.buffer.append((now, payload))
        cutoff = now - self.max_buffer_s
        while self.buffer and self.buffer[0][0] < cutoff:
            self.buffer.popleft()

    def on_command(self, msg):
        text = msg.data.strip()
        try:
            command = json.loads(text)
        except json.JSONDecodeError:
            command = {"command": "capture", "name": text}

        action = str(command.get("command", "capture")).lower()
        if action == "capture":
            self.capture_pose(command)
        elif action == "save":
            self.save()
            self.publish_status("saved", f"Saved {len(self.calibration['poses'])} poses to {self.output}")
        elif action == "reset":
            self.calibration = self.load_or_create_calibration()
            self.calibration["poses"] = []
            self.publish_status("reset", "Cleared recorded poses in memory")
        else:
            self.publish_status("error", f"Unknown command: {action}")

    def capture_pose(self, command):
        name = str(command.get("name", "")).strip()
        if not name:
            self.publish_status("error", "Capture command needs a pose name")
            return

        defaults = DEFAULT_CALIBRATION_POSES.get(name, {})
        pose = {
            "name": name,
            "x_m": finite_float(command.get("x_m", defaults.get("x_m", 0.0))),
            "y_m": finite_float(command.get("y_m", defaults.get("y_m", 0.0))),
            "z_m": finite_float(command.get("z_m", defaults.get("z_m", 0.0))),
            "roll_deg": finite_float(command.get("roll_deg", defaults.get("roll_deg", 0.0))),
            "pitch_deg": finite_float(command.get("pitch_deg", defaults.get("pitch_deg", 0.0))),
            "yaw_deg": finite_float(command.get("yaw_deg", defaults.get("yaw_deg", 0.0))),
        }

        now = time.time()
        payloads = [payload for seen_at, payload in self.buffer if now - seen_at <= self.duration_s]
        measurements = aggregate_observations(payloads)
        missing = missing_channels(measurements, self.basestations)
        pose["measurements"] = measurements
        pose["missing_channels"] = missing

        replaced = False
        for index, existing in enumerate(self.calibration["poses"]):
            if existing.get("name") == name:
                self.calibration["poses"][index] = pose
                replaced = True
                break
        if not replaced:
            self.calibration["poses"].append(pose)

        self.save()
        state = "replaced" if replaced else "captured"
        self.publish_status(
            state,
            f"{name}: {len(measurements)} measurements, {len(missing)} missing channels",
        )

    def save(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.calibration["basestations"] = self.basestations
        self.calibration["duration_s_per_pose"] = self.duration_s
        with self.output.open("w") as f:
            json.dump(self.calibration, f, indent=2)

    def publish_status(self, state, message):
        msg = String()
        msg.data = json_dumps({
            "stamp_unix_time_s": time.time(),
            "state": state,
            "message": message,
            "output": str(self.output),
            "pose_count": len(self.calibration.get("poses", [])),
        })
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = Lh2CalibrationRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

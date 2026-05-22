from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    duration_s = LaunchConfiguration("duration_s")
    output = LaunchConfiguration("output")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="COM3"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("duration_s", default_value="4.0"),
        DeclareLaunchArgument("output", default_value="config/calibration_poses_2d_ros.json"),
        Node(
            package="lh2_ros_bridge",
            executable="lh2_serial_node",
            name="lh2_serial_node",
            parameters=[{"port": port, "baudrate": baudrate}],
        ),
        Node(
            package="lh2_ros_bridge",
            executable="lh2_parser_node",
            name="lh2_parser_node",
            parameters=[{"factory_calibs": "auto"}],
        ),
        Node(
            package="lh2_ros_bridge",
            executable="lh2_calibration_recorder_node",
            name="lh2_calibration_recorder_node",
            parameters=[{"duration_s": duration_s, "output": output}],
        ),
    ])

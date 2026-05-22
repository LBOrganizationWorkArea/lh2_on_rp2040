from setuptools import setup
from glob import glob


package_name = "lh2_ros_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="elkah",
    maintainer_email="elkah@example.com",
    description="Minimal ROS2 bridge for LH2P serial data.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lh2_serial_node = lh2_ros_bridge.lh2_serial_node:main",
            "lh2_parser_node = lh2_ros_bridge.lh2_parser_node:main",
            "lh2_calibration_recorder_node = lh2_ros_bridge.lh2_calibration_recorder_node:main",
            "lh2_position_node = lh2_ros_bridge.lh2_position_node:main",
        ],
    },
)

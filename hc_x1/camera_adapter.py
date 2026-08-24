#!/usr/bin/env python3
"""Camera-only bridge from X1 private compressed topics to HC topics."""

from __future__ import annotations

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    RCLError = Exception


def camera_qos() -> QoSProfile:
    """Large images are live data: retain one frame and never build a queue."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class HcX1CameraAdapter(Node):
    """Relay only already-compressed camera frames in an isolated process."""

    def __init__(self) -> None:
        super().__init__("hc_x1_camera_adapter")
        qos = camera_qos()

        self.head_color_pub = self.create_publisher(
            CompressedImage,
            "/hc_teleop/camera_head/color/compressed",
            qos,
        )
        for topic in (
            "/io_teleop/camera_head/color/compressed",
            "/io_teleop/camera_head/color",
            "/cameras/eye/color/compressed",
        ):
            self.create_subscription(
                CompressedImage, topic, self.head_color_pub.publish, qos
            )

        self.head_depth_pub = self.create_publisher(
            CompressedImage,
            "/hc_teleop/camera_head/depth/compressed",
            qos,
        )
        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_head/depth",
            self.head_depth_pub.publish,
            qos,
        )

        self.overhead_color_pub = self.create_publisher(
            CompressedImage,
            "/hc_teleop/camera_overhead/color/compressed",
            qos,
        )
        for topic in (
            "/cameras/Bfront/color/compressed",
            "/io_teleop/camera_overhead/color/compressed",
            "/cameras/overhead/color/compressed",
        ):
            self.create_subscription(
                CompressedImage, topic, self.overhead_color_pub.publish, qos
            )

        self._relay(
            "/io_teleop/camera_d405_left/color/compressed",
            "/hc_teleop/camera_d405_left/color/compressed",
            "d405_left_color_pub",
            qos,
        )
        self._relay(
            "/io_teleop/camera_d405_right/color/compressed",
            "/hc_teleop/camera_d405_right/color/compressed",
            "d405_right_color_pub",
            qos,
        )
        self._relay(
            "/io_teleop/camera_d405_left/depth/compressed",
            "/hc_teleop/camera_d405_left/depth/compressed",
            "d405_left_depth_pub",
            qos,
        )
        self._relay(
            "/io_teleop/camera_d405_right/depth/compressed",
            "/hc_teleop/camera_d405_right/depth/compressed",
            "d405_right_depth_pub",
            qos,
        )

        custom_input = os.environ.get("HC_CUSTOM_CAMERA_INPUT", "").strip()
        custom_output = os.environ.get(
            "HC_CUSTOM_CAMERA_OUTPUT",
            "/hc_teleop/camera_custom/color/compressed",
        ).strip()
        if custom_input:
            self._relay(
                custom_input,
                custom_output,
                "custom_camera_pub",
                qos,
            )

        self.get_logger().info(
            "HC X1 camera adapter ready: compressed latest-frame relay only"
        )

    def _relay(
        self,
        input_topic: str,
        output_topic: str,
        publisher_attribute: str,
        qos: QoSProfile,
    ) -> None:
        publisher = self.create_publisher(CompressedImage, output_topic, qos)
        setattr(self, publisher_attribute, publisher)
        self.create_subscription(
            CompressedImage,
            input_topic,
            publisher.publish,
            qos,
        )


def main() -> None:
    domain_id = int(os.environ.get("ROS_DOMAIN_ID", 14))
    rclpy.init(domain_id=domain_id)
    node = None
    try:
        node = HcX1CameraAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()

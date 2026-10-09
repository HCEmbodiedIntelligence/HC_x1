#!/usr/bin/env python3
"""Low-latency X1 control and state bridge for the HC standard interface."""

from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    RCLError = Exception


def latest_qos(reliability: ReliabilityPolicy) -> QoSProfile:
    """Keep only the newest control sample so stale frames cannot accumulate."""
    return QoSProfile(
        reliability=reliability,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class HcX1ControlAdapter(Node):
    """Bridge control/state topics without sharing an executor with cameras."""

    def __init__(self) -> None:
        super().__init__("hc_x1_control_adapter")

        latest_reliable = latest_qos(ReliabilityPolicy.RELIABLE)
        latest_best_effort = latest_qos(ReliabilityPolicy.BEST_EFFORT)

        # Robot feedback stays reliable for the IK subscribers, while
        # KEEP_LAST(1) prevents network jitter from delivering an old backlog.
        self.hardware_ready_pub = self.create_publisher(
            Bool, "/status/hardware/ready", latest_reliable
        )
        self.create_subscription(
            Bool,
            "/io_teleop/hardware_ready",
            self.hardware_ready_pub.publish,
            latest_reliable,
        )

        self.joint_state_pub = self.create_publisher(
            JointState, "/observation/joint_states", latest_reliable
        )
        self.create_subscription(
            JointState,
            "/io_teleop/joint_states",
            self.joint_state_pub.publish,
            latest_best_effort,
        )

        # Arm joint commands use the dedicated 100 Hz smoother and intentionally
        # do not pass through this bridge.
        self.base_move_pub = self.create_publisher(
            Float64MultiArray, "/io_teleop/target_base_move", latest_reliable
        )
        self.create_subscription(
            Float64MultiArray,
            "/action/base/command",
            self.base_move_pub.publish,
            latest_best_effort,
        )

        self.finger_left_pub = self.create_publisher(
            JointState,
            "/io_teleop/joint_cmd_finger_left",
            latest_best_effort,
        )
        self.create_subscription(
            JointState,
            "/action/left_hand/joint_states",
            self.finger_left_pub.publish,
            latest_best_effort,
        )
        self.finger_right_pub = self.create_publisher(
            JointState,
            "/io_teleop/joint_cmd_finger_right",
            latest_best_effort,
        )
        self.create_subscription(
            JointState,
            "/action/right_hand/joint_states",
            self.finger_right_pub.publish,
            latest_best_effort,
        )

        self.hand_state_pub = self.create_publisher(
            JointState, "/observation/hands/joint_states", latest_reliable
        )
        self.create_subscription(
            JointState,
            "/io_teleop/hand_joint_states",
            self.hand_state_pub.publish,
            latest_best_effort,
        )
        self.hand_trigger_pub = self.create_publisher(
            PoseArray, "/observation/hands/triggers", latest_reliable
        )
        self.create_subscription(
            PoseArray,
            "/io_teleop/hand_trigger",
            self.hand_trigger_pub.publish,
            latest_best_effort,
        )

        self.chassis_state_pub = self.create_publisher(
            String, "/status/base", latest_reliable
        )
        self.create_subscription(
            String,
            "/io_teleop/chassis_state",
            self.chassis_state_pub.publish,
            latest_best_effort,
        )

        self.get_logger().info(
            "HC X1 control adapter ready: latest-only joint, finger and base bridge"
        )


def main() -> None:
    domain_id = int(os.environ.get("ROS_DOMAIN_ID", 14))
    rclpy.init(domain_id=domain_id)
    node = None
    try:
        node = HcX1ControlAdapter()
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

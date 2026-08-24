#!/usr/bin/env python3
"""监听 joint_cmd_finger_* 并打印映射后的 10 轴 motor 位置，用于排查左右拇指不对称。

用法:
  export ROS_DOMAIN_ID=14
  source install/setup.bash
  python3 src/omnihand_o10_rs485_bridge/scripts/test_finger_cmd_debug.py

可选发布测试指令 (不连 TeleXperience):
  python3 .../test_finger_cmd_debug.py --publish-test zero
  python3 .../test_finger_cmd_debug.py --publish-test open
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

# 与 hand_config.hpp 保持同步
RIGHT_RANGE = [
    (["R_thumb_roll_joint"], (0.4, 1.121), (4096, 800)),
    (["R_thumb_abad_joint"], (-1.642, 0.045), (4096, 0)),
    (["R_thumb_mcp_joint"], (0.0, 1.0), (4096, 0)),
    (["R_index_abad_joint"], (-0.13, -0.05), (4096, 0)),
    (["R_index_pip_joint"], (0.0, 2.5), (4096, 0)),
    (["R_middle_pip_joint"], (0.0, 2.5), (4096, 0)),
    (["R_ring_abad_joint"], (0.05, 0.13), (0, 4096)),
    (["R_ring_pip_joint"], (0.0, 2.5), (4096, 0)),
    (["R_pinky_abad_joint"], (0.05, 0.13), (4096, 0)),
    (["R_pinky_pip_joint"], (0.0, 2.5), (4096, 0)),
]

LEFT_RANGE = [
    (["L_thumb_roll_joint"], (-0.4, -1.121), (4096, 800)),
    (["L_thumb_abad_joint"], (-0.045, 1.642), (4096, 0)),
    (["L_thumb_mcp_joint"], (0.0, -1.0), (4096, 0)),
    (["L_index_abad_joint"], (0.05, 0.13), (4096, 0)),
    (["L_index_pip_joint"], (0.0, 2.5), (4096, 0)),
    (["L_middle_pip_joint"], (0.0, 2.5), (4096, 0)),
    (["L_ring_abad_joint"], (-0.13, -0.05), (0, 4096)),
    (["L_ring_pip_joint"], (0.0, 2.5), (4096, 0)),
    (["L_pinky_abad_joint"], (-0.13, 0.05), (4096, 0)),
    (["L_pinky_pip_joint"], (0.0, 2.5), (4096, 0)),
]

MOTOR_LABELS = [
    "thumb_roll", "thumb_abad", "thumb_mcp", "index_abad", "index_pip",
    "middle_pip", "ring_abad", "ring_pip", "pinky_abad", "pinky_pip",
]

K_OPEN_MOTOR = [3100, 67, 4091, 111, 4090, 4091, 4032, 4096, 74, 4091]

RIGHT_JOINTS_16 = [
    "R_thumb_roll_joint", "R_thumb_abad_joint", "R_thumb_mcp_joint",
    "R_thumb_pip_joint", "R_thumb_dip_joint", "R_index_abad_joint",
    "R_index_mcp_joint", "R_index_pip_joint", "R_index_dip_joint",
    "R_middle_mcp_joint", "R_middle_pip_joint", "R_middle_dip_joint",
    "R_ring_abad_joint", "R_ring_pip_joint", "R_ring_dip_joint",
    "R_pinky_abad_joint", "R_pinky_pip_joint", "R_pinky_dip_joint",
]

LEFT_JOINTS_16 = [n.replace("R_", "L_") for n in RIGHT_JOINTS_16]


def map_to_motor(
    finger_range: Sequence[Tuple[List[str], Tuple[float, float], Tuple[int, int]]],
    names: Sequence[str],
    positions: Sequence[float],
) -> List[int]:
    joint_dic = {n: p for n, p in zip(names, positions)}
    motors: List[int] = []
    for joint_names, in_rng, out_rng in finger_range:
        val = sum(joint_dic.get(n, 0.0) for n in joint_names)
        span = in_rng[1] - in_rng[0]
        t = 0.0 if abs(span) < 1e-9 else max(0.0, min(1.0, (val - in_rng[0]) / span))
        mapped = out_rng[0] + t * (out_rng[1] - out_rng[0])
        motors.append(int(mapped) & 0xFFFF)
    return motors


def thumb_summary(side: str, motors: List[int], raw_thumb: Dict[str, float]) -> str:
    lines = [f"[{side}] motor 拇指: roll={motors[0]} abad={motors[1]} mcp={motors[2]}"]
    if raw_thumb:
        lines.append(f"       原始 rad: {raw_thumb}")
    return "\n".join(lines)


def extract_thumb_raw(names: Sequence[str], positions: Sequence[float]) -> Dict[str, float]:
    out = {}
    for n, p in zip(names, positions):
        if "thumb" in n and ("roll" in n or "abad" in n or "mcp" in n):
            out[n.split("_", 1)[1]] = round(p, 4)
    return out


class FingerCmdDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("finger_cmd_debug")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self._last_r: List[int] | None = None
        self._last_l: List[int] | None = None
        self.create_subscription(JointState, "/io_teleop/joint_cmd_finger_right", qos,
                                 lambda m: self._on_msg("右", RIGHT_RANGE, m, "r"))
        self.create_subscription(JointState, "/io_teleop/joint_cmd_finger_left", qos,
                                 lambda m: self._on_msg("左", LEFT_RANGE, m, "l"))
        self.get_logger().info(
            "监听 /io_teleop/joint_cmd_finger_left|right; init 张开 motor=%s", K_OPEN_MOTOR[:3]
        )

    def _on_msg(self, side: str, rng, msg: JointState, key: str) -> None:
        motors = map_to_motor(rng, msg.name, msg.position)
        raw = extract_thumb_raw(msg.name, msg.position)
        self.get_logger().info("\n%s\n%s", thumb_summary(side, motors, raw),
                               self._format_all(motors))
        if key == "r":
            self._last_r = motors
        else:
            self._last_l = motors
        if self._last_r and self._last_l:
            diff = [self._last_r[i] - self._last_l[i] for i in range(3)]
            self.get_logger().info("拇指 motor 差 (R-L): roll=%d abad=%d mcp=%d",
                                   diff[0], diff[1], diff[2])

    @staticmethod
    def _format_all(motors: List[int]) -> str:
        parts = [f"  [{i}] {MOTOR_LABELS[i]}={motors[i]}" for i in range(10)]
        return "  全轴: " + ", ".join(parts)


def publish_test(node: Node, mode: str) -> None:
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    pub_r = node.create_publisher(JointState, "/io_teleop/joint_cmd_finger_right", qos)
    pub_l = node.create_publisher(JointState, "/io_teleop/joint_cmd_finger_left", qos)

    if mode == "zero":
        r_pos = [0.0] * len(RIGHT_JOINTS_16)
        l_pos = [0.0] * len(LEFT_JOINTS_16)
    elif mode == "open":
        r_pos = [0.0] * len(RIGHT_JOINTS_16)
        l_pos = [0.0] * len(LEFT_JOINTS_16)
        for i, name in enumerate(RIGHT_JOINTS_16):
            if "thumb_roll" in name:
                r_pos[i] = RIGHT_RANGE[0][1][0]
            elif "thumb_abad" in name:
                r_pos[i] = RIGHT_RANGE[1][1][0]
            elif "thumb_mcp" in name:
                r_pos[i] = RIGHT_RANGE[2][1][0]
        for i, name in enumerate(LEFT_JOINTS_16):
            if "thumb_roll" in name:
                l_pos[i] = LEFT_RANGE[0][1][0]
            elif "thumb_abad" in name:
                l_pos[i] = LEFT_RANGE[1][1][0]
            elif "thumb_mcp" in name:
                l_pos[i] = LEFT_RANGE[2][1][0]
    else:
        raise ValueError(mode)

    msg_r = JointState(name=RIGHT_JOINTS_16, position=r_pos)
    msg_l = JointState(name=LEFT_JOINTS_16, position=l_pos)

    node.get_logger().info("发布测试 joint_cmd: mode=%s", mode)
    for _ in range(50):
        pub_r.publish(msg_r)
        pub_l.publish(msg_l)
        rclpy.spin_once(node, timeout_sec=0.02)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--publish-test", choices=["zero", "open"],
                        help="发布测试 joint_cmd (50 帧 @ ~50Hz)")
    args = parser.parse_args()

    rclpy.init()
    node = FingerCmdDebugNode()
    if args.publish_test:
        publish_test(node, args.publish_test)
        # 继续监听自己发的消息
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

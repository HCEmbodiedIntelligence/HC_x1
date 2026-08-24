#!/usr/bin/env python3
"""Dedicated 100 Hz conditioner between standard HC commands and X1 drivers."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    RCLError = Exception
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

from hc_x1.command_smoothing import JointTrajectorySmoother, SmoothingLimits


def load_smoothing_config(path: str) -> dict:
    config_path = Path(path).expanduser().resolve()
    with config_path.open(encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
    control = root.get("control", {})
    config = control.get("command_smoothing", {})
    if not isinstance(config, dict):
        raise ValueError("control.command_smoothing must be an object")
    return config


class JointCommandSmootherNode(Node):
    def __init__(self, config_path: str):
        config = load_smoothing_config(config_path)
        super().__init__(str(config.get("node_name", "hc_x1_joint_command_smoother")))

        self.rate_hz = float(config.get("rate_hz", 100.0))
        self.input_timeout = float(config.get("input_timeout", 0.15))
        self.feedback_timeout = float(config.get("feedback_timeout", 0.5))
        self.feedback_resync_threshold = float(
            config.get("feedback_resync_threshold", 0.5)
        )
        self.publisher_conflict_check_period = float(
            config.get("publisher_conflict_check_period", 0.5)
        )
        self.conflict_activity_timeout = float(
            config.get("conflict_activity_timeout", 0.25)
        )
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("command_smoothing.rate_hz must be positive")
        for key, value in (
            ("input_timeout", self.input_timeout),
            ("feedback_timeout", self.feedback_timeout),
            ("feedback_resync_threshold", self.feedback_resync_threshold),
            (
                "publisher_conflict_check_period",
                self.publisher_conflict_check_period,
            ),
            ("conflict_activity_timeout", self.conflict_activity_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"command_smoothing.{key} must be positive")

        self.smoother = JointTrajectorySmoother(
            SmoothingLimits(
                time_constant=float(config.get("time_constant", 0.04)),
                max_velocity=float(config.get("max_velocity", 2.5)),
                max_acceleration=float(config.get("max_acceleration", 12.0)),
                deadband=float(config.get("deadband", 0.00015)),
            )
        )
        self.input_topic = str(config.get("input_topic", "/hc_teleop/joint_cmd"))
        self.feedback_topic = str(
            config.get("feedback_topic", "/io_teleop/joint_states")
        )
        self.output_topic = str(config.get("output_topic", "/io_teleop/joint_cmd"))
        self.conflict_activity_topic = str(
            config.get("conflict_activity_topic", "/io_teleop/sol_q")
        )
        allowed_publishers = config.get(
            "allowed_idle_output_publishers", ["robot_control_pid_node"]
        )
        if not isinstance(allowed_publishers, list) or not all(
            isinstance(name, str) and name.strip() for name in allowed_publishers
        ):
            raise ValueError(
                "command_smoothing.allowed_idle_output_publishers must be a string list"
            )
        self.allowed_idle_output_publishers = {
            name.strip().lstrip("/") for name in allowed_publishers
        }
        for key, topic in (
            ("input_topic", self.input_topic),
            ("feedback_topic", self.feedback_topic),
            ("output_topic", self.output_topic),
            ("conflict_activity_topic", self.conflict_activity_topic),
        ):
            if not topic.startswith("/"):
                raise ValueError(f"command_smoothing.{key} must start with /")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(
            JointState, self.output_topic, reliable_qos
        )
        self.create_subscription(
            JointState, self.input_topic, self._target_callback, reliable_qos
        )
        self.create_subscription(
            JointState,
            self.feedback_topic,
            self._feedback_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.conflict_activity_topic,
            self._conflict_activity_callback,
            qos_profile_sensor_data,
        )

        self.target_names: list[str] = []
        self.target_positions: list[float] = []
        self.feedback: dict[str, float] = {}
        self.last_target_time = 0.0
        self.last_feedback_time = 0.0
        self.last_tick_time = time.monotonic()
        self.last_conflict_check_time = 0.0
        self.last_conflict_activity_time = 0.0
        self.foreign_output_publishers: list[str] = []
        self.output_conflicts: tuple[str, ...] = ()
        self.active = False
        self.timed_out = False
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        limits = self.smoother.limits
        self.get_logger().info(
            "dedicated joint smoother ready: "
            f"{self.input_topic} -> {self.output_topic}, rate={self.rate_hz:.1f} Hz, "
            f"tau={limits.time_constant:.3f}s, velocity={limits.max_velocity:.2f} rad/s, "
            f"acceleration={limits.max_acceleration:.2f} rad/s^2"
        )

    def _target_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position) or not message.name:
            return
        if len(set(message.name)) != len(message.name):
            return
        positions = [float(value) for value in message.position]
        if not all(math.isfinite(value) for value in positions):
            return
        self.target_names = list(message.name)
        self.target_positions = positions
        self.last_target_time = time.monotonic()

    def _feedback_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, (float(value) for value in message.position)))
        if not all(math.isfinite(value) for value in values.values()):
            return
        self.feedback = values
        self.last_feedback_time = time.monotonic()

    def _conflict_activity_callback(self, _message: JointState) -> None:
        self.last_conflict_activity_time = time.monotonic()

    def _fresh_feedback(self, now: float) -> dict[str, float]:
        if now - self.last_feedback_time <= self.feedback_timeout:
            return self.feedback
        return {}

    def _needs_feedback_resync(self, feedback: dict[str, float]) -> bool:
        for name, position in self.smoother.positions.items():
            if name in feedback and abs(feedback[name] - position) > self.feedback_resync_threshold:
                return True
        return False

    def _foreign_output_publishers(self) -> list[str]:
        publishers = self.get_publishers_info_by_topic(self.output_topic)
        own_namespace = self.get_namespace().rstrip("/")
        own_name = self.get_name()
        foreign = []
        for endpoint in publishers:
            namespace = endpoint.node_namespace.rstrip("/")
            if endpoint.node_name == own_name and namespace == own_namespace:
                continue
            if endpoint.node_name.lstrip("/") in self.allowed_idle_output_publishers:
                continue
            foreign.append(f"{endpoint.node_namespace}/{endpoint.node_name}")
        return sorted(set(name.replace("//", "/") for name in foreign))

    def _output_is_exclusive(self, now: float) -> bool:
        if (
            now - self.last_conflict_check_time
            >= self.publisher_conflict_check_period
        ):
            self.last_conflict_check_time = now
            self.foreign_output_publishers = self._foreign_output_publishers()

        conflicts = list(self.foreign_output_publishers)
        if (
            self.last_conflict_activity_time > 0.0
            and now - self.last_conflict_activity_time
            <= self.conflict_activity_timeout
        ):
            conflicts.append(f"active input {self.conflict_activity_topic}")
        current = tuple(conflicts)
        if current and not self.output_conflicts:
            self.get_logger().error(
                "joint command output blocked: competing IO control is active: "
                f"{', '.join(current)}"
            )
        elif not current and self.output_conflicts:
            self.get_logger().info(
                "joint command output is exclusive again; waiting for fresh target"
            )
        if current != self.output_conflicts:
            self.active = False
            self.smoother.clear()
        self.output_conflicts = current
        return not current

    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last_tick_time
        self.last_tick_time = now
        dt = min(max(elapsed, 0.002), 0.03)

        if not self._output_is_exclusive(now):
            return

        if (
            not self.target_names
            or now - self.last_target_time > self.input_timeout
        ):
            if self.active:
                self.active = False
                self.smoother.clear()
                if not self.timed_out:
                    self.get_logger().warning(
                        "joint command input timed out; stopped publishing for driver watchdog hold"
                    )
                    self.timed_out = True
            return

        feedback = self._fresh_feedback(now)
        if not self.active or self._needs_feedback_resync(feedback):
            self.smoother.reset(
                self.target_names, self.target_positions, feedback=feedback
            )
            self.active = True
            if self.timed_out:
                self.get_logger().info("joint command stream resumed from feedback")
            self.timed_out = False

        positions, _velocities = self.smoother.step(
            self.target_names,
            self.target_positions,
            dt,
            feedback=feedback,
        )
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = list(self.target_names)
        output.position = positions
        self.publisher.publish(output)


def default_config() -> str:
    return str(PROJECT_ROOT / "config" / "x1_hardware.yaml")


def main() -> None:
    parser = argparse.ArgumentParser(description="Fixed-rate HC joint command smoother")
    parser.add_argument("--config", default=default_config())
    arguments = parser.parse_args()
    domain_id = int(os.environ.get("ROS_DOMAIN_ID", 14))
    rclpy.init(args=[], domain_id=domain_id)
    node = None
    try:
        node = JointCommandSmootherNode(arguments.config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

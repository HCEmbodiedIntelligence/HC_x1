"""Deterministic joint-space command shaping for real-robot teleoperation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence


_TRUE_VALUES = {"true", "1", "yes", "on"}
_FALSE_VALUES = {"false", "0", "no", "off"}


def disabled_joints_for_launch(
    launch_args: Sequence[str], joint_groups: Mapping[str, Sequence[str]]
) -> set[str]:
    """Resolve read-only/disabled hardware groups from the launch arguments.

    Arguments unrelated to motion control are intentionally ignored so the
    launcher can forward its complete argument list without maintaining a
    second parser in shell.
    """
    enabled = {
        "arms": True,
        "arms_control": True,
        "left_arm_control": True,
        "right_arm_control": True,
        "waist": True,
        "waist_control": True,
    }
    for argument in launch_args:
        name, separator, raw_value = argument.partition(":=")
        if not separator or name not in enabled:
            continue
        value = raw_value.lower()
        if value not in _TRUE_VALUES | _FALSE_VALUES:
            raise ValueError(f"invalid launch argument: {argument}")
        enabled[name] = value in _TRUE_VALUES

    disabled_groups = set()
    if not (enabled["arms"] and enabled["arms_control"]):
        disabled_groups.update(("left_arm", "right_arm"))
    else:
        if not enabled["left_arm_control"]:
            disabled_groups.add("left_arm")
        if not enabled["right_arm_control"]:
            disabled_groups.add("right_arm")
    if not (enabled["waist"] and enabled["waist_control"]):
        disabled_groups.add("waist")

    disabled: set[str] = set()
    for group in disabled_groups:
        names = joint_groups.get(group, ())
        if isinstance(names, (str, bytes)):
            raise ValueError(f"command_smoothing.joint_groups.{group} must be a list")
        disabled.update(str(name) for name in names)
    return disabled


def command_stream_state(age: float, hold_timeout: float, stop_timeout: float) -> str:
    """Classify a command gap without turning a short scheduler stall into a stop."""
    values = (age, hold_timeout, stop_timeout)
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError("command stream timing must be finite and non-negative")
    if stop_timeout <= hold_timeout:
        raise ValueError("stop_timeout must be greater than hold_timeout")
    if age <= hold_timeout:
        return "fresh"
    if age <= stop_timeout:
        return "hold"
    return "stop"


@dataclass(frozen=True)
class SmoothingLimits:
    """Limits for the fixed-rate joint trajectory generator."""

    time_constant: float = 0.04
    max_velocity: float = 2.5
    max_acceleration: float = 12.0
    deadband: float = 0.00015
    braking_damping: float = 0.9

    def validate(self) -> None:
        values = (
            self.time_constant,
            self.max_velocity,
            self.max_acceleration,
            self.deadband,
            self.braking_damping,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("joint smoothing limits must be finite")
        if self.time_constant <= 0.0:
            raise ValueError("joint smoothing time_constant must be positive")
        if self.max_velocity <= 0.0:
            raise ValueError("joint smoothing max_velocity must be positive")
        if self.max_acceleration <= 0.0:
            raise ValueError("joint smoothing max_acceleration must be positive")
        if not 0.0 < self.braking_damping <= 1.0:
            raise ValueError("joint smoothing braking_damping must be in (0, 1]")
        if self.deadband < 0.0:
            raise ValueError("joint smoothing deadband must be non-negative")


class JointTrajectorySmoother:
    """Turn arbitrary position targets into bounded position trajectories.

    The target is converted to a desired velocity through ``time_constant``.
    Velocity and acceleration are then limited independently.  This mirrors the
    separate ``sol_q -> control_pid -> joint_cmd`` structure used by the IO
    teleoperation stack while remaining deterministic and independent of ROS.
    """

    def __init__(self, limits: SmoothingLimits):
        limits.validate()
        self.limits = limits
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}

    def clear(self) -> None:
        self.positions.clear()
        self.velocities.clear()

    def divergent_joints(
        self,
        names: Sequence[str],
        feedback: Mapping[str, float],
        threshold: float,
    ) -> list[str]:
        """Report tracking lag without mutating the generated trajectory.

        Live feedback is deliberately observational while a command stream is
        active.  Snapping the generated position back to a lagging motor turns
        ordinary tracking delay into a discontinuous command and makes the
        joint repeatedly accelerate from rest.  Recovery paths still call
        :meth:`reset` explicitly before a stream becomes active again.
        """
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise ValueError("feedback resync threshold must be finite and positive")
        divergent = []
        for name in names:
            if name not in self.positions or name not in feedback:
                continue
            actual = float(feedback[name])
            if not math.isfinite(actual):
                continue
            if abs(actual - self.positions[name]) <= threshold:
                continue
            divergent.append(name)
        return divergent

    def reset(
        self,
        names: Sequence[str],
        targets: Sequence[float],
        feedback: Mapping[str, float] | None = None,
    ) -> list[float]:
        values = self._validated_targets(names, targets)
        feedback = feedback or {}
        self.clear()
        output = []
        for name, target in zip(names, values):
            source = feedback.get(name, target)
            position = float(source) if math.isfinite(float(source)) else target
            self.positions[name] = position
            self.velocities[name] = 0.0
            output.append(position)
        return output

    def step(
        self,
        names: Sequence[str],
        targets: Sequence[float],
        dt: float,
        feedback: Mapping[str, float] | None = None,
    ) -> tuple[list[float], list[float]]:
        values = self._validated_targets(names, targets)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("joint smoothing dt must be finite and positive")
        feedback = feedback or {}
        positions: list[float] = []
        velocities: list[float] = []
        limits = self.limits

        for name, target in zip(names, values):
            if name not in self.positions:
                source = feedback.get(name, target)
                position = float(source) if math.isfinite(float(source)) else target
                self.positions[name] = position
                self.velocities[name] = 0.0

            position = self.positions[name]
            velocity = self.velocities[name]
            error = target - position

            if (
                abs(error) <= limits.deadband
                and abs(velocity) <= limits.max_acceleration * dt
            ):
                velocity = 0.0
            else:
                # Continuous stopping-speed envelope removes the discontinuous
                # cruise/zero switch. Existing velocity/acceleration bounds and
                # the stationary deadband still apply without any relaxation.
                desired_speed = min(
                    limits.max_velocity,
                    abs(error) / limits.time_constant,
                    limits.braking_damping * math.sqrt(2.0 * limits.max_acceleration * abs(error)),
                )
                desired_velocity = math.copysign(desired_speed, error)
                velocity_delta = max(
                    -limits.max_acceleration * dt,
                    min(
                        limits.max_acceleration * dt,
                        desired_velocity - velocity,
                    ),
                )
                velocity = max(
                    -limits.max_velocity,
                    min(limits.max_velocity, velocity + velocity_delta),
                )
                position += velocity * dt

            self.positions[name] = position
            self.velocities[name] = velocity
            positions.append(position)
            velocities.append(velocity)

        active_names = set(names)
        for stale_name in set(self.positions) - active_names:
            self.positions.pop(stale_name, None)
            self.velocities.pop(stale_name, None)
        return positions, velocities

    @staticmethod
    def _validated_targets(
        names: Sequence[str], targets: Sequence[float]
    ) -> list[float]:
        if len(names) != len(targets) or not names:
            raise ValueError("joint names and targets must be non-empty and equal length")
        if len(set(names)) != len(names):
            raise ValueError("joint names must be unique")
        values = [float(value) for value in targets]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("joint targets must be finite")
        return values

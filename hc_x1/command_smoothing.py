"""Deterministic joint-space command shaping for real-robot teleoperation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence


@dataclass(frozen=True)
class SmoothingLimits:
    """Limits for the fixed-rate joint trajectory generator."""

    time_constant: float = 0.04
    max_velocity: float = 2.5
    max_acceleration: float = 12.0
    deadband: float = 0.00015

    def validate(self) -> None:
        values = (
            self.time_constant,
            self.max_velocity,
            self.max_acceleration,
            self.deadband,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("joint smoothing limits must be finite")
        if self.time_constant <= 0.0:
            raise ValueError("joint smoothing time_constant must be positive")
        if self.max_velocity <= 0.0:
            raise ValueError("joint smoothing max_velocity must be positive")
        if self.max_acceleration <= 0.0:
            raise ValueError("joint smoothing max_acceleration must be positive")
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
                # Track ordinary motion through a first-order target velocity,
                # but start braking as soon as the remaining error is no larger
                # than the distance needed to stop at the configured maximum
                # acceleration. Asking for zero velocity here (instead of
                # snapping the output to the target) keeps the final sample
                # acceleration-bounded as well.
                desired_speed = min(
                    limits.max_velocity, abs(error) / limits.time_constant
                )
                desired_velocity = math.copysign(desired_speed, error)
                braking_distance = velocity * velocity / (
                    2.0 * limits.max_acceleration
                )
                if error * velocity > 0.0 and braking_distance >= abs(error):
                    desired_velocity = 0.0
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

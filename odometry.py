"""Differential-drive odometry for Roomba 680 encoder counts."""

import math
from dataclasses import dataclass
from typing import Optional

# Roomba 680 / OI encoder parameters
WHEEL_DIAMETER = 0.072       # m
WHEEL_BASE = 0.235           # m
TICKS_PER_REV = 508.8
ENCODER_MODULO = 65536


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class Odometry:
    """Estimate a 2D pose from unsigned 16-bit wheel encoder counts."""

    def __init__(
        self,
        wheel_diameter: float = WHEEL_DIAMETER,
        wheel_base: float = WHEEL_BASE,
        ticks_per_rev: float = TICKS_PER_REV,
    ) -> None:
        if wheel_diameter <= 0 or wheel_base <= 0 or ticks_per_rev <= 0:
            raise ValueError("wheel dimensions and ticks_per_rev must be positive")
        self.wheel_diameter = float(wheel_diameter)
        self.wheel_base = float(wheel_base)
        self.ticks_per_rev = float(ticks_per_rev)
        self.pose = Pose2D()
        self.prev_left: Optional[int] = None
        self.prev_right: Optional[int] = None

    @staticmethod
    def tick_diff(current: int, previous: int) -> int:
        """Return the shortest signed delta between 16-bit tick counts."""
        Odometry._validate_encoder(current, "current")
        Odometry._validate_encoder(previous, "previous")
        return (
            (current - previous + 32768) % ENCODER_MODULO
        ) - 32768

    def reset(self, pose: Optional[Pose2D] = None) -> None:
        """Reset the pose and require a fresh encoder baseline."""
        self.pose = Pose2D() if pose is None else Pose2D(pose.x, pose.y, pose.theta)
        self.pose.theta = self._normalize_angle(self.pose.theta)
        self.prev_left = None
        self.prev_right = None

    def update(self, left: int, right: int) -> Pose2D:
        """Update and return pose from left/right unsigned encoder counts."""
        self._validate_encoder(left, "left")
        self._validate_encoder(right, "right")
        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            return self.pose

        dl_ticks = self.tick_diff(left, self.prev_left)
        dr_ticks = self.tick_diff(right, self.prev_right)

        self.prev_left = left
        self.prev_right = right

        meters_per_tick = (
            math.pi * self.wheel_diameter / self.ticks_per_rev
        )

        dl = dl_ticks * meters_per_tick
        dr = dr_ticks * meters_per_tick

        ds = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_base

        theta_mid = self.pose.theta + dtheta / 2.0

        self.pose.x += ds * math.cos(theta_mid)
        self.pose.y += ds * math.sin(theta_mid)

        self.pose.theta = self._normalize_angle(self.pose.theta + dtheta)

        return self.pose

    @staticmethod
    def _validate_encoder(value: int, name: str) -> None:
        if not isinstance(value, int) or isinstance(value, bool):
            raise TypeError(f"{name} encoder count must be an integer")
        if not 0 <= value < ENCODER_MODULO:
            raise ValueError(f"{name} encoder count must be from 0 to 65535")

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

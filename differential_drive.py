"""Compatibility import for the shared differential-drive conversion."""

from simulation.differential_drive import (  # noqa: F401
    DEFAULT_MAX_WHEEL_SPEED_MM_S,
    DEFAULT_TRACK_WIDTH_M,
    OI_MAX_WHEEL_SPEED_MM_S,
    twist_to_wheel_speeds,
)

__all__ = [
    "DEFAULT_MAX_WHEEL_SPEED_MM_S",
    "DEFAULT_TRACK_WIDTH_M",
    "OI_MAX_WHEEL_SPEED_MM_S",
    "twist_to_wheel_speeds",
]

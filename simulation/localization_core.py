"""ROS-independent wheel odometry and map/odom pose fusion."""

import math
import struct
from dataclasses import dataclass


ENCODER_MODULO = 65536


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))
    return value


def decode_encoder_response(response):
    """Decode OI Query List packets 43 then 44 as unsigned big-endian counts."""
    if not isinstance(response, (bytes, bytearray)):
        raise TypeError("encoder response must be bytes")
    if len(response) != 4:
        raise ValueError("encoder response must be exactly 4 bytes")
    return struct.unpack(">HH", response)


def encoder_delta(current, previous):
    for value, name in ((current, "current"), (previous, "previous")):
        if not isinstance(value, int) or isinstance(value, bool):
            raise TypeError("{} encoder count must be an integer".format(name))
        if not 0 <= value < ENCODER_MODULO:
            raise ValueError("{} encoder count must be from 0 to 65535".format(name))
    return (current - previous + 32768) % ENCODER_MODULO - 32768


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class WheelUpdate:
    pose: Pose2D
    linear_velocity: float
    angular_velocity: float
    timestamp: float
    initialized: bool
    left_delta: int = 0
    right_delta: int = 0


class WheelOdometry:
    def __init__(
        self,
        encoder_counts_per_rev,
        wheel_diameter_m,
        wheel_separation_m,
        max_tick_delta=2000,
    ):
        self.counts_per_rev = _finite(encoder_counts_per_rev, "encoder_counts_per_rev")
        self.wheel_diameter = _finite(wheel_diameter_m, "wheel_diameter_m")
        self.wheel_separation = _finite(wheel_separation_m, "wheel_separation_m")
        if min(self.counts_per_rev, self.wheel_diameter, self.wheel_separation) <= 0:
            raise ValueError("wheel parameters must be positive")
        if not isinstance(max_tick_delta, int) or max_tick_delta < 1:
            raise ValueError("max_tick_delta must be a positive integer")
        self.max_tick_delta = max_tick_delta
        self.pose = Pose2D()
        self.previous = None
        self.last_timestamp = None

    def update(self, left, right, timestamp):
        timestamp = _finite(timestamp, "timestamp")
        encoder_delta(left, left)
        encoder_delta(right, right)
        if self.last_timestamp is not None and timestamp <= self.last_timestamp:
            raise ValueError("encoder timestamp must increase")
        if self.previous is None:
            self.previous = (left, right)
            self.last_timestamp = timestamp
            return WheelUpdate(self.pose, 0.0, 0.0, timestamp, False)

        dl_ticks = encoder_delta(left, self.previous[0])
        dr_ticks = encoder_delta(right, self.previous[1])
        if abs(dl_ticks) > self.max_tick_delta or abs(dr_ticks) > self.max_tick_delta:
            raise ValueError("encoder jump exceeds max_tick_delta")
        dt = timestamp - self.last_timestamp
        metres_per_tick = math.pi * self.wheel_diameter / self.counts_per_rev
        dl = dl_ticks * metres_per_tick
        dr = dr_ticks * metres_per_tick
        distance = (dl + dr) / 2.0
        delta_yaw = (dr - dl) / self.wheel_separation
        midpoint = self.pose.yaw + delta_yaw / 2.0
        self.pose = Pose2D(
            self.pose.x + distance * math.cos(midpoint),
            self.pose.y + distance * math.sin(midpoint),
            normalize_angle(self.pose.yaw + delta_yaw),
        )
        self.previous = (left, right)
        self.last_timestamp = timestamp
        return WheelUpdate(
            self.pose,
            distance / dt,
            delta_yaw / dt,
            timestamp,
            True,
            dl_ticks,
            dr_ticks,
        )

    def resynchronize(self):
        """Keep pose but require the next valid counts to establish a baseline."""
        self.previous = None
        self.last_timestamp = None


@dataclass(frozen=True)
class PoseComparison:
    aruco_relative: Pose2D
    position_error_m: float
    yaw_error_rad: float
    aruco_age_s: float


class RelativePoseComparator:
    """Compare local odometry with ArUco displacement without mixing frames."""

    def __init__(self):
        self.aruco_origin = None
        self.latest_aruco = None
        self.latest_timestamp = None

    def observe_aruco(self, pose, timestamp):
        timestamp = _finite(timestamp, "ArUco timestamp")
        if self.latest_timestamp is not None and timestamp <= self.latest_timestamp:
            raise ValueError("ArUco timestamp must increase")
        pose = Pose2D(*map(float, (pose.x, pose.y, pose.yaw)))
        if self.aruco_origin is None:
            self.aruco_origin = pose
        self.latest_aruco = pose
        self.latest_timestamp = timestamp

    def compare(self, odom_pose, timestamp):
        timestamp = _finite(timestamp, "odometry timestamp")
        if self.latest_aruco is None:
            return None
        relative = compose(inverse(self.aruco_origin), self.latest_aruco)
        dx = odom_pose.x - relative.x
        dy = odom_pose.y - relative.y
        return PoseComparison(
            relative,
            math.hypot(dx, dy),
            normalize_angle(odom_pose.yaw - relative.yaw),
            timestamp - self.latest_timestamp,
        )


def compose(first, second):
    """Compose 2D transforms: result = first * second."""
    cosine, sine = math.cos(first.yaw), math.sin(first.yaw)
    return Pose2D(
        first.x + cosine * second.x - sine * second.y,
        first.y + sine * second.x + cosine * second.y,
        normalize_angle(first.yaw + second.yaw),
    )


def inverse(pose):
    cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
    return Pose2D(
        -cosine * pose.x - sine * pose.y,
        sine * pose.x - cosine * pose.y,
        normalize_angle(-pose.yaw),
    )


class PoseFusion:
    """Fuse map-frame ArUco fixes with odom-frame wheel predictions.

    Correction updates the rigid map<-odom transform. It never averages map
    coordinates directly with odom coordinates.
    """

    def __init__(
        self,
        *,
        correction_gain=0.25,
        max_position_innovation_m=0.5,
        max_yaw_innovation_rad=0.8,
        max_aruco_age=0.5,
        max_odom_age=0.5,
        max_dead_reckon_age=2.0,
    ):
        values = (
            correction_gain,
            max_position_innovation_m,
            max_yaw_innovation_rad,
            max_aruco_age,
            max_odom_age,
            max_dead_reckon_age,
        )
        if any(not math.isfinite(float(value)) or float(value) <= 0 for value in values):
            raise ValueError("fusion parameters must be finite and positive")
        if correction_gain > 1.0 or max_dead_reckon_age < max_aruco_age:
            raise ValueError("invalid fusion gain or dead-reckoning age")
        self.gain = float(correction_gain)
        self.max_position_innovation = float(max_position_innovation_m)
        self.max_yaw_innovation = float(max_yaw_innovation_rad)
        self.max_aruco_age = float(max_aruco_age)
        self.max_odom_age = float(max_odom_age)
        self.max_dead_reckon_age = float(max_dead_reckon_age)
        self.map_from_odom = None
        self.odom_pose = None
        self.odom_timestamp = None
        self.aruco_timestamp = None
        self.last_rejection = None

    def observe_odom(self, pose, timestamp, frame_id="odom"):
        if frame_id != "odom":
            raise ValueError("wheel odometry frame must be odom")
        timestamp = _finite(timestamp, "odom timestamp")
        if self.odom_timestamp is not None and timestamp <= self.odom_timestamp:
            raise ValueError("odom timestamp must increase")
        self.odom_pose = Pose2D(*map(float, (pose.x, pose.y, pose.yaw)))
        self.odom_timestamp = timestamp

    def observe_aruco(self, pose, timestamp, now, frame_id="map"):
        if frame_id != "map":
            return False, "frame_mismatch"
        timestamp = _finite(timestamp, "aruco timestamp")
        now = _finite(now, "now")
        if now - timestamp < -0.1 or now - timestamp > self.max_aruco_age:
            return False, "stale_aruco"
        if self.aruco_timestamp is not None and timestamp <= self.aruco_timestamp:
            return False, "non_monotonic_aruco"
        if self.odom_pose is None or now - self.odom_timestamp > self.max_odom_age:
            return False, "odom_unavailable_or_stale"
        measurement = Pose2D(*map(float, (pose.x, pose.y, pose.yaw)))
        target_transform = compose(measurement, inverse(self.odom_pose))
        initializing = self.map_from_odom is None
        if self.map_from_odom is None:
            self.map_from_odom = target_transform
        else:
            prediction = compose(self.map_from_odom, self.odom_pose)
            position_error = math.hypot(
                measurement.x - prediction.x, measurement.y - prediction.y
            )
            yaw_error = normalize_angle(measurement.yaw - prediction.yaw)
            if position_error > self.max_position_innovation:
                self.last_rejection = "position_outlier"
                return False, self.last_rejection
            if abs(yaw_error) > self.max_yaw_innovation:
                self.last_rejection = "yaw_outlier"
                return False, self.last_rejection
            transform_yaw_error = normalize_angle(
                target_transform.yaw - self.map_from_odom.yaw
            )
            self.map_from_odom = Pose2D(
                self.map_from_odom.x + self.gain * (target_transform.x - self.map_from_odom.x),
                self.map_from_odom.y + self.gain * (target_transform.y - self.map_from_odom.y),
                normalize_angle(self.map_from_odom.yaw + self.gain * transform_yaw_error),
            )
        self.aruco_timestamp = timestamp
        self.last_rejection = None
        return True, "initialized" if initializing else "corrected"

    def estimate(self, now):
        now = _finite(now, "now")
        if self.map_from_odom is None:
            return None, "UNINITIALIZED", 0.0
        if self.odom_pose is None or now - self.odom_timestamp > self.max_odom_age:
            return None, "STALE_ODOM", 0.0
        aruco_age = now - self.aruco_timestamp
        if aruco_age > self.max_dead_reckon_age:
            return None, "LOST", 0.0
        pose = compose(self.map_from_odom, self.odom_pose)
        if aruco_age <= self.max_aruco_age:
            return pose, "FUSED", 1.0
        confidence = max(
            0.0,
            1.0 - (aruco_age - self.max_aruco_age)
            / (self.max_dead_reckon_age - self.max_aruco_age),
        )
        return pose, "ODOM_ONLY", confidence

import math


def require_fresh_timestamp(timestamp, now, max_age):
    timestamp = float(timestamp)
    age = now - timestamp
    if not math.isfinite(timestamp) or age > max_age:
        raise ValueError(
            f"stale detection timestamp: age={age:.3f}s, limit={max_age:.3f}s"
        )
    return timestamp


def select_best_detection(detections, target_label, min_confidence):
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError("min_detection_confidence must be between 0 and 1")
    matches = [
        item
        for item in detections
        if str(item.get("label", "")).lower() == target_label.lower()
        and float(item.get("confidence", 0.0)) >= min_confidence
    ]
    return max(
        matches,
        key=lambda item: float(item.get("confidence", 0.0)),
        default=None,
    )


def evaluate_robot_guard(target, robot_position, radius):
    """Return an acceptance decision for a target in the robot pose frame."""
    radius = float(radius)
    if radius < 0.0 or not math.isfinite(radius):
        raise ValueError("robot_guard_radius must be finite and non-negative")
    distance = math.hypot(
        float(target[0]) - float(robot_position[0]),
        float(target[1]) - float(robot_position[1]),
    )
    return {
        "accepted": distance > radius,
        "reason": None if distance > radius else "robot_proximity",
        "distance": distance,
    }


def format_robot_proximity_log(frame, target, confidence, distance, timestamp):
    return (
        "Detection rejected: reason=robot_proximity "
        f"target_{frame}=({target[0]:.3f}, {target[1]:.3f}) "
        f"confidence={confidence:.3f} distance={distance:.3f}m "
        f"timestamp={timestamp:.6f}"
    )


class SafeTargetManager:
    """Keep a stable target identity across detector jitter and dropouts."""

    def __init__(self):
        self.reference_target = None
        self.reached = False
        self.suspended = False

    def consider(self, target, change_distance):
        if change_distance < 0.0 or not math.isfinite(change_distance):
            raise ValueError("target_change_distance must be finite and non-negative")

        if self.reference_target is None:
            self.reference_target = target
            self.reached = False
            self.suspended = False
            return "new"

        distance = math.hypot(
            target[0] - self.reference_target[0],
            target[1] - self.reference_target[1],
        )
        if distance <= change_distance:
            if self.reached or self.suspended:
                return "blocked_same"
            return "same"

        self.reference_target = target
        self.reached = False
        self.suspended = False
        return "new"

    def mark_reached(self):
        self.reached = True

    def suspend(self):
        self.suspended = True

    def reset(self):
        self.reference_target = None
        self.reached = False
        self.suspended = False

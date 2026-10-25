#!/usr/bin/env python3
"""Replay a recorded Create3 pose trajectory in Gazebo for visualization only."""

import argparse
import bisect
import json
import math
import subprocess
import time
from pathlib import Path


class GazeboPoseTimeout(RuntimeError):
    """Gazebo did not acknowledge a visualization pose update in time."""


class RvizReplayPublisher:
    """Publish a replay-only robot marker without replacing simulator TF/odom."""

    def __init__(self, samples, target=None, frame_id="odom"):
        import rclpy
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        self.rclpy = rclpy
        self.Point = Point
        self.Marker = Marker
        rclpy.init(args=None)
        self.node = rclpy.create_node("create3_trajectory_replay")
        self.publisher = self.node.create_publisher(
            Marker, "/pai/presentation_markers", 10
        )
        self.frame_id = frame_id
        self.target = target
        self.path_points = [Point(x=x, y=y, z=0.025) for _, x, y, _ in samples]

        deadline = time.monotonic() + 2.0
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _marker(self, marker_id, marker_type):
        marker = self.Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        # Reuse the namespace already present in the shipped RViz config.  The
        # replay IDs are kept above 100 to avoid the live overlay's IDs 0-4.
        marker.ns = "pai_demo"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = self.Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 0.95
        return marker

    def publish(self, pose):
        x, y, yaw = pose

        path = self._marker(100, self.Marker.LINE_STRIP)
        path.scale.x = 0.025
        path.color.r, path.color.g, path.color.b = 0.05, 0.75, 1.0
        path.points = self.path_points
        self.publisher.publish(path)

        body = self._marker(101, self.Marker.CYLINDER)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.055
        body.scale.x, body.scale.y, body.scale.z = 0.34, 0.34, 0.11
        body.color.r, body.color.g, body.color.b = 0.02, 0.42, 0.72
        self.publisher.publish(body)

        heading = self._marker(102, self.Marker.ARROW)
        heading.pose.position.x, heading.pose.position.y, heading.pose.position.z = x, y, 0.13
        heading.pose.orientation.z = math.sin(yaw / 2.0)
        heading.pose.orientation.w = math.cos(yaw / 2.0)
        heading.scale.x, heading.scale.y, heading.scale.z = 0.28, 0.045, 0.045
        heading.color.r, heading.color.g, heading.color.b = 1.0, 0.85, 0.05
        self.publisher.publish(heading)

        if self.target is not None:
            target = self._marker(103, self.Marker.CYLINDER)
            target.pose.position.x = self.target[0]
            target.pose.position.y = self.target[1]
            target.pose.position.z = 0.04
            target.scale.x, target.scale.y, target.scale.z = 0.18, 0.18, 0.08
            target.color.r, target.color.g, target.color.b = 1.0, 0.28, 0.03
            self.publisher.publish(target)

        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def load_passed_trajectory(path):
    payload = json.loads(Path(path).read_text())
    if payload.get("status") != "passed":
        raise ValueError("only a result with status=passed can be replayed")
    raw = payload.get("trajectory")
    if not isinstance(raw, list) or len(raw) < 2:
        raise ValueError("trajectory must contain at least two samples")

    samples = []
    previous_time = None
    for index, item in enumerate(raw):
        try:
            sample = tuple(float(item[key]) for key in ("time_s", "x", "y", "theta"))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"invalid trajectory sample {index}") from error
        if not all(math.isfinite(value) for value in sample):
            raise ValueError(f"non-finite trajectory sample {index}")
        if previous_time is not None and sample[0] <= previous_time:
            raise ValueError("trajectory timestamps must be strictly increasing")
        samples.append(sample)
        previous_time = sample[0]
    return payload, samples


def interpolate(samples, elapsed_original_s):
    first_time = samples[0][0]
    target_time = first_time + elapsed_original_s
    if target_time <= first_time:
        return samples[0][1:]
    if target_time >= samples[-1][0]:
        return samples[-1][1:]

    times = [sample[0] for sample in samples]
    right_index = bisect.bisect_right(times, target_time)
    left = samples[right_index - 1]
    right = samples[right_index]
    ratio = (target_time - left[0]) / (right[0] - left[0])
    x = left[1] + ratio * (right[1] - left[1])
    y = left[2] + ratio * (right[2] - left[2])
    yaw_delta = math.atan2(math.sin(right[3] - left[3]), math.cos(right[3] - left[3]))
    yaw = math.atan2(
        math.sin(left[3] + ratio * yaw_delta),
        math.cos(left[3] + ratio * yaw_delta),
    )
    return x, y, yaw


def pose_request(model, x, y, yaw):
    return (
        f'name: "{model}", position: {{x: {x:.9f}, y: {y:.9f}, z: 0.0}}, '
        f'orientation: {{z: {math.sin(yaw / 2.0):.9f}, '
        f'w: {math.cos(yaw / 2.0):.9f}}}'
    )


def set_pose(world, model, pose, timeout_ms=1000):
    command = [
        "gz",
        "service",
        "-s",
        f"/world/{world}/set_pose",
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        str(timeout_ms),
        "--req",
        pose_request(model, *pose),
    ]
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=(timeout_ms / 1000.0) + 1.0,
        )
    except subprocess.TimeoutExpired as error:
        raise GazeboPoseTimeout(
            f"Gazebo set_pose timed out after {error.timeout:.1f}s"
        ) from error
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(f"Gazebo set_pose failed: {detail or completed.returncode}")


def replay(samples, world, model, speed, fps, dry_run=False, pose_callback=None):
    original_duration = samples[-1][0] - samples[0][0]
    playback_duration = original_duration / speed
    frame_count = max(2, math.ceil(playback_duration * fps) + 1)
    if dry_run:
        for frame_index in range(frame_count):
            playback_elapsed = min(frame_index / fps, playback_duration)
            pose = interpolate(samples, playback_elapsed * speed)
            if pose_callback is not None:
                pose_callback(pose)
    else:
        start = time.monotonic()
        next_frame = 0
        consecutive_timeouts = 0
        while True:
            playback_elapsed = min(time.monotonic() - start, playback_duration)
            pose = interpolate(samples, playback_elapsed * speed)
            if pose_callback is not None:
                pose_callback(pose)
            try:
                set_pose(world, model, pose)
                consecutive_timeouts = 0
            except GazeboPoseTimeout as error:
                consecutive_timeouts += 1
                print(
                    f"warning: {error}; skipped delayed visualization frame "
                    f"({consecutive_timeouts}/3)",
                    flush=True,
                )
                if consecutive_timeouts >= 3:
                    raise RuntimeError("Gazebo set_pose timed out three times consecutively") from error

            if playback_elapsed >= playback_duration:
                break
            next_frame += 1
            now = time.monotonic()
            next_frame = max(next_frame, math.floor((now - start) * fps) + 1)
            delay = (start + next_frame / fps) - now
            if delay > 0.0:
                time.sleep(delay)

        # The loop already sends the exact final interpolated pose. Do not issue a
        # redundant service call, which previously caused avoidable timeout load.
    if pose_callback is not None:
        pose_callback(samples[-1][1:])
    return original_duration, playback_duration, frame_count


def main():
    parser = argparse.ArgumentParser(
        description="Visually replay a saved Create3 trajectory; no velocity is published."
    )
    parser.add_argument("result_json")
    parser.add_argument("--world", default="room_camera")
    parser.add_argument("--model", default="create3")
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--rviz-frame", default="odom")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or args.speed <= 0.0 or args.speed > 20.0:
        parser.error("--speed must be > 0 and <= 20")
    if not math.isfinite(args.fps) or args.fps < 1.0 or args.fps > 30.0:
        parser.error("--fps must be from 1 to 30")

    payload, samples = load_passed_trajectory(args.result_json)
    rviz = None
    if not args.dry_run and not args.no_rviz:
        target = payload.get("target_map_m")
        if not (
            isinstance(target, list)
            and len(target) == 2
            and all(isinstance(value, (int, float)) and math.isfinite(value) for value in target)
        ):
            target = None
        rviz = RvizReplayPublisher(samples, target, args.rviz_frame)
    try:
        original, playback, frames = replay(
            samples,
            args.world,
            args.model,
            args.speed,
            args.fps,
            args.dry_run,
            None if rviz is None else rviz.publish,
        )
    finally:
        if rviz is not None:
            rviz.close()
    print(
        "trajectory replay complete: trial_id={} samples={} original={:.3f}s "
        "playback={:.3f}s frames={} speed={}x visualization_only=true".format(
            payload.get("trial_id", "unknown"),
            len(samples),
            original,
            playback,
            frames,
            args.speed,
        )
    )


if __name__ == "__main__":
    main()

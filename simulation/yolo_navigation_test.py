#!/usr/bin/env python3
"""Monitor one explicit YOLO-driven Create3 integration run."""

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from irobot_create_msgs.msg import HazardDetectionVector
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

from known_target_test import spin_for, wait_until


RESULT = Path("/tmp/yolo_navigation_test_result.json")


def set_controller_parameters(executor, node, values, timeout=20.0):
    client = node.create_client(
        SetParameters, "/vision_turn_controller/set_parameters"
    )
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError("vision controller parameter service is unavailable")
    request = SetParameters.Request()
    request.parameters = [
        Parameter(name=name, value=value).to_parameter_msg()
        for name, value in values.items()
    ]
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError("vision controller parameter service timed out")
    response = future.result()
    failures = [
        result.reason or "rejected"
        for result in response.results
        if not result.successful
    ]
    if failures:
        raise RuntimeError(f"vision controller rejected parameters: {failures}")


class YoloNavigationMonitor(Node):
    def __init__(self):
        super().__init__("yolo_navigation_test")
        self.pose = None
        self.target = None
        self.command = (0.0, 0.0)
        self.reached = False
        self.reached_at = None
        self.last_detection = None
        self.hazard_count = 0
        self.post_reach_nonzero_commands = 0
        self.monitor_post_reach = False
        self.trajectory = []
        self.started_at = time.monotonic()
        self.stop_publisher = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.create_subscription(PoseStamped, "/robot/pose", self.on_pose, 10)
        self.create_subscription(PointStamped, "/litter/target", self.on_target, 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.on_command, 10)
        self.create_subscription(Bool, "/litter/reached", self.on_reached, 10)
        self.create_subscription(String, "/webcam/detections", self.on_detection, 10)
        self.create_subscription(
            HazardDetectionVector, "/hazard_detection", self.on_hazard, 10
        )

    def elapsed(self):
        return time.monotonic() - self.started_at

    def on_pose(self, message):
        pose = message.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
        )
        self.pose = (pose.position.x, pose.position.y, yaw)
        if not self.trajectory or self.elapsed() - self.trajectory[-1]["time_s"] >= 0.1:
            self.trajectory.append(
                {"time_s": self.elapsed(), "x": self.pose[0], "y": self.pose[1], "theta": yaw}
            )

    def on_target(self, message):
        self.target = (message.point.x, message.point.y)

    def on_command(self, message):
        self.command = (message.linear.x, message.angular.z)
        if self.monitor_post_reach and (
            abs(message.linear.x) >= 1e-4 or abs(message.angular.z) >= 1e-4
        ):
            self.post_reach_nonzero_commands += 1

    def on_reached(self, message):
        self.reached = bool(message.data)
        if self.reached and self.reached_at is None:
            self.reached_at = self.elapsed()

    def on_detection(self, message):
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            if detections:
                self.last_detection = max(
                    detections,
                    key=lambda item: float(item.get("confidence", 0.0)),
                )
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def on_hazard(self, message):
        self.hazard_count += len(message.detections)

    def publish_stop(self):
        for _ in range(3):
            self.stop_publisher.publish(Twist())
            time.sleep(0.05)


def main():
    result = {"status": "failed", "stop_reason": "setup failure"}
    rclpy.init()
    node = YoloNavigationMonitor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    enabled = False
    try:
        # Metric targets are intentionally suppressed until the validated
        # simulation calibration is enabled. Motion remains disabled here.
        set_controller_parameters(
            executor,
            node,
            {"control_enabled": False, "calibration_ready": True},
        )
        ready = wait_until(
            executor,
            lambda: node.pose is not None
            and node.target is not None
            and node.last_detection is not None,
            60.0,
        )
        if not ready:
            raise TimeoutError("pose, YOLO detection, or transformed target was not received")
        result.update(
            {
                "initial_pose": node.pose,
                "target_map_m": node.target,
                "detection": node.last_detection,
                "speed_limit_m_s": 0.08,
            }
        )

        set_controller_parameters(
            executor,
            node,
            {
                "control_enabled": False,
                "control_mode": "approach",
                "max_linear_speed": 0.08,
            },
        )
        set_controller_parameters(
            executor, node, {"control_enabled": True}
        )
        enabled = True

        if not wait_until(
            executor,
            lambda: abs(node.command[1]) > 0.05 or node.command[0] > 1e-5,
            20.0,
        ):
            raise RuntimeError("navigation did not produce a motion command")
        # Gazebo can run well below real time on the 8 GB submission machine,
        # especially while the fixed-camera recording is active.  This is a
        # wall-clock monitor limit only; navigation gains and speed stay fixed.
        if not wait_until(executor, lambda: node.reached, 360.0):
            raise TimeoutError("arrival was not reported")

        node.monitor_post_reach = True
        spin_for(executor, 3.2)
        final_pose = node.pose
        final_target = node.target
        final_error = math.hypot(
            final_target[0] - final_pose[0], final_target[1] - final_pose[1]
        )
        stopped = abs(node.command[0]) < 1e-4 and abs(node.command[1]) < 1e-4
        result.update(
            {
                "arrival_time_s": node.reached_at,
                "final_pose": final_pose,
                "final_target_map_m": final_target,
                "final_position_error_m": final_error,
                "stopped_at_arrival": stopped,
                "post_reach_nonzero_commands": node.post_reach_nonzero_commands,
                "hazard_count": node.hazard_count,
            }
        )
        if not stopped or node.post_reach_nonzero_commands or final_error > 0.18:
            raise RuntimeError("arrival stop, latch, or final position error failed")
        result["status"] = "passed"
        result["stop_reason"] = "YOLO target reached and arrival latch held for 3.2 s"
    except Exception as exc:
        result["error"] = str(exc)
        result["stop_reason"] = f"exception: {exc}"
    finally:
        try:
            set_controller_parameters(
                executor,
                node,
                {"control_enabled": False, "calibration_ready": False},
            )
            result["control_disabled_after_test"] = True
        except Exception as exc:
            result["control_disabled_after_test"] = False
            result["disable_error"] = str(exc)
        node.publish_stop()
        spin_for(executor, 0.2)
        result["control_was_enabled"] = enabled
        result["final_command"] = list(node.command)
        result["trajectory"] = node.trajectory
        RESULT.write_text(json.dumps(result, indent=2) + "\n")
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

    summary = {key: value for key, value in result.items() if key != "trajectory"}
    summary["trajectory_samples"] = len(result["trajectory"])
    print(json.dumps(summary, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Run an explicit, simulation-only known-coordinate navigation test."""

import argparse
import json
import math
import re
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


CONFIG = Path("/etc/known-target-test.json")
CONTROLLER_CONFIG = Path("/etc/vision-controller.sim-camera.yaml")
CALIBRATION_POINTS = Path("/etc/sim-camera-correspondences.json")
RESULT = Path("/tmp/known_target_test_result.json")


def config_value(text, name):
    match = re.search(rf"^\s*{re.escape(name)}:\s*(.+?)\s*$", text, re.MULTILINE)
    if not match:
        raise ValueError(f"missing {name} in simulated-camera controller config")
    return match.group(1)


def validate_safety_configuration(test_config, controller_text, points_payload):
    if config_value(controller_text, "control_enabled") != "false":
        raise ValueError("control_enabled must initially be false")
    if config_value(controller_text, "calibration_ready") != "false":
        raise ValueError("calibration_ready must initially be false")
    configured_speed = float(config_value(controller_text, "max_linear_speed"))
    allowed_speed = float(test_config["max_linear_speed_m_s"])
    if configured_speed > allowed_speed or allowed_speed > 0.08:
        raise ValueError("simulated linear speed limit exceeds 0.08 m/s")

    raw_homography = config_value(controller_text, "image_to_odom_homography")
    homography = [float(value) for value in raw_homography.strip("[]").split(",")]
    if len(homography) != 9:
        raise ValueError("simulated-camera homography must contain nine values")
    identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    if all(abs(actual - expected) <= 1e-9 for actual, expected in zip(homography, identity)):
        raise ValueError("identity homography is not accepted as calibrated")

    errors = []
    for pixel_x, pixel_y, expected_x, expected_y in points_payload["validation_points"]:
        denominator = homography[6] * pixel_x + homography[7] * pixel_y + homography[8]
        actual_x = (homography[0] * pixel_x + homography[1] * pixel_y + homography[2]) / denominator
        actual_y = (homography[3] * pixel_x + homography[4] * pixel_y + homography[5]) / denominator
        errors.append(math.hypot(actual_x - expected_x, actual_y - expected_y))
    maximum_error = max(errors)
    if maximum_error > float(test_config["calibration_max_error_m"]):
        raise ValueError(f"calibration error {maximum_error:.4f} m exceeds limit")
    return maximum_error


def set_controller_parameter(name, value):
    deadline = time.monotonic() + 60.0
    last_message = "no response"
    while time.monotonic() < deadline:
        try:
            completed = subprocess.run(
                ["ros2", "param", "set", "/vision_turn_controller", name, str(value)],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )
            last_message = f"{completed.stdout.strip()} {completed.stderr.strip()}".strip()
            if completed.returncode == 0 and "successful" in completed.stdout.lower():
                return
        except subprocess.TimeoutExpired:
            last_message = "parameter service timed out"
        time.sleep(1.0)
    raise RuntimeError(f"could not set {name}: {last_message}")


class KnownTargetTest(Node):
    def __init__(self):
        super().__init__("known_target_navigation_test")
        self.target_publisher = self.create_publisher(PointStamped, "/litter/target_odom", 10)
        self.stop_publisher = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.create_subscription(PoseStamped, "/robot/pose", self.on_pose, 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.on_command, 10)
        self.create_subscription(Bool, "/litter/reached", self.on_reached, 10)
        self.pose = None
        self.command = (0.0, 0.0)
        self.reached = False
        self.reached_at = None
        self.monitor_post_reach = False
        self.post_reach_nonzero_commands = 0
        self.trajectory = []
        self.events = []
        self.started_at = time.monotonic()

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

    def publish_target(self, target):
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.point.x, message.point.y = target
        self.target_publisher.publish(message)

    def publish_stop(self):
        for _ in range(3):
            self.stop_publisher.publish(Twist())
            time.sleep(0.05)


def spin_for(executor, seconds, callback=None, interval=0.05):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if callback:
            callback()
        executor.spin_once(timeout_sec=interval)


def wait_until(executor, predicate, timeout, callback=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if callback:
            callback()
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate-abort", action="store_true")
    args = parser.parse_args()
    test_config = json.loads(CONFIG.read_text())
    calibration_error = validate_safety_configuration(
        test_config,
        CONTROLLER_CONFIG.read_text(),
        json.loads(CALIBRATION_POINTS.read_text()),
    )
    result = {
        "status": "failed",
        "stop_reason": "setup failure",
        "calibration_validation_max_error_m": calibration_error,
        "speed_limit_m_s": test_config["max_linear_speed_m_s"],
        "target_input_topic": "/litter/target_odom",
        "target_output_topic": "/litter/target",
        "pose_topic": "/robot/pose",
    }

    rclpy.init()
    node = KnownTargetTest()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    control_was_enabled = False
    try:
        if not wait_until(executor, lambda: node.pose is not None, 60.0):
            raise TimeoutError("robot pose was not received")
        initial_pose = node.pose
        target = tuple(float(value) for value in test_config["target_odom_m"])
        result["initial_pose"] = initial_pose
        result["target_odom_m"] = target

        set_controller_parameter("control_enabled", "false")
        set_controller_parameter("calibration_ready", "true")
        set_controller_parameter("control_mode", "approach")
        set_controller_parameter("max_linear_speed", test_config["max_linear_speed_m_s"])
        set_controller_parameter("control_enabled", "true")
        control_was_enabled = True

        publish = lambda: node.publish_target(target)
        saw_turn = wait_until(
            executor,
            lambda: abs(node.command[1]) > 0.05 and abs(node.command[0]) < 0.005,
            8.0,
            publish,
        )
        result["saw_turn"] = saw_turn
        if not saw_turn:
            raise RuntimeError("turn phase was not observed")

        if args.simulate_abort:
            result["stop_reason"] = "simulated abnormal exit"
            raise RuntimeError("simulated abnormal exit")

        saw_forward = wait_until(
            executor,
            lambda: node.command[0] > 1e-5,
            45.0,
            publish,
        )
        result["saw_forward"] = saw_forward
        if not saw_forward:
            raise RuntimeError("forward phase was not observed")

        reached = wait_until(
            executor,
            lambda: node.reached,
            float(test_config["test_timeout_s"]),
            publish,
        )
        if not reached:
            raise TimeoutError("arrival was not reported")
        node.monitor_post_reach = True
        spin_for(executor, 1.0, publish)
        stopped_at_arrival = abs(node.command[0]) < 1e-4 and abs(node.command[1]) < 1e-4
        final_pose = node.pose
        final_error = math.hypot(target[0] - final_pose[0], target[1] - final_pose[1])
        result.update(
            {
                "saw_reached": True,
                "stopped_at_arrival": stopped_at_arrival,
                "final_pose": final_pose,
                "final_position_error_m": final_error,
                "arrival_time_s": node.reached_at,
                "post_reach_nonzero_commands": node.post_reach_nonzero_commands,
            }
        )
        if (
            not stopped_at_arrival
            or node.post_reach_nonzero_commands
            or final_error > test_config["arrival_tolerance_m"]
        ):
            raise RuntimeError("arrival stop or final position error failed")

        timeout_target = (final_pose[0], final_pose[1] + 0.5)
        publish_timeout_target = lambda: node.publish_target(timeout_target)
        became_active = wait_until(
            executor,
            lambda: abs(node.command[0]) > 0.005 or abs(node.command[1]) > 0.05,
            5.0,
            publish_timeout_target,
        )
        if not became_active:
            raise RuntimeError("timeout test could not activate motion")
        spin_for(executor, float(test_config["input_timeout_s"]) + 0.7)
        stopped_on_timeout = abs(node.command[0]) < 1e-4 and abs(node.command[1]) < 1e-4
        result["stopped_on_target_loss"] = stopped_on_timeout
        result["stopped_on_input_timeout"] = stopped_on_timeout
        if not stopped_on_timeout:
            raise RuntimeError("controller did not stop after target input timeout")

        result["status"] = "passed"
        result["stop_reason"] = "target reached; target-loss timeout stop verified"
    except Exception as exc:
        result["error"] = str(exc)
        if result["stop_reason"] == "setup failure":
            result["stop_reason"] = f"exception: {exc}"
    finally:
        disabled_after_test = False
        try:
            set_controller_parameter("control_enabled", "false")
            set_controller_parameter("calibration_ready", "false")
            disabled_after_test = True
        except Exception as exc:
            result["disable_error"] = str(exc)
        node.publish_stop()
        spin_for(executor, 0.2)
        result["control_was_enabled"] = control_was_enabled
        result["control_disabled_after_test"] = disabled_after_test
        result["final_command"] = [node.command[0], node.command[1]]
        if node.pose is not None:
            result.setdefault("final_pose", node.pose)
            if "target_odom_m" in result:
                target_x, target_y = result["target_odom_m"]
                result.setdefault(
                    "final_position_error_m",
                    math.hypot(target_x - node.pose[0], target_y - node.pose[1]),
                )
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

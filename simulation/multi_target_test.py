#!/usr/bin/env python3
"""Simulation-only two-target navigation and safety regression test."""

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from irobot_create_msgs.msg import HazardDetectionVector
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from known_target_test import (
    CALIBRATION_POINTS,
    CONTROLLER_CONFIG,
    config_value,
    set_controller_parameter,
    validate_safety_configuration,
)


CONFIG = Path("/etc/multi-target-test.json")
RESULT = Path("/tmp/multi_target_test_result.json")
REPORT = Path("/tmp/multi_target_test_report.md")
NONZERO_LINEAR = 1e-4
NONZERO_ANGULAR = 1e-4


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class MultiTargetTestNode(Node):
    def __init__(self):
        super().__init__("multi_target_navigation_test")
        self.target_publisher = self.create_publisher(PointStamped, "/litter/target_odom", 10)
        self.stop_publisher = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.create_subscription(PoseStamped, "/robot/pose", self.on_pose, 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.on_command, 10)
        self.create_subscription(Bool, "/litter/reached", self.on_reached, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.on_hazard,
            qos_profile_sensor_data,
        )
        self.started_at = time.monotonic()
        self.pose = None
        self.command = (0.0, 0.0)
        self.reached = False
        self.phase = "setup"
        self.command_events = []
        self.odom_events = []
        self.hazard_events = []
        self.latch_transitions = []
        self.monitor_post_reach = False
        self.post_reach_nonzero_commands = 0
        self.last_odom_recorded_at = -1.0

    def elapsed(self):
        return time.monotonic() - self.started_at

    def set_phase(self, phase):
        self.phase = phase

    def on_pose(self, message):
        pose = message.pose
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def on_command(self, message):
        linear = message.linear.x
        angular = message.angular.z
        self.command = (linear, angular)
        self.command_events.append(
            {
                "time_s": self.elapsed(),
                "phase": self.phase,
                "linear_x": linear,
                "angular_z": angular,
            }
        )
        if self.monitor_post_reach and (
            abs(linear) >= NONZERO_LINEAR or abs(angular) >= NONZERO_ANGULAR
        ):
            self.post_reach_nonzero_commands += 1

    def on_reached(self, message):
        value = bool(message.data)
        if value != self.reached:
            distance = None
            if self.pose is not None and self.current_target is not None:
                distance = math.hypot(
                    self.current_target[0] - self.pose[0],
                    self.current_target[1] - self.pose[1],
                )
            self.latch_transitions.append(
                {
                    "time_s": self.elapsed(),
                    "phase": self.phase,
                    "from": self.reached,
                    "to": value,
                    "distance_m": distance,
                }
            )
        self.reached = value

    @property
    def current_target(self):
        return getattr(self, "_current_target", None)

    def publish_target(self, target):
        self._current_target = target
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.point.x, message.point.y = target
        self.target_publisher.publish(message)

    def on_odom(self, message):
        now = self.elapsed()
        if now - self.last_odom_recorded_at < 0.1:
            return
        pose = message.pose.pose
        self.odom_events.append(
            {
                "time_s": now,
                "phase": self.phase,
                "x": pose.position.x,
                "y": pose.position.y,
                "theta": yaw_from_quaternion(pose.orientation),
                "linear_x": message.twist.twist.linear.x,
                "angular_z": message.twist.twist.angular.z,
            }
        )
        self.last_odom_recorded_at = now

    def on_hazard(self, message):
        if message.detections:
            self.hazard_events.append(
                {
                    "time_s": self.elapsed(),
                    "phase": self.phase,
                    "detections": [
                        {"type": int(item.type), "frame_id": item.header.frame_id}
                        for item in message.detections
                    ],
                }
            )

    def publish_stop(self):
        for _ in range(3):
            self.stop_publisher.publish(Twist())
            time.sleep(0.05)


def spin_for(executor, seconds, callback=None, interval=0.05):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if callback is not None:
            callback()
        executor.spin_once(timeout_sec=interval)


def wait_until(executor, predicate, timeout, callback=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if callback is not None:
            callback()
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def command_is_nonzero(command):
    return abs(command[0]) >= NONZERO_LINEAR or abs(command[1]) >= NONZERO_ANGULAR


def run_goal(node, executor, target_spec, config, require_latch_release):
    name = target_spec["name"]
    target = (float(target_spec["x"]), float(target_spec["y"]))
    node.set_phase("target_{}_switch".format(name))
    started_at = node.elapsed()
    start_pose = node.pose
    transition_start = len(node.latch_transitions)
    command_start = len(node.command_events)
    publish = lambda: node.publish_target(target)

    publish()
    stale_nonzero_before_release = 0
    if require_latch_release:
        released = wait_until(executor, lambda: not node.reached, 3.0, publish)
        if not released:
            raise RuntimeError("target {} did not release the arrival latch".format(name))
        release_events = [
            event for event in node.latch_transitions[transition_start:]
            if not event["to"]
        ]
        release_time = release_events[0]["time_s"]
        stale_nonzero_before_release = sum(
            1 for event in node.command_events[command_start:]
            if event["time_s"] <= release_time
            and (
                abs(event["linear_x"]) >= NONZERO_LINEAR
                or abs(event["angular_z"]) >= NONZERO_ANGULAR
            )
        )
        if stale_nonzero_before_release:
            raise RuntimeError("target {} retained a stale motion command".format(name))

    node.set_phase("target_{}_motion".format(name))
    saw_turn = wait_until(
        executor,
        lambda: abs(node.command[1]) > 0.05 and abs(node.command[0]) < 0.005,
        12.0,
        publish,
    )
    if not saw_turn:
        raise RuntimeError("target {} turn phase was not observed".format(name))
    saw_forward = wait_until(
        executor,
        lambda: node.command[0] > 1e-5,
        45.0,
        publish,
    )
    if not saw_forward:
        raise RuntimeError("target {} forward phase was not observed".format(name))
    arrived = wait_until(
        executor,
        lambda: node.reached,
        float(config["test_timeout_s"]),
        publish,
    )
    if not arrived:
        raise TimeoutError("target {} arrival was not reported".format(name))

    reached_at = node.elapsed()
    arrival_pose = node.pose
    arrival_distance = math.hypot(target[0] - arrival_pose[0], target[1] - arrival_pose[1])
    node.set_phase("target_{}_hold".format(name))
    node.monitor_post_reach = True
    node.post_reach_nonzero_commands = 0
    spin_for(executor, float(config["post_arrival_hold_s"]), publish)
    node.monitor_post_reach = False
    post_reach_nonzero = node.post_reach_nonzero_commands
    final_pose = node.pose
    final_error = math.hypot(target[0] - final_pose[0], target[1] - final_pose[1])
    same_target_latch_held = node.reached
    stopped = not command_is_nonzero(node.command)

    if not same_target_latch_held:
        raise RuntimeError("target {} same-coordinate republish released latch".format(name))
    if not stopped or post_reach_nonzero:
        raise RuntimeError("target {} emitted motion after arrival".format(name))
    if final_error > float(config["arrival_tolerance_m"]):
        raise RuntimeError("target {} final position error exceeded tolerance".format(name))

    commands = node.command_events[command_start:]
    reverse_count = sum(1 for item in commands if item["linear_x"] < -0.005)
    return {
        "name": name,
        "target_odom_m": list(target),
        "start_time_s": started_at,
        "arrival_time_s": reached_at,
        "elapsed_to_arrival_s": reached_at - started_at,
        "start_pose": start_pose,
        "arrival_pose": arrival_pose,
        "final_pose_after_hold": final_pose,
        "arrival_distance_m": arrival_distance,
        "final_position_error_m": final_error,
        "saw_turn": saw_turn,
        "saw_forward": saw_forward,
        "same_target_latch_held": same_target_latch_held,
        "post_arrival_hold_s": config["post_arrival_hold_s"],
        "post_reach_nonzero_commands": post_reach_nonzero,
        "stale_nonzero_commands_before_latch_release": stale_nonzero_before_release,
        "reverse_command_count": reverse_count,
        "latch_transitions": node.latch_transitions[transition_start:],
        "stop_reason": "arrival latch",
    }


def write_report(result):
    goals = result.get("goals", [])
    lines = [
        "# 第4段階B 複数目標試験",
        "",
        "- 結果: `{}`".format(result["status"]),
        "- 停止理由: {}".format(result["stop_reason"]),
        "- hazard発生回数: {}".format(result.get("hazard_count", 0)),
        "- 意図しない後退指令: {}".format(result.get("reverse_command_count", 0)),
        "- B切替でラッチ解除: `{}`".format(
            result.get("new_target_released_latch", False)
        ),
        "- 目標喪失・入力タイムアウト停止: `{}` / `{}`".format(
            result.get("stopped_on_target_loss", False),
            result.get("stopped_on_input_timeout", False),
        ),
        "- 終了後制御無効: `{}`".format(result.get("control_disabled_after_test", False)),
        "",
        "| 目標 | 座標 (odom) | 到達時間 | 到達距離 | 最終誤差 | 到達後非ゼロ | 切替前の古い非ゼロ |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for goal in goals:
        lines.append(
            "| {name} | ({x:.3f}, {y:.3f}) | {elapsed:.3f}s | {distance:.6f}m | "
            "{error:.6f}m | {commands} | {stale} |".format(
                name=goal["name"],
                x=goal["target_odom_m"][0],
                y=goal["target_odom_m"][1],
                elapsed=goal["elapsed_to_arrival_s"],
                distance=goal["arrival_distance_m"],
                error=goal["final_position_error_m"],
                commands=goal["post_reach_nonzero_commands"],
                stale=goal["stale_nonzero_commands_before_latch_release"],
            )
        )
    REPORT.write_text("\n".join(lines) + "\n")


def main():
    config = json.loads(CONFIG.read_text())
    calibration_error = validate_safety_configuration(
        config,
        CONTROLLER_CONFIG.read_text(),
        json.loads(CALIBRATION_POINTS.read_text()),
    )
    arrival_epsilon = float(
        config_value(CONTROLLER_CONFIG.read_text(), "arrival_epsilon")
    )
    if abs(arrival_epsilon - 0.001) > 1e-12:
        raise ValueError("multi-target test requires arrival_epsilon=0.001")
    result = {
        "status": "failed",
        "stop_reason": "setup failure",
        "calibration_validation_max_error_m": calibration_error,
        "speed_limit_m_s": config["max_linear_speed_m_s"],
        "arrival_epsilon_m": arrival_epsilon,
        "target_input_topic": "/litter/target_odom",
        "target_output_topic": "/litter/target",
        "pose_topic": "/robot/pose",
        "goals": [],
    }

    rclpy.init()
    node = MultiTargetTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    control_was_enabled = False
    try:
        if not wait_until(executor, lambda: node.pose is not None, 60.0):
            raise TimeoutError("robot pose was not received")
        result["initial_pose"] = node.pose
        result["minimum_static_obstacle_clearance_m"] = config[
            "minimum_static_obstacle_clearance_m"
        ]
        result["topic_graph"] = {
            topic: {
                "publishers": [
                    {"node": info.node_name, "type": info.topic_type}
                    for info in node.get_publishers_info_by_topic(topic)
                ],
                "subscriptions": [
                    {"node": info.node_name, "type": info.topic_type}
                    for info in node.get_subscriptions_info_by_topic(topic)
                ],
            }
            for topic in ("/litter/target_odom", "/litter/target", "/robot/pose")
        }

        set_controller_parameter("control_enabled", "false")
        set_controller_parameter("calibration_ready", "true")
        set_controller_parameter("control_mode", "approach")
        set_controller_parameter("max_linear_speed", config["max_linear_speed_m_s"])
        set_controller_parameter("control_enabled", "true")
        control_was_enabled = True

        targets = config["targets_odom_m"]
        result["goals"].append(run_goal(node, executor, targets[0], config, False))
        result["goals"].append(run_goal(node, executor, targets[1], config, True))

        switch_events = result["goals"][1]["latch_transitions"]
        released_for_b = any(not event["to"] for event in switch_events)
        relatched_for_b = any(event["to"] for event in switch_events)
        result["new_target_released_latch"] = released_for_b
        result["new_target_relatched_on_arrival"] = relatched_for_b
        if not released_for_b or not relatched_for_b:
            raise RuntimeError("target B latch transition sequence failed")

        node.set_phase("target_loss_motion")
        pose = node.pose
        timeout_target = (pose[0], pose[1] + 0.5)
        publish_timeout_target = lambda: node.publish_target(timeout_target)
        activated = wait_until(
            executor,
            lambda: command_is_nonzero(node.command),
            8.0,
            publish_timeout_target,
        )
        if not activated:
            raise RuntimeError("target-loss test could not activate motion")
        node.set_phase("target_loss_wait")
        spin_for(executor, float(config["input_timeout_s"]) + 0.8)
        stopped_on_timeout = not command_is_nonzero(node.command)
        result["stopped_on_target_loss"] = stopped_on_timeout
        result["stopped_on_input_timeout"] = stopped_on_timeout
        if not stopped_on_timeout:
            raise RuntimeError("controller did not stop after target input timeout")

        result["hazard_count"] = len(node.hazard_events)
        result["reverse_command_count"] = sum(
            1 for event in node.command_events if event["linear_x"] < -0.005
        )
        result["odom_reverse_count"] = sum(
            1 for event in node.odom_events if event["linear_x"] < -0.005
        )
        if result["hazard_count"]:
            raise RuntimeError("hazard was detected")
        if result["reverse_command_count"] or result["odom_reverse_count"]:
            raise RuntimeError("unexpected reverse motion was detected")

        result["status"] = "passed"
        result["stop_reason"] = "targets A and B reached; target-loss timeout stop verified"
    except Exception as error:
        result["error"] = str(error)
        result["stop_reason"] = "exception: {}".format(error)
    finally:
        disabled_after_test = False
        try:
            set_controller_parameter("control_enabled", "false")
            set_controller_parameter("calibration_ready", "false")
            disabled_after_test = True
        except Exception as error:
            result["disable_error"] = str(error)
        node.set_phase("cleanup")
        node.publish_stop()
        spin_for(executor, 0.2)
        result["control_was_enabled"] = control_was_enabled
        result["control_disabled_after_test"] = disabled_after_test
        result["final_command"] = list(node.command)
        result["latch_transitions"] = node.latch_transitions
        result["command_events"] = node.command_events
        result["odom_events"] = node.odom_events
        result["hazard_events"] = node.hazard_events
        RESULT.write_text(json.dumps(result, indent=2) + "\n")
        write_report(result)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    summary = {key: value for key, value in result.items()
               if key not in ("command_events", "odom_events", "hazard_events")}
    summary["command_samples"] = len(result["command_events"])
    summary["odom_samples"] = len(result["odom_events"])
    print(json.dumps(summary, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

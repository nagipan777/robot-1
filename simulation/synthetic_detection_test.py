#!/usr/bin/env python3
"""Non-driving HTTP-to-ROS coordinate integration check for room_camera."""

import json
import math
import time
import urllib.request
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


RESULT = Path("/tmp/synthetic_detection_test_result.json")
HTTP_URL = "http://127.0.0.1:8765/detections"
PIXEL = (279.4, 239.5)
EXPECTED_ODOM = (-0.000000000004, 0.500555)
POSITION_TOLERANCE_M = 0.01
POSE_TOLERANCE_M = 0.03
YAW_TOLERANCE_RAD = 0.03


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def post_detection(timestamp, pixel=PIXEL):
    payload = {
        "test_data": True,
        "test_source": "synthetic-coordinate-validation",
        "timestamp": timestamp,
        "image_width": 640,
        "image_height": 480,
        "detections": [
            {
                "label": "litter",
                "confidence": 0.99,
                "center_x": pixel[0],
                "center_y": pixel[1],
                "width": 20.0,
                "height": 20.0,
            }
        ],
    }
    request = urllib.request.Request(
        HTTP_URL,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        if response.status != 204:
            raise RuntimeError(f"HTTP bridge returned {response.status}")
    return payload


class CoordinateMonitor(Node):
    def __init__(self):
        super().__init__("synthetic_detection_coordinate_test")
        self.detections = []
        self.targets_odom = []
        self.targets_map = []
        self.odom_pose = None
        self.map_pose = None
        self.nonzero_commands = []
        self.create_subscription(String, "/webcam/detections", self.on_detection, 10)
        self.create_subscription(
            PointStamped, "/litter/target_odom", self.on_target_odom, 10
        )
        self.create_subscription(PointStamped, "/litter/target", self.on_target_map, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/robot/pose", self.on_pose, 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.on_command, 10)

    def on_detection(self, message):
        self.detections.append(json.loads(message.data))

    def on_target_odom(self, message):
        self.targets_odom.append(
            (message.point.x, message.point.y, message.header.frame_id)
        )

    def on_target_map(self, message):
        self.targets_map.append(
            (message.point.x, message.point.y, message.header.frame_id)
        )

    def on_odom(self, message):
        pose = message.pose.pose
        self.odom_pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
            message.header.frame_id,
        )

    def on_pose(self, message):
        self.map_pose = (
            message.pose.position.x,
            message.pose.position.y,
            yaw_from_quaternion(message.pose.orientation),
            message.header.frame_id,
        )

    def on_command(self, message):
        if abs(message.linear.x) > 1e-9 or abs(message.angular.z) > 1e-9:
            self.nonzero_commands.append((message.linear.x, message.angular.z))

    def publishers(self, topic):
        return sorted(info.node_name for info in self.get_publishers_info_by_topic(topic))


def spin_until(executor, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def read_safety_flags(node, executor):
    client = node.create_client(
        GetParameters, "/vision_turn_controller/get_parameters"
    )
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError("vision controller parameter service is unavailable")
    request = GetParameters.Request()
    request.names = ["control_enabled", "calibration_ready"]
    future = client.call_async(request)
    if not spin_until(executor, future.done, 30.0):
        raise RuntimeError("vision controller parameter request timed out")
    response = future.result()
    if response is None or len(response.values) != 2:
        raise RuntimeError("vision controller returned invalid parameters")
    return tuple(value.bool_value for value in response.values)


def set_calibration_ready(node, executor, enabled):
    client = node.create_client(
        SetParameters, "/vision_turn_controller/set_parameters"
    )
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError("vision controller parameter service is unavailable")
    request = SetParameters.Request()
    request.parameters = [
        Parameter(
            name="calibration_ready",
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=enabled
            ),
        )
    ]
    future = client.call_async(request)
    if not spin_until(executor, future.done, 30.0):
        raise RuntimeError("calibration parameter update timed out")
    response = future.result()
    if response is None or not response.results or not response.results[0].successful:
        raise RuntimeError("calibration parameter update was rejected")


def pose_near_origin(pose):
    return (
        pose is not None
        and math.hypot(pose[0], pose[1]) <= POSE_TOLERANCE_M
        and abs(pose[2]) <= YAW_TOLERANCE_RAD
    )


def main():
    result = {
        "status": "failed",
        "test_data": True,
        "http_url": HTTP_URL,
        "input_pixel": list(PIXEL),
        "expected_odom_m": list(EXPECTED_ODOM),
    }
    rclpy.init()
    node = CoordinateMonitor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    calibration_enabled_for_test = False
    try:
        control_enabled, calibration_ready = read_safety_flags(node, executor)
        result["control_enabled"] = control_enabled
        result["calibration_ready"] = calibration_ready
        if control_enabled or calibration_ready:
            raise RuntimeError("control and calibration flags must both remain false")

        if not spin_until(
            executor,
            lambda: node.odom_pose is not None and node.map_pose is not None,
            30.0,
        ):
            raise TimeoutError("/odom or /robot/pose was not received")
        result["initial_odom_pose"] = node.odom_pose
        result["initial_map_pose"] = node.map_pose
        if not pose_near_origin(node.odom_pose) or not pose_near_origin(node.map_pose):
            raise RuntimeError("odom/map initial pose is not aligned with world origin")

        graph = {
            topic: node.publishers(topic)
            for topic in (
                "/litter/target_odom",
                "/litter/target",
                "/roomba/cmd_vel_unstamped",
            )
        }
        result["publishers"] = graph
        if len(graph["/litter/target_odom"]) != 1:
            raise RuntimeError("/litter/target_odom has competing publishers")
        if len(graph["/litter/target"]) != 1:
            raise RuntimeError("/litter/target has competing publishers")
        if graph["/roomba/cmd_vel_unstamped"]:
            raise RuntimeError("real-Roomba command topic has a publisher")

        # The integration homography has independent-point validation. Enable
        # only that calibration gate; motion control remains disabled.
        set_calibration_ready(node, executor, True)
        calibration_enabled_for_test = True

        before_odom = len(node.targets_odom)
        before_map = len(node.targets_map)
        stale_payload = post_detection(time.time() - 60.0, pixel=(319.5, 239.5))
        if not spin_until(executor, lambda: len(node.detections) >= 1, 3.0):
            raise TimeoutError("stale test JSON did not reach /webcam/detections")
        stale_deadline = time.monotonic() + 1.2
        while time.monotonic() < stale_deadline:
            executor.spin_once(timeout_sec=0.05)
        stale_rejected = (
            len(node.targets_odom) == before_odom and len(node.targets_map) == before_map
        )
        result["stale_input"] = stale_payload
        result["stale_detection_rejected"] = stale_rejected
        if not stale_rejected:
            raise RuntimeError("stale detection produced a target")

        valid_payload = post_detection(time.time())
        if not spin_until(
            executor,
            lambda: len(node.targets_odom) > before_odom
            and len(node.targets_map) > before_map,
            5.0,
        ):
            raise TimeoutError("valid detection did not traverse both target topics")
        target_odom = node.targets_odom[-1]
        target_map = node.targets_map[-1]
        result["valid_input"] = valid_payload
        result["target_odom"] = target_odom
        result["target_map"] = target_map
        odom_error = math.hypot(
            target_odom[0] - EXPECTED_ODOM[0], target_odom[1] - EXPECTED_ODOM[1]
        )
        map_odom_error = math.hypot(
            target_map[0] - target_odom[0], target_map[1] - target_odom[1]
        )
        result["target_error_m"] = odom_error
        result["map_odom_error_m"] = map_odom_error
        if target_odom[2] != "odom" or target_map[2] != "map":
            raise RuntimeError("target frame IDs are incorrect")
        if odom_error > POSITION_TOLERANCE_M or map_odom_error > 1e-6:
            raise RuntimeError("image, odom, and map coordinates are not aligned")

        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        result["nonzero_cmd_vel_count"] = len(node.nonzero_commands)
        if node.nonzero_commands:
            raise RuntimeError("non-zero /cmd_vel_unstamped was observed")
        control_enabled, calibration_ready = read_safety_flags(node, executor)
        if control_enabled or not calibration_ready:
            raise RuntimeError("coordinate validation safety state changed")
        result["status"] = "passed"
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        if calibration_enabled_for_test:
            try:
                set_calibration_ready(node, executor, False)
                result["calibration_restored"] = True
            except Exception as exc:
                result["calibration_restored"] = False
                result["calibration_restore_error"] = str(exc)
        result.setdefault("nonzero_cmd_vel_count", len(node.nonzero_commands))
        RESULT.write_text(json.dumps(result, indent=2) + "\n")
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

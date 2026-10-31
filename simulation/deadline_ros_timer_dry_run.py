#!/usr/bin/env python3
"""ROS-executor dry-run for the bounded-drive deadline timer.

No network socket, Pico, ARM topic, or physical command topic is used.
"""

import argparse
import json
import pathlib
import time


LIMIT_S = 0.4
PHASES_S = (0.000, 0.002, 0.004, 0.006, 0.008)
LOADS_S = (0.000, 0.003, 0.008)


class DeadlineTrial:
    def __init__(self, node, *, poll_period_s, phase_s, callback_load_s):
        self.node = node
        self.poll_period_s = poll_period_s
        self.phase_s = phase_s
        self.callback_load_s = callback_load_s
        self.started_at = None
        self.deadline = None
        self.detected_at = None
        self.stop_started_at = None
        self.dispatch_attempts = 0
        self.pre_deadline_dispatches = 0
        self.post_deadline_dispatches = 0
        self.post_deadline_rejections = 0
        self._start_timer = node.create_timer(0.020 + phase_s, self._start)
        self._deadline_timer = node.create_timer(poll_period_s, self._poll_deadline)
        self._navigation_timer = node.create_timer(0.010, self._navigation_attempt)
        self._load_timer = node.create_timer(0.050, self._callback_load)

    @property
    def complete(self):
        return self.detected_at is not None

    def _start(self):
        if self.started_at is not None:
            return
        self._start_timer.cancel()
        self.started_at = time.monotonic()
        self.deadline = self.started_at + LIMIT_S

    def _poll_deadline(self):
        if self.deadline is None or self.detected_at is not None:
            return
        now = time.monotonic()
        if now >= self.deadline:
            self.detected_at = now
            # A real STOP would begin here. This dry-run performs no I/O.
            self.stop_started_at = time.monotonic()

    def _navigation_attempt(self):
        if self.deadline is None or self.detected_at is not None:
            return
        self.dispatch_attempts += 1
        checked_at = time.monotonic()
        # Mirrors the transport's adjacent pre-send deadline check.
        if checked_at >= self.deadline:
            self.post_deadline_rejections += 1
            return
        self.pre_deadline_dispatches += 1

    def _callback_load(self):
        if self.started_at is None or self.detected_at is not None:
            return
        until = time.monotonic() + self.callback_load_s
        while time.monotonic() < until:
            pass

    def result(self):
        return {
            "poll_period_s": self.poll_period_s,
            "phase_s": self.phase_s,
            "callback_load_s": self.callback_load_s,
            "duration_limit_s": LIMIT_S,
            "detection_late_s": self.detected_at - self.deadline,
            "stop_handler_delay_s": self.stop_started_at - self.detected_at,
            "dispatch_attempts": self.dispatch_attempts,
            "pre_deadline_dispatches": self.pre_deadline_dispatches,
            "post_deadline_dispatches": self.post_deadline_dispatches,
            "post_deadline_rejections": self.post_deadline_rejections,
        }


def summarize(results):
    groups = {}
    for result in results:
        key = "{:g}Hz".format(round(1.0 / result["poll_period_s"]))
        group = groups.setdefault(
            key,
            {
                "trials": 0,
                "max_detection_late_s": 0.0,
                "max_stop_handler_delay_s": 0.0,
                "post_deadline_dispatches": 0,
            },
        )
        group["trials"] += 1
        group["max_detection_late_s"] = max(
            group["max_detection_late_s"], result["detection_late_s"]
        )
        group["max_stop_handler_delay_s"] = max(
            group["max_stop_handler_delay_s"], result["stop_handler_delay_s"]
        )
        group["post_deadline_dispatches"] += result["post_deadline_dispatches"]
    return groups


def run_trial(rclpy, *, poll_period_s, phase_s, callback_load_s):
    node = rclpy.create_node("deadline_ros_timer_dry_run")
    trial = DeadlineTrial(
        node,
        poll_period_s=poll_period_s,
        phase_s=phase_s,
        callback_load_s=callback_load_s,
    )
    expires_at = time.monotonic() + 1.0
    while rclpy.ok() and not trial.complete and time.monotonic() < expires_at:
        rclpy.spin_once(node, timeout_sec=0.02)
    try:
        if not trial.complete:
            raise RuntimeError("deadline trial timed out")
        return trial.result()
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    try:
        results = []
        for period in (0.050, 0.010):
            for load in LOADS_S:
                for phase in PHASES_S:
                    results.append(
                        run_trial(
                            rclpy,
                            poll_period_s=period,
                            phase_s=phase,
                            callback_load_s=load,
                        )
                    )
    finally:
        rclpy.shutdown()

    groups = summarize(results)
    passed = (
        groups["100Hz"]["max_detection_late_s"] <= 0.020
        and groups["100Hz"]["post_deadline_dispatches"] == 0
        and groups["20Hz"]["post_deadline_dispatches"] == 0
    )
    payload = {
        "pass": passed,
        "hardware_connected": False,
        "arm_sent": False,
        "physical_command_topic_created": False,
        "groups": groups,
        "trials": results,
    }
    output = pathlib.Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

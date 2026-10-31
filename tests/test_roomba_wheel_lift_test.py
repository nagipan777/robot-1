import os
import pathlib
import subprocess
import tempfile
import unittest

from simulation.roomba_wheel_lift_test_core import (
    ATTENDED_FLOOR_CONFIRMATION,
    CONTINUOUS_CONFIRMATION,
    MAX_CONTINUOUS_DRIVE_DURATION_S,
    MAX_DRIVE_DURATION_S,
    MAX_WHEEL_SPEED_MM_S,
    ROTATION_CONFIRMATION,
    SubscriberConnectionGate,
    WATCHDOG_SILENCE_S,
    WheelLiftTestProfile,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]


class WheelLiftTestProfileTests(unittest.TestCase):
    def test_publisher_qos_keeps_only_latest_reliable_sample(self):
        source = (ROOT / "simulation" / "roomba_wheel_lift_test.py").read_text()
        qos = source[
            source.index("COMMAND_QOS =") : source.index("class RoombaWheelLiftTest")
        ]
        self.assertIn("history=HistoryPolicy.KEEP_LAST", qos)
        self.assertIn("depth=1", qos)
        self.assertIn("reliability=ReliabilityPolicy.RELIABLE", qos)
        self.assertIn("durability=DurabilityPolicy.VOLATILE", qos)

    def run_launcher(self, mode, *extra):
        with tempfile.TemporaryDirectory() as temporary_directory:
            fake_docker = pathlib.Path(temporary_directory) / "docker"
            fake_docker.write_text(
                '#!/bin/bash\nprintf "%s\\n" "$@"\n', encoding="utf-8"
            )
            fake_docker.chmod(0o755)
            environment = os.environ.copy()
            environment["PATH"] = temporary_directory + os.pathsep + environment["PATH"]
            return subprocess.run(
                [
                    "/bin/bash",
                    str(ROOT / "simulation" / "sim.sh"),
                    "roomba-wheel-lift-test",
                    mode,
                    *extra,
                ],
                cwd=ROOT,
                env=environment,
                check=False,
                capture_output=True,
                text=True,
            )

    def test_default_is_stop_only(self):
        profile = WheelLiftTestProfile()
        self.assertEqual(profile.mode, "stop")
        self.assertFalse(profile.sends_nonzero)

    def test_subscriber_connection_gate_waits_then_succeeds(self):
        gate = SubscriberConnectionGate(timeout_s=2.0, started_at=10.0)
        self.assertFalse(gate.ready(0, 11.9))
        self.assertTrue(gate.ready(1, 11.9))

    def test_subscriber_connection_gate_times_out_and_rejects_invalid_timeout(self):
        gate = SubscriberConnectionGate(timeout_s=2.0, started_at=10.0)
        with self.assertRaisesRegex(TimeoutError, "no subscriber matched"):
            gate.ready(0, 12.0)
        for timeout in (0.0, -1.0, float("nan")):
            with self.subTest(timeout=timeout):
                with self.assertRaises(ValueError):
                    SubscriberConnectionGate(timeout_s=timeout)

    def test_rotation_requires_two_explicit_permissions(self):
        for allow, confirmation in (
            (False, ""),
            (True, ""),
            (False, ROTATION_CONFIRMATION),
            (True, "wrong"),
        ):
            with self.subTest(allow=allow, confirmation=confirmation):
                with self.assertRaises(PermissionError):
                    WheelLiftTestProfile(
                        "rotate",
                        allow_rotation=allow,
                        confirmation=confirmation,
                    )

        profile = WheelLiftTestProfile(
            "rotate",
            allow_rotation=True,
            confirmation=ROTATION_CONFIRMATION,
        )
        self.assertTrue(profile.sends_nonzero)

    def test_watchdog_requires_same_explicit_permissions(self):
        with self.assertRaises(PermissionError):
            WheelLiftTestProfile("watchdog")
        profile = WheelLiftTestProfile(
            "watchdog",
            allow_rotation=True,
            confirmation=ROTATION_CONFIRMATION,
        )
        self.assertEqual(WATCHDOG_SILENCE_S, 0.75)
        self.assertTrue(profile.sends_nonzero)

    def test_attended_floor_requires_distinct_confirmation_and_same_limits(self):
        with self.assertRaises(PermissionError):
            WheelLiftTestProfile(
                "floor_straight",
                allow_rotation=True,
                confirmation=ROTATION_CONFIRMATION,
            )
        profile = WheelLiftTestProfile(
            "floor_straight",
            allow_rotation=True,
            confirmation=ATTENDED_FLOOR_CONFIRMATION,
        )
        self.assertEqual(profile.twist(), (0.02, 0.0))
        self.assertTrue(profile.sends_nonzero)

    def test_continuous_straight_is_separate_confirmed_and_bounded(self):
        self.assertEqual(MAX_CONTINUOUS_DRIVE_DURATION_S, 5.0)
        for confirmation in ("", ATTENDED_FLOOR_CONFIRMATION, ROTATION_CONFIRMATION):
            with self.subTest(confirmation=confirmation), self.assertRaises(
                PermissionError
            ):
                WheelLiftTestProfile(
                    "continuous_straight",
                    drive_duration_s=2.0,
                    allow_rotation=True,
                    confirmation=confirmation,
                )
        profile = WheelLiftTestProfile(
            "continuous_straight",
            drive_duration_s=2.0,
            allow_rotation=True,
            confirmation=CONTINUOUS_CONFIRMATION,
        )
        self.assertEqual(profile.twist(), (0.02, 0.0))

    def test_motion_waits_for_matching_arm_acceptance(self):
        source = (ROOT / "simulation" / "roomba_wheel_lift_test.py").read_text()
        tick = source[source.index("    def tick(self):") :]
        self.assertIn("if self.arm_accepted_at is None:", tick)
        self.assertIn("matching ARM acceptance timed out; STOP only", tick)
        self.assertLess(
            tick.index("if self.arm_accepted_at is None:"),
            tick.index("self.publish_test_velocity()"),
        )

    def test_mismatched_arm_acceptance_cannot_start_drive_window(self):
        source = (ROOT / "simulation" / "roomba_wheel_lift_test.py").read_text()
        callback = source[
            source.index("    def on_arm_accepted") : source.index(
                "    def publish_test_velocity"
            )
        ]
        self.assertIn("int(message.data) != self.arm_generation", callback)
        self.assertIn("self.arm_accepted_at = time.monotonic()", callback)
        with self.assertRaises(ValueError):
            WheelLiftTestProfile(
                "continuous_straight",
                drive_duration_s=5.001,
                allow_rotation=True,
                confirmation=CONTINUOUS_CONFIRMATION,
            )

    def test_wheel_speed_and_duration_limits_are_enforced(self):
        self.assertEqual(MAX_WHEEL_SPEED_MM_S, 20)
        self.assertEqual(MAX_DRIVE_DURATION_S, 1.0)
        for kwargs in (
            {"left_mm_s": 20.001},
            {"right_mm_s": -20.001},
            {"drive_duration_s": 1.001},
            {"drive_duration_s": 0.0},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    WheelLiftTestProfile("rejection", **kwargs)

    def test_wheel_speeds_convert_to_twist(self):
        straight = WheelLiftTestProfile(
            "rejection", left_mm_s=20, right_mm_s=20
        )
        self.assertEqual(straight.twist(), (0.02, 0.0))
        turn = WheelLiftTestProfile(
            "rejection", left_mm_s=-20, right_mm_s=20
        )
        self.assertAlmostEqual(turn.twist()[0], 0.0)
        self.assertAlmostEqual(turn.twist()[1], 0.04 / 0.235)

    def test_node_is_dedicated_and_watchdog_window_has_no_publish(self):
        source = (
            ROOT / "simulation" / "roomba_wheel_lift_test.py"
        ).read_text()
        self.assertIn('"/roomba/cmd_vel_unstamped"', source)
        self.assertNotIn('"/cmd_vel_unstamped"', source)
        window = source[
            source.index('if self.profile.mode == "watchdog":'):
            source.index("        self.finish()", source.index('if self.profile.mode == "watchdog":'))
        ]
        self.assertNotIn("publish_stop", window)
        self.assertNotIn("publish_test_velocity", window)
        self.assertIn("node.emergency_stop()", source)
        self.assertIn("get_subscription_count()", source)
        self.assertIn("SubscriberConnectionGate", source)
        self.assertIn("published non-zero Twist", source)
        self.assertIn("qos=reliable/volatile", source)
        self.assertIn('declare_parameter("publish_period_s", 0.05)', source)
        self.assertIn("self.create_timer(self.publish_period_s", source)
        self.assertIn("self.drive_started_at = None", source)
        self.assertIn("self.arm_accepted_at = None", source)
        self.assertIn("def on_arm_accepted", source)
        self.assertIn("matching ARM acceptance timed out; STOP only", source)
        self.assertIn("elapsed = now - self.drive_started_at", source)
        self.assertNotIn("elapsed = now - self.arm_sent_at - ARM_SETTLE_S", source)
        self.assertNotIn("ARM_SETTLE_S", source)

    def test_existing_drive_locks_remain_disabled(self):
        bridge = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        relay = (ROOT / "pico" / "pico_relay.py").read_text()
        self.assertIn('declare_parameter("drive_enabled", False)', bridge)
        self.assertIn("ALLOW_NONZERO_DRIVE = False", relay)

    def test_container_and_explicit_launcher_install_only_the_test_node(self):
        dockerfile = (ROOT / "simulation" / "Dockerfile").read_text()
        script = (ROOT / "simulation" / "sim.sh").read_text()
        self.assertIn(
            "COPY roomba_wheel_lift_test.py /usr/local/bin/roomba-wheel-lift-test",
            dockerfile,
        )
        start = script.index("  roomba-wheel-lift-test)")
        branch = script[start:script.index("  test-marker)", start)]
        self.assertIn('mode="${2:-stop}"', branch)
        self.assertIn('"${3:-}" != "WHEELS_LIFTED"', branch)
        self.assertNotIn("drive_enabled:=true", branch)

    def test_stop_and_rejection_launch_without_optional_array_arguments(self):
        for mode in ("stop", "rejection"):
            with self.subTest(mode=mode):
                result = self.run_launcher(mode)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(mode, result.stdout.splitlines())
                self.assertNotIn("allow_rotation:=true", result.stdout)
                self.assertNotIn("confirmation:=WHEELS_LIFTED", result.stdout)

    def test_rotate_and_watchdog_require_and_forward_both_locks(self):
        for mode in ("rotate", "watchdog"):
            with self.subTest(mode=mode):
                denied = self.run_launcher(mode)
                self.assertEqual(denied.returncode, 2)
                self.assertIn("WHEELS_LIFTED", denied.stderr)

                allowed = self.run_launcher(mode, "WHEELS_LIFTED")
                self.assertEqual(allowed.returncode, 0, allowed.stderr)
                arguments = allowed.stdout.splitlines()
                self.assertIn(mode, arguments)
                self.assertIn("allow_rotation:=true", arguments)
                self.assertIn("confirmation:=WHEELS_LIFTED", arguments)
                self.assertTrue(
                    any(value.startswith("arm_generation:=") for value in arguments)
                )

    def test_watchdog_window_is_point_seven_five_seconds_and_has_no_publish(self):
        source = (ROOT / "simulation" / "roomba_wheel_lift_test.py").read_text()
        self.assertEqual(WATCHDOG_SILENCE_S, 0.75)
        window = source[
            source.index('if self.profile.mode == "watchdog":') :
            source.index("        self.finish()", source.index('if self.profile.mode == "watchdog":'))
        ]
        self.assertIn("publishing nothing", window)
        self.assertNotIn("self.publisher.publish", window)


if __name__ == "__main__":
    unittest.main()

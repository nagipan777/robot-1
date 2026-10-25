import math
import pathlib
import struct
import unittest

from simulation.differential_drive import twist_to_wheel_speeds
from simulation.roomba_tcp_transport import (
    DIAGNOSTIC_MAGIC,
    SEQUENCED_DRIVE,
    drive_direct_packet,
    with_crc,
)
from simulation.target_manager import SafeTargetManager, evaluate_robot_guard


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RealRoombaIntegrationReadinessTests(unittest.TestCase):
    def test_guard_pose_freshness_and_output_remain_dry_run_only(self):
        config = (
            ROOT / "simulation" / "vision_controller.real_dry_run.yaml"
        ).read_text()
        adapter = (ROOT / "simulation" / "cmd_vel_output_adapter.yaml").read_text()

        self.assertRegex(config, r"(?m)^\s*control_enabled:\s*false$")
        self.assertRegex(config, r"(?m)^\s*calibration_ready:\s*false$")
        self.assertRegex(config, r"(?m)^\s*dry_run_enabled:\s*true$")
        self.assertRegex(config, r"(?m)^\s*robot_guard_radius:\s*0\.5$")
        self.assertRegex(config, r"(?m)^\s*max_robot_pose_age:\s*1\.0$")
        self.assertRegex(adapter, r"(?m)^\s*mode:\s*dry_run$")
        self.assertIn("dry_run_topic: /roomba/cmd_vel_dry_run", adapter)

        self.assertFalse(evaluate_robot_guard((0.5, 0.0), (0.0, 0.0), 0.5)["accepted"])
        self.assertTrue(evaluate_robot_guard((0.501, 0.0), (0.0, 0.0), 0.5)["accepted"])

    def test_reached_target_jitter_stays_latched_and_new_target_releases(self):
        manager = SafeTargetManager()
        self.assertEqual(manager.consider((1.0, 0.0), 0.20), "new")
        manager.mark_reached()
        self.assertEqual(manager.consider((1.05, 0.02), 0.20), "blocked_same")
        manager.suspend()
        self.assertEqual(manager.consider((1.10, 0.0), 0.20), "blocked_same")
        self.assertEqual(manager.consider((1.25, 0.0), 0.20), "new")

    def test_navigation_twist_is_bounded_before_mock_tcp9000_encoding(self):
        # Exercise the physical bridge's existing proportional wheel limit
        # without opening a socket or sending to TCP 9000.
        left, right = twist_to_wheel_speeds(
            0.08,
            0.8,
            track_width_m=0.235,
            max_wheel_speed_mm_s=50,
        )
        self.assertLessEqual(max(abs(left), abs(right)), 50)
        self.assertEqual((left, right), (-4, 50))

        generation = 123
        sequence = 1
        payload = (
            bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE])
            + struct.pack(">II", generation, sequence)
            + drive_direct_packet(left, right)
        )
        frame = with_crc(payload)
        self.assertEqual(len(frame), 17)
        self.assertEqual(frame[10], 145)
        self.assertEqual(struct.unpack(">hh", frame[11:15]), (right, left))

        # Final live-camera dry-run command, evaluated with the attended
        # hardware trial's 20 mm/s bridge limit.
        left, right = twist_to_wheel_speeds(
            0.08,
            0.15653609480666394,
            track_width_m=0.235,
            max_wheel_speed_mm_s=20,
        )
        self.assertEqual((left, right), (13, 20))

    def test_existing_incomplete_ack_and_fail_safe_checks_are_present(self):
        transport_tests = (ROOT / "tests" / "test_roomba_tcp_transport.py").read_text()
        relay_tests = (ROOT / "tests" / "test_pico_relay_protocol.py").read_text()
        bridge = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()

        self.assertIn("test_drive_ack_failure_is_rejected", transport_tests)
        self.assertIn("test_incomplete_drive_uart_results", relay_tests)
        self.assertIn("RoombaDiagnosticError", bridge)
        self.assertIn("self.handle_send_failure(error)", bridge)


if __name__ == "__main__":
    unittest.main()

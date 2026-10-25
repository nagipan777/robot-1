import math
import pathlib
import unittest

from simulation.aruco_pose_core import (
    PoseFreshnessGate,
    StreamTimeoutGate,
    marker_pose,
    select_marker_observation,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]


class ArucoPoseGeometryTests(unittest.TestCase):
    H = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0]

    def test_marker_detection_success_missing_and_id_mismatch(self):
        observation = {"marker_id": 3, "corners": [[0, 0]] * 4}
        self.assertIs(select_marker_observation([observation], 3), observation)
        self.assertIsNone(select_marker_observation([], 3))
        self.assertIsNone(select_marker_observation([observation], 4))
        with self.assertRaises(ValueError):
            select_marker_observation({}, 3)

    def test_center_yaw_height_and_robot_offset(self):
        pose = marker_pose(
            [(90, 90), (110, 90), (110, 110), (90, 110)],
            self.H,
            camera_ground=(0.0, 0.0),
            camera_height=2.0,
            marker_height=0.2,
            marker_to_robot_forward=0.1,
        )
        self.assertAlmostEqual(pose[0], 1.0)
        self.assertAlmostEqual(pose[1], 0.9)
        self.assertAlmostEqual(pose[2], 0.0)

    def test_yaw_and_left_offset(self):
        x, y, yaw = marker_pose(
            [(110, 90), (110, 110), (90, 110), (90, 90)],
            self.H,
            camera_ground=(0.0, 0.0),
            camera_height=2.0,
            marker_height=0.0,
            marker_to_robot_left=0.1,
        )
        self.assertAlmostEqual(x, 0.9)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_invalid_geometry_is_rejected(self):
        cases = (
            ([(0, 0)] * 4, 2.0, 0.0),
            ([(0, 0), (20, 0), (20, 20), (0, 20)], 0.0, 0.0),
            ([(0, 0), (20, 0), (20, 20), (0, 20)], 1.0, 1.0),
        )
        for corners, camera_height, marker_height in cases:
            with self.subTest(corners=corners, camera_height=camera_height):
                with self.assertRaises(ValueError):
                    marker_pose(
                        corners,
                        self.H,
                        camera_ground=(0, 0),
                        camera_height=camera_height,
                        marker_height=marker_height,
                    )

    def test_map_bounds_reject_pose(self):
        with self.assertRaisesRegex(ValueError, "outside map bounds"):
            marker_pose(
                [(190, 190), (210, 190), (210, 210), (190, 210)],
                self.H,
                camera_ground=(0, 0),
                camera_height=2.0,
                marker_height=0.0,
                map_bounds=[0, 1, 0, 1],
            )


class PoseFreshnessTests(unittest.TestCase):
    def test_requires_consecutive_fresh_monotonic_detections(self):
        gate = PoseFreshnessGate(max_age=0.5, min_consecutive=3)
        self.assertEqual(gate.observe(9.7, 10.0), (False, "warming_up"))
        self.assertEqual(gate.observe(9.8, 10.0), (False, "warming_up"))
        self.assertEqual(gate.observe(9.9, 10.0), (True, "accepted"))

    def test_stale_replay_and_loss_reset_gate(self):
        gate = PoseFreshnessGate(max_age=0.5, min_consecutive=2)
        self.assertEqual(gate.observe(9.0, 10.0)[1], "stale_or_future")
        self.assertEqual(gate.observe(9.8, 10.0)[1], "warming_up")
        self.assertEqual(gate.observe(9.8, 10.0)[1], "non_monotonic")
        self.assertEqual(gate.missing()[1], "marker_lost")

    def test_camera_stream_timeout_is_distinct_from_marker_loss(self):
        gate = StreamTimeoutGate(max_age=0.5)
        self.assertFalse(gate.check(10.0))
        gate.frame_received(10.0)
        self.assertFalse(gate.check(10.5))
        self.assertTrue(gate.check(10.501))
        self.assertFalse(gate.check(11.0))
        gate.frame_received(12.0)
        self.assertTrue(gate.check(12.501))

    def test_defaults_and_existing_safety_flags_remain_locked(self):
        config = (ROOT / "simulation/aruco_pose.yaml").read_text()
        self.assertIn("pose_calibration_ready: false", config)
        self.assertIn("pose_output_enabled: false", config)
        self.assertIn("marker_yaw_offset_rad: 1.508362", config)
        controller = (ROOT / "simulation/vision_controller.real_dry_run.yaml").read_text()
        self.assertRegex(controller, r"(?m)^\s*control_enabled:\s*false$")
        self.assertRegex(controller, r"(?m)^\s*calibration_ready:\s*false$")
        relay = (ROOT / "pico/pico_relay.py").read_text()
        self.assertIn("ALLOW_NONZERO_DRIVE = False", relay)
        node = (ROOT / "simulation/aruco_pose_publisher.py").read_text()
        self.assertIn('self.create_timer(0.1, self.check_marker_timeout)', node)
        self.assertIn('"LOST", "marker_timeout"', node)
        dockerfile = (ROOT / "simulation/Dockerfile").read_text()
        self.assertIn("COPY aruco_pose_publisher.py /usr/local/bin/aruco-pose-publisher", dockerfile)
        launcher = (ROOT / "simulation/sim.sh").read_text()
        self.assertIn("  aruco-pose)", launcher)
        self.assertIn("  aruco-pose-start)", launcher)
        supervisor = (ROOT / "simulation/supervisord.conf").read_text()
        self.assertIn("[program:aruco-pose]", supervisor)
        self.assertIn("autostart=false", supervisor)

    def test_uncalibrated_pose_is_candidate_only(self):
        source = (ROOT / "simulation/aruco_pose_publisher.py").read_text()
        candidate = source.index("self.candidate.publish(pose)")
        calibration_gate = source.index("if not calibration_ready:", candidate)
        localization = source.index("self.localization.publish(pose)", calibration_gate)
        self.assertLess(candidate, calibration_gate)
        self.assertLess(calibration_gate, localization)
        self.assertIn(
            'self.publish_status("CANDIDATE_ONLY", "pose_calibration_not_ready")',
            source[calibration_gate:localization],
        )
        self.assertIn("return", source[calibration_gate:localization])


if __name__ == "__main__":
    unittest.main()

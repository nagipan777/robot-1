import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class LocalizationConfigTests(unittest.TestCase):
    def test_preview_is_mock_only_and_pose_output_is_locked(self):
        config = (ROOT / "simulation/localization_preview.yaml").read_text()
        self.assertIn("preview_enabled: true", config)
        self.assertIn("input_topic: /localization/encoder_counts_mock", config)
        self.assertIn("output_topic: /localization/wheel_odom", config)
        self.assertIn("pose_output_enabled: false", config)
        for forbidden in ("9000", "192.168."):
            self.assertNotIn(forbidden, config)
        sources = "".join(
            (ROOT / path).read_text()
            for path in (
                "simulation/wheel_odometry_node.py",
                "simulation/localization_fusion_node.py",
            )
        )
        self.assertNotIn("import socket", sources)
        self.assertNotIn("import machine", sources)

    def test_aruco_has_distinct_raw_and_accepted_topics(self):
        config = (ROOT / "simulation/aruco_pose.yaml").read_text()
        self.assertIn("candidate_topic: /robot/pose_candidate", config)
        self.assertIn("localization_topic: /localization/aruco_pose", config)
        source = (ROOT / "simulation/aruco_pose_publisher.py").read_text()
        accepted = source.index("self.localization.publish(pose)")
        gate = source.index("if not accepted:")
        self.assertGreater(accepted, gate)

    def test_hardware_live_command_is_fail_closed(self):
        launcher = (ROOT / "simulation/sim.sh").read_text()
        section = launcher.split("localization-hardware-live)", 1)[1].split(";;", 1)[0]
        self.assertIn("Refusing hardware-live localization", section)
        self.assertIn("exit 1", section)

    def test_existing_motion_locks_remain_false(self):
        real_config = (ROOT / "simulation/vision_controller.real_dry_run.yaml").read_text()
        self.assertRegex(real_config, r"(?m)^\s*control_enabled:\s*false$")
        self.assertRegex(real_config, r"(?m)^\s*calibration_ready:\s*false$")
        relay = (ROOT / "pico/pico_relay.py").read_text()
        self.assertIn("ALLOW_NONZERO_DRIVE = False", relay)
        bridge = (ROOT / "simulation/roomba_bridge_tcp.py").read_text()
        self.assertIn('self.declare_parameter("drive_enabled", False)', bridge)

    def test_roomba_odometry_observer_is_opt_in_and_navigation_independent(self):
        config = (ROOT / "simulation/roomba_odometry_observer.yaml").read_text()
        self.assertIn("odometry_enabled: false", config)
        self.assertIn("input_topic: /roomba/encoder_counts", config)
        self.assertIn("output_topic: /roomba/odom", config)
        self.assertIn("frame_id: roomba_odom", config)
        self.assertIn("child_frame_id: roomba_base_link", config)
        node = (ROOT / "simulation/wheel_odometry_node.py").read_text()
        self.assertNotIn("TransformBroadcaster", node)
        self.assertNotIn('create_publisher(PoseStamped, "/robot/pose"', node)
        self.assertNotIn("import socket", node)
        self.assertNotIn("import serial", node)


if __name__ == "__main__":
    unittest.main()

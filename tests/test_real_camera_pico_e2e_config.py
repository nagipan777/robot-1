import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RealCameraPicoE2EConfigTests(unittest.TestCase):
    def setUp(self):
        self.config = (ROOT / "simulation/vision_controller.real_dry_run.yaml").read_text()
        self.aruco_config = (
            ROOT / "simulation/vision_controller.real_aruco_dry_run.yaml"
        ).read_text()
        self.controller = (ROOT / "simulation/vision_turn_controller.py").read_text()

    def test_physical_control_flags_stay_locked(self):
        self.assertRegex(self.config, r"(?m)^\s*control_enabled:\s*false$")
        self.assertRegex(self.config, r"(?m)^\s*calibration_ready:\s*false$")
        self.assertRegex(self.config, r"(?m)^\s*dry_run_enabled:\s*true$")
        self.assertIn("dry_run_enabled and control_enabled cannot both be true", self.controller)

    def test_real_camera_calibration_and_guard_are_explicit(self):
        self.assertRegex(self.config, r"(?m)^\s*calibration_image_width:\s*1280$")
        self.assertRegex(self.config, r"(?m)^\s*calibration_image_height:\s*960$")
        self.assertRegex(self.config, r"(?m)^\s*robot_guard_radius:\s*0\.5$")
        self.assertRegex(self.config, r"(?m)^\s*max_robot_pose_age:\s*1\.0$")
        self.assertRegex(self.config, r"(?m)^\s*min_detection_confidence:\s*0\.50$")
        self.assertRegex(self.config, r"(?m)^\s*target_confirmation_count:\s*3$")
        self.assertRegex(
            self.config,
            r"(?m)^\s*target_confirmation_distance:\s*0\.20$",
        )

    def test_physical_dry_run_keeps_target_identity_and_stop_latch(self):
        self.assertRegex(
            self.config,
            r"(?m)^\s*safe_target_management_enabled:\s*true$",
        )
        self.assertRegex(self.config, r"(?m)^\s*target_change_distance:\s*0\.20$")
        lose_start = self.controller.index("    def lose_detection(self):")
        lose_end = self.controller.index("    def publish_stop", lose_start)
        lose_detection = self.controller[lose_start:lose_end]
        self.assertIn("control_enabled or dry_run_enabled", lose_detection)

    def test_candidate_and_approved_topics_are_separate(self):
        self.assertIn("detection_target_topic: /litter/target_candidate", self.config)
        self.assertIn("candidate_target_topic: /litter/target_candidate", self.config)
        self.assertIn("target_topic: /litter/target", self.config)
        guard = self.controller.index('reason=robot_pose_unavailable')
        approved = self.controller.index("self.approved_target_publisher.publish(message)")
        self.assertLess(guard, approved)

    def test_dry_run_output_uses_navigation_boundary(self):
        self.assertIn("dry_run_cmd_vel_topic: /nav/cmd_vel_unstamped", self.config)
        adapter = (ROOT / "simulation/cmd_vel_output_adapter.yaml").read_text()
        self.assertIn("input_topic: /nav/cmd_vel_unstamped", adapter)
        self.assertIn("dry_run_topic: /roomba/cmd_vel_dry_run", adapter)

    def test_pico_receiver_remains_uart_free_and_on_9001(self):
        receiver = (ROOT / "pico/cmd_vel_dry_run_receiver.py").read_text()
        self.assertIn("PORT = 9001", receiver)
        self.assertNotIn("import machine", receiver)
        self.assertIn("UART=DISABLED", receiver)

    def test_e2e_nodes_can_start_under_the_existing_supervisor(self):
        supervisor = (ROOT / "simulation/supervisord.conf").read_text()
        self.assertIn("[program:cmd-vel-output-adapter]", supervisor)
        self.assertIn("[program:dry-run-pose-publisher]", supervisor)
        self.assertIn("[program:pico-dry-run-bridge]", supervisor)
        self.assertIn("PICO_DRY_RUN_HOST", supervisor)

    def test_real_camera_dry_run_disables_map_frame_adapter(self):
        supervisor = (ROOT / "simulation/supervisord.conf").read_text()
        start = supervisor.index("[program:map-frame-adapter]")
        end = supervisor.index("[program:vision-turn-controller]", start)
        adapter = supervisor[start:end]
        self.assertIn(
            '[ \\"$VISION_CONTROLLER_CONFIG\\" = '
            "/etc/vision-controller.real-dry-run.yaml ]",
            adapter,
        )
        self.assertIn(
            '[ \\"$VISION_CONTROLLER_CONFIG\\" = '
            "/etc/vision-controller.real-aruco-dry-run.yaml ]",
            adapter,
        )

    def test_detection_metadata_is_buffered_by_candidate_stamp(self):
        self.assertIn("self.pending_detection_metadata = {}", self.controller)
        self.assertIn(
            "metadata = self.pending_detection_metadata.pop(stamp_key, None)",
            self.controller,
        )
        self.assertIn("while len(self.pending_detection_metadata) > 32", self.controller)

    def test_direct_map_candidate_does_not_self_subscribe(self):
        self.assertIn(
            "self.direct_candidate_processing = detection_target_topic == candidate_target_topic",
            self.controller,
        )
        self.assertIn("if not self.direct_candidate_processing:", self.controller)
        self.assertIn("if self.direct_candidate_processing:\n            self.on_target(point)", self.controller)

    def test_unconfirmed_candidate_does_not_enter_control_state(self):
        confirmation = self.controller.index("self.target_confirmation.consider(")
        target_assignment = self.controller.index(
            "self.target_odom = candidate_target", confirmation
        )
        self.assertLess(confirmation, target_assignment)

    def test_live_aruco_dry_run_uses_candidate_pose_without_physical_control(self):
        self.assertIn("pose_topic: /robot/pose_candidate", self.aruco_config)
        self.assertRegex(self.aruco_config, r"(?m)^\s*control_enabled:\s*false$")
        self.assertRegex(self.aruco_config, r"(?m)^\s*calibration_ready:\s*false$")
        self.assertRegex(self.aruco_config, r"(?m)^\s*dry_run_enabled:\s*true$")
        self.assertIn("dry_run_cmd_vel_topic: /nav/cmd_vel_unstamped", self.aruco_config)

    def test_live_aruco_dry_run_has_separate_safe_start_command(self):
        script = (ROOT / "simulation/sim.sh").read_text()
        self.assertIn("real-camera-aruco-dry-run-up)", script)
        self.assertIn(
            "VISION_CONTROLLER_CONFIG=/etc/vision-controller.real-aruco-dry-run.yaml",
            script,
        )
        self.assertIn('PICO_DRY_RUN_HOST="" SPAWN_DOCK=false', script)

    def test_live_aruco_mode_disables_fixed_pose_and_map_adapter(self):
        supervisor = (ROOT / "simulation/supervisord.conf").read_text()
        self.assertIn("/etc/vision-controller.real-aruco-dry-run.yaml", supervisor)
        fixed_start = supervisor.index("[program:dry-run-pose-publisher]")
        fixed_end = supervisor.index("[program:pico-dry-run-bridge]", fixed_start)
        fixed = supervisor[fixed_start:fixed_end]
        self.assertNotIn("real-aruco-dry-run", fixed)

    def test_evidence_recorder_captures_live_aruco_yaw(self):
        recorder = (ROOT / "simulation/e2e_dry_run_recorder.py").read_text()
        self.assertIn('"/robot/pose_candidate"', recorder)
        self.assertIn('"yaw": yaw', recorder)


if __name__ == "__main__":
    unittest.main()

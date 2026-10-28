import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class WheelMotionMonitorConfigurationTests(unittest.TestCase):
    def test_monitor_is_opt_in_observation_only_and_uses_existing_topics(self):
        config = (ROOT / "simulation" / "wheel_motion_monitor.yaml").read_text()
        node = (ROOT / "simulation" / "wheel_motion_monitor_node.py").read_text()
        self.assertIn("monitor_enabled: false", config)
        self.assertIn("/roomba/encoder_counts", config)
        self.assertIn("/roomba/cmd_vel_unstamped", config)
        self.assertIn("/roomba/arm_generation", config)
        self.assertIn("warnings are not connected to STOP", node)
        self.assertNotIn("RoombaTcpTransport", node)
        self.assertNotIn("socket", node)
        self.assertNotIn("send_stop", node)

    def test_docker_image_and_dryrun_launcher_include_monitor(self):
        dockerfile = (ROOT / "simulation" / "Dockerfile").read_text()
        script = (ROOT / "simulation" / "sim.sh").read_text()
        self.assertIn("COPY wheel_motion_monitor.py", dockerfile)
        self.assertIn("COPY wheel_motion_monitor_node.py", dockerfile)
        self.assertIn("wheel-monitor-dryrun)", script)
        case = script[
            script.index("  wheel-monitor-dryrun)") :
            script.index("  localization-hardware-live)")
        ]
        self.assertIn("run_wheel_monitor_dryrun.py", case)
        self.assertNotIn("docker", case)
        self.assertNotIn("roomba-tcp", case)


if __name__ == "__main__":
    unittest.main()

import importlib.util
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "simulation" / "deadline_ros_timer_dry_run.py"


def load_module():
    spec = importlib.util.spec_from_file_location("deadline_ros_timer_dry_run", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DeadlineRosTimerDryRunTests(unittest.TestCase):
    def test_harness_is_hardware_free_and_checks_pre_send_deadline(self):
        source = MODULE_PATH.read_text()
        self.assertNotIn("import socket", source)
        self.assertNotIn("/roomba/cmd_vel_unstamped", source)
        self.assertIn("checked_at >= self.deadline", source)
        self.assertIn('"hardware_connected": False', source)
        self.assertIn('"arm_sent": False', source)

    def test_summary_separates_20hz_and_100hz(self):
        module = load_module()
        results = [
            {
                "poll_period_s": 0.05,
                "detection_late_s": 0.041,
                "stop_handler_delay_s": 0.0002,
                "post_deadline_dispatches": 0,
            },
            {
                "poll_period_s": 0.01,
                "detection_late_s": 0.008,
                "stop_handler_delay_s": 0.0001,
                "post_deadline_dispatches": 0,
            },
        ]
        summary = module.summarize(results)
        self.assertEqual(summary["20Hz"]["max_detection_late_s"], 0.041)
        self.assertEqual(summary["100Hz"]["max_detection_late_s"], 0.008)


if __name__ == "__main__":
    unittest.main()

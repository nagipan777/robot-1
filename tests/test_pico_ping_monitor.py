import unittest
from unittest.mock import patch
import subprocess

from pico_ping_monitor import parse_latency, ping_command, run_ping, summarize


class PicoPingMonitorTests(unittest.TestCase):
    def test_macos_timeout_is_in_milliseconds(self):
        command = ping_command("192.0.2.1", 1.5, system="Darwin", executable="ping")
        self.assertEqual(command, ["ping", "-n", "-c", "1", "-W", "1500", "192.0.2.1"])

    def test_parses_macos_latency(self):
        self.assertEqual(parse_latency("64 bytes: time=12.345 ms"), 12.345)
        self.assertIsNone(parse_latency("Request timeout"))

    @patch("pico_ping_monitor.subprocess.run")
    def test_process_timeout_is_recorded_as_failed_sample(self, mocked_run):
        mocked_run.side_effect = subprocess.TimeoutExpired(["ping"], 2.0)
        self.assertEqual(run_ping("192.0.2.1", 1.0), (False, None, 124))

    def test_summary_reports_startup_and_failure_streak(self):
        samples = [
            {"success": False, "latency_ms": None, "elapsed_seconds": 1.0},
            {"success": False, "latency_ms": None, "elapsed_seconds": 2.0},
            {"success": True, "latency_ms": 10.0, "elapsed_seconds": 2.5},
            {"success": False, "latency_ms": None, "elapsed_seconds": 4.0},
            {"success": True, "latency_ms": 20.0, "elapsed_seconds": 5.0},
        ]
        result = summarize(samples)
        self.assertEqual(result["first_response_seconds"], 2.5)
        self.assertEqual(result["longest_consecutive_failures"], 2)
        self.assertEqual(result["response_rate_percent"], 40.0)
        self.assertEqual(result["average_latency_ms"], 15.0)


if __name__ == "__main__":
    unittest.main()

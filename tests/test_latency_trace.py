import json
import pathlib
import tempfile
import unittest

from simulation.latency_trace import LatencyTrace


class LatencyTraceTests(unittest.TestCase):
    def test_empty_path_disables_trace(self):
        trace = LatencyTrace("", "disabled")
        self.assertFalse(trace.enabled)
        trace.record("ignored", value=1)
        trace.close()

    def test_jsonl_contains_both_clocks_and_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "trace.jsonl"
            trace = LatencyTrace(path, "test")
            trace.record("bridge_receive", generation=42)
            trace.close()
            event = json.loads(path.read_text())
        self.assertEqual(event["component"], "test")
        self.assertEqual(event["stage"], "bridge_receive")
        self.assertEqual(event["generation"], 42)
        self.assertIsInstance(event["monotonic_ns"], int)
        self.assertIsInstance(event["wall_time_ns"], int)

    def test_navigation_and_bridge_trace_are_opt_in(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        navigation = (root / "simulation" / "vision_turn_controller.py").read_text()
        bridge = (root / "simulation" / "roomba_bridge_tcp.py").read_text()
        self.assertIn('declare_parameter("latency_trace_path", "")', navigation)
        self.assertIn('declare_parameter("latency_trace_path", "")', bridge)
        self.assertIn('"navigation_command_created"', navigation)
        self.assertIn('"navigation_publish_return"', navigation)
        self.assertIn('"bridge_receive"', bridge)
        self.assertIn('reason="drive_enabled_false"', bridge)


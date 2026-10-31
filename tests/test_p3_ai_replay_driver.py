import unittest

from simulation.p3_ai_replay_driver import fresh_copy


class P3AIReplayDriverTests(unittest.TestCase):
    def test_fresh_copy_preserves_input_and_sets_acquisition_timestamp(self):
        original = {"timestamp": 1.0, "detections": [{"label": "litter"}]}
        copied = fresh_copy(original, timestamp=12.5)
        self.assertEqual(original["timestamp"], 1.0)
        self.assertEqual(copied["timestamp"], 12.5)
        self.assertEqual(copied["capture_timestamp"], 12.5)
        self.assertEqual(copied["processing_completed_timestamp"], 12.5)
        self.assertEqual(copied["processing_latency_s"], 0.0)


if __name__ == "__main__":
    unittest.main()

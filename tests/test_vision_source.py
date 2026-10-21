import argparse
import unittest

from vision import (
    DEFAULT_CONFIDENCE,
    SimulatorConnection,
    confidence_threshold,
    open_video_capture,
    select_video_source,
)


class VideoSourceTest(unittest.TestCase):
    def test_camera_zero_remains_default_source(self):
        self.assertEqual(select_video_source(0, None), 0)

    def test_camera_number_is_preserved(self):
        self.assertEqual(select_video_source(2, None), 2)

    def test_video_url_takes_precedence(self):
        url = "http://127.0.0.1:8766/stream.mjpg"
        self.assertEqual(select_video_source(0, url), url)

    def test_default_confidence_preserves_existing_behavior(self):
        self.assertEqual(DEFAULT_CONFIDENCE, 0.5)

    def test_confidence_threshold_accepts_evaluation_value(self):
        self.assertEqual(confidence_threshold("0.25"), 0.25)

    def test_confidence_threshold_rejects_out_of_range_value(self):
        with self.assertRaises(argparse.ArgumentTypeError):
            confidence_threshold("1.01")

    def test_http_timeout_is_configurable_without_changing_default(self):
        self.assertEqual(SimulatorConnection("http://example.invalid").timeout, 0.2)
        self.assertEqual(
            SimulatorConnection("http://example.invalid", 2.0).timeout,
            2.0,
        )

    def test_network_capture_helper_is_available_for_reconnects(self):
        self.assertTrue(callable(open_video_capture))


if __name__ == "__main__":
    unittest.main()

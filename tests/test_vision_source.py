import argparse
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from vision import (
    DEFAULT_CONFIDENCE,
    DEFAULT_IMAGE_SIZE,
    SimulatorConnection,
    confidence_threshold,
    create_aruco_detector,
    detect_aruco,
    open_video_capture,
    model_sha256,
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
        self.assertEqual(DEFAULT_IMAGE_SIZE, 640)

    def test_model_hash_is_reproducible(self):
        import tempfile

        with tempfile.NamedTemporaryFile() as stream:
            stream.write(b"fixed-model")
            stream.flush()
            self.assertEqual(
                model_sha256(stream.name),
                "d909a4d88bdba9e54ae4133d89cca765299a5beae4ec190d4210574c5f7a8329",
            )

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

    @patch("vision.cv2.VideoCapture")
    @patch("vision.platform.system", return_value="Darwin")
    def test_macos_camera_uses_avfoundation(self, _system, video_capture):
        open_video_capture(0)
        video_capture.assert_called_once_with(0, cv2.CAP_AVFOUNDATION)

    def test_aruco_is_opt_in_and_uses_existing_payload(self):
        source = (__import__("pathlib").Path(__file__).resolve().parents[1] / "vision.py").read_text()
        self.assertIn('default=-1', source)
        self.assertIn('"aruco_detections": aruco_detections', source)
        self.assertIn('"capture_timestamp": capture_timestamp', source)
        self.assertIn('"processing_completed_timestamp": processing_completed_timestamp', source)
        self.assertIn('"timestamp": capture_timestamp', source)
        self.assertIn("imgsz=args.imgsz", source)

    def test_controller_preserves_log_values_before_direct_rejection(self):
        source = (
            __import__("pathlib").Path(__file__).resolve().parents[1]
            / "simulation"
            / "vision_turn_controller.py"
        ).read_text()
        self.assertLess(
            source.index("pixel_x, pixel_y = self.target_pixel"),
            source.index("self.on_target(point)"),
        )

    def test_synthetic_aruco_detection_and_id_filter(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 240)
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[120:360, 200:440] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        detector = create_aruco_detector("DICT_4X4_50")
        matches, _corners, _ids = detect_aruco(detector, frame, 0)
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0]["marker_id"], 0)
        mismatch, _corners, _ids = detect_aruco(detector, frame, 1)
        self.assertEqual(mismatch, [])


if __name__ == "__main__":
    unittest.main()

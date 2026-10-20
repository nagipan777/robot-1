import unittest

from unittest.mock import patch

from collect_dirt_dataset import open_camera, yolo_label


class CollectDirtDatasetTests(unittest.TestCase):
    def test_positive_box_is_written_as_litter_class_zero(self):
        self.assertEqual(
            yolo_label((128, 96, 256, 192), (1280, 960)),
            "0 0.200000 0.200000 0.200000 0.200000\n",
        )

    def test_empty_positive_box_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "non-empty litter box"):
            yolo_label((0, 0, 0, 0), (1280, 960))

    @patch("collect_dirt_dataset.cv2.VideoCapture")
    @patch("collect_dirt_dataset.platform.system", return_value="Darwin")
    def test_macos_camera_uses_avfoundation(self, _system, video_capture):
        open_camera(0)
        video_capture.assert_called_once_with(0, 1200)


if __name__ == "__main__":
    unittest.main()

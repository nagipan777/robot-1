import unittest

from vision import select_video_source


class VideoSourceTest(unittest.TestCase):
    def test_camera_zero_remains_default_source(self):
        self.assertEqual(select_video_source(0, None), 0)

    def test_camera_number_is_preserved(self):
        self.assertEqual(select_video_source(2, None), 2)

    def test_video_url_takes_precedence(self):
        url = "http://127.0.0.1:8766/stream.mjpg"
        self.assertEqual(select_video_source(0, url), url)


if __name__ == "__main__":
    unittest.main()

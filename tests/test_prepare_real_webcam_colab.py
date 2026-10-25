import tempfile
import unittest
from pathlib import Path

from prepare_real_webcam_colab import inspect_real_dataset


class PrepareRealWebcamColabTests(unittest.TestCase):
    def make_dataset(self, root):
        for split in ("train", "val", "test"):
            (root / "images" / split).mkdir(parents=True)
            (root / "labels" / split).mkdir(parents=True)
            for kind, label in (("positive", "0 0.5 0.5 0.2 0.2\n"), ("negative", "")):
                stem = f"{split}_{kind}"
                (root / "images" / split / f"{stem}.jpg").write_bytes(stem.encode())
                (root / "labels" / split / f"{stem}.txt").write_text(label)

    def test_accepts_positive_and_negative_in_every_split(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            records, counts, excluded = inspect_real_dataset(root)
        self.assertEqual(len(records), 6)
        self.assertEqual(counts["test"]["positive"], 1)
        self.assertEqual(counts["test"]["negative"], 1)
        self.assertEqual(excluded, set())

    def test_refuses_incomplete_split(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            (root / "labels" / "test" / "test_negative.txt").unlink()
            (root / "images" / "test" / "test_negative.jpg").unlink()
            with self.assertRaisesRegex(ValueError, "both positive and negative"):
                inspect_real_dataset(root)


if __name__ == "__main__":
    unittest.main()

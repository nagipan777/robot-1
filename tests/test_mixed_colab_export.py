import tempfile
import unittest
from pathlib import Path

from build_mixed_colab_export import (
    TACO_SPLIT_PLAN,
    deterministic_select,
    taco_group,
    validate_label,
)


class MixedColabExportTests(unittest.TestCase):
    def test_taco_groups_are_disjoint(self):
        groups = [plan["groups"] for plan in TACO_SPLIT_PLAN.values()]
        self.assertFalse(groups[0] & groups[1])
        self.assertFalse(groups[0] & groups[2])
        self.assertFalse(groups[1] & groups[2])

    def test_taco_group_parses_both_filename_styles(self):
        self.assertEqual(taco_group(Path("batch_10_000001.jpg")), 10)
        self.assertEqual(taco_group(Path("batch_3_IMG_4852.JPG")), 3)

    def test_selection_is_deterministic(self):
        paths = [Path(f"batch_1_{number:06d}.jpg") for number in range(10)]
        self.assertEqual(
            deterministic_select(paths, 4, "train"),
            deterministic_select(list(reversed(paths)), 4, "train"),
        )

    def test_label_validation_accepts_class_zero_and_empty(self):
        empty = validate_label(b"", Path("empty.txt"))
        label = validate_label(b"0 0.5 0.5 0.2 0.3\n", Path("label.txt"))
        self.assertTrue(empty["empty"])
        self.assertEqual(label["boxes"], 1)

    def test_label_validation_rejects_other_classes(self):
        with self.assertRaises(ValueError):
            validate_label(b"1 0.5 0.5 0.2 0.3\n", Path("bad.txt"))


if __name__ == "__main__":
    unittest.main()

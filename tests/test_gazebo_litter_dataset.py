import unittest

from build_gazebo_litter_dataset import KINDS, label_line, schedule


class GazeboLitterDatasetTests(unittest.TestCase):
    def test_schedule_counts_and_scene_split_isolation(self):
        rows = schedule()
        self.assertEqual(len(rows), 120)
        for kind in KINDS:
            self.assertEqual(sum(row["kind"] == kind for row in rows), 40)
        scene_splits = {}
        for row in rows:
            scene_splits.setdefault(row["scene"], set()).add(row["split"])
        self.assertTrue(all(len(splits) == 1 for splits in scene_splits.values()))
        self.assertEqual(sum(row["fixed_holdout"] for row in rows), 9)

    def test_yolo_label_is_normalized(self):
        values = label_line([64, 48, 128, 96], 640, 480).split()
        self.assertEqual(values[0], "0")
        self.assertEqual([float(value) for value in values[1:]], [0.15, 0.15, 0.1, 0.1])


if __name__ == "__main__":
    unittest.main()

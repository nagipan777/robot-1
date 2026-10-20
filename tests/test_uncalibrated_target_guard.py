import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class UncalibratedTargetGuardTests(unittest.TestCase):
    def test_metric_target_requires_calibration_before_transform(self):
        source = (ROOT / "simulation/vision_turn_controller.py").read_text()
        guard = source.index("calibration_allowed =")
        transform = source.index("self.target_odom = transform_point", guard)
        publish = source.index("self.publish_target", transform)
        self.assertLess(guard, transform)
        self.assertLess(transform, publish)
        self.assertIn("refusing to publish a metric litter target", source)


if __name__ == "__main__":
    unittest.main()

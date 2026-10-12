import unittest
import math

from navigation import calculate_goal


class TestCalculateGoal(unittest.TestCase):

    def test_forward(self):
        distance, angle = calculate_goal(
            0, 0, 0, 1, 0
        )

        self.assertAlmostEqual(distance, 1.0)
        self.assertAlmostEqual(angle, 0.0)

    def test_left(self):
        distance, angle = calculate_goal(
            0, 0, 0, 0, 1
        )

        self.assertAlmostEqual(distance, 1.0)
        self.assertAlmostEqual(angle, math.pi / 2)

    def test_diagonal(self):
        distance, angle = calculate_goal(
            0, 0, 0, 3, 4
        )

        self.assertAlmostEqual(distance, 5.0)
        self.assertAlmostEqual(
            angle,
            math.atan2(4, 3)
        )

    def test_angle_wrap(self):
        distance, angle = calculate_goal(
            0, 0,
            math.radians(170),
            -1, -math.tan(math.radians(10))
        )

        self.assertAlmostEqual(
            distance,
            math.hypot(1, math.tan(math.radians(10)))
        )
        self.assertAlmostEqual(
            angle,
            math.radians(20),
            places=6
        )


if __name__ == "__main__":
    unittest.main()
import unittest

import goal_math
import navigation


class NavigationCompatibilityTests(unittest.TestCase):
    def test_goal_math_uses_canonical_navigation_function(self):
        self.assertIs(goal_math.calculate_goal, navigation.calculate_goal)


if __name__ == "__main__":
    unittest.main()

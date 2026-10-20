import unittest
from pathlib import Path
from unittest import mock

import validate_pai_output


class PaiOutputValidationTests(unittest.TestCase):
    def test_three_models_are_kept_separate(self):
        self.assertEqual(
            set(validate_pai_output.MODELS),
            {"taco_original", "gazebo_only", "taco_gazebo_mixed"},
        )
        self.assertEqual(len(set(validate_pai_output.MODELS.values())), 3)

    def test_yaml_uses_test_split_and_one_litter_class(self):
        path = mock.MagicMock(spec=Path)
        validate_pai_output.write_yaml(path, Path("/tmp/dataset"))
        text = path.write_text.call_args.args[0]
        self.assertIn("test: images/test", text)
        self.assertIn("0: litter", text)


if __name__ == "__main__":
    unittest.main()

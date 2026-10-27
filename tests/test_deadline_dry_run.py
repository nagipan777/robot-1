import pathlib
import subprocess
import sys
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class DeadlineDryRunTests(unittest.TestCase):
    def test_hardware_free_evidence_run_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "simulation" / "deadline_dry_run.py"),
                    "--output-dir",
                    directory,
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = (pathlib.Path(directory) / "summary.json").read_text()
            self.assertIn('"pass": true', summary)
            self.assertIn('"post_deadline_socket_sends": 0', summary)
            self.assertTrue((pathlib.Path(directory) / "events.jsonl").is_file())


if __name__ == "__main__":
    unittest.main()

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SimulationDdsConfigurationTests(unittest.TestCase):
    def test_colima_container_disables_only_fastdds_shared_memory(self):
        compose = (ROOT / "simulation" / "compose.yaml").read_text()
        self.assertIn("FASTDDS_BUILTIN_TRANSPORTS: UDPv4", compose)
        self.assertIn('ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST', compose)
        self.assertIn('ROS_DOMAIN_ID: "33"', compose)


if __name__ == "__main__":
    unittest.main()

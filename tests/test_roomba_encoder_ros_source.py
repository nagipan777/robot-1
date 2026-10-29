import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RoombaEncoderRosSourceTests(unittest.TestCase):
    def test_docker_installs_ros_encoder_nodes(self):
        dockerfile = (ROOT / "simulation/Dockerfile").read_text()
        self.assertIn(
            "COPY roomba_encoder_ros_node.py /usr/local/bin/roomba-encoder-ros-node",
            dockerfile,
        )
        self.assertIn(
            "COPY roomba_encoder_ros_e2e.py /usr/local/bin/roomba-encoder-ros-e2e",
            dockerfile,
        )

    def test_counts_order_and_invalid_suppression_are_explicit(self):
        source = (ROOT / "simulation/roomba_encoder_ros_node.py").read_text()
        self.assertIn("counts.data = [left, right]", source)
        error_block = source.split("except Exception as error:", 1)[1]
        self.assertNotIn("counts_publisher.publish", error_block)
        self.assertIn('state="disconnected"', error_block)

    def test_phase1_protocol_and_pico_are_unchanged_in_phase2_commit(self):
        transport = (ROOT / "simulation/roomba_tcp_transport.py").read_text()
        relay = (ROOT / "pico/pico_relay.py").read_text()
        self.assertIn("DIAGNOSTIC_ENCODERS = 0x07", transport)
        self.assertIn("DIAGNOSTIC_ENCODERS_RESPONSE = 0x87", transport)
        self.assertIn("DIAGNOSTIC_ENCODERS = 0x07", relay)
        self.assertIn("ALLOW_NONZERO_DRIVE = False", relay)


if __name__ == "__main__":
    unittest.main()

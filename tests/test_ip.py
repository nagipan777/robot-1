import io
import socket
import unittest
from unittest import mock

import test_ip


class TestIpTests(unittest.TestCase):
    @mock.patch("test_ip.socket.create_connection")
    def test_default_is_connection_only(self, create_connection):
        connection = create_connection.return_value
        self.assertEqual(test_ip.run("192.0.2.1", 9000), 0)
        connection.sendall.assert_not_called()
        connection.close.assert_called_once()

    @mock.patch("test_ip.time.sleep")
    @mock.patch("test_ip.time.monotonic", side_effect=[0.0, 0.0, 2.0])
    @mock.patch("test_ip.socket.create_connection")
    def test_drive_requires_flag_and_sends_stop(self, create_connection, monotonic, sleep):
        connection = create_connection.return_value
        self.assertEqual(test_ip.run("192.0.2.1", 9000, drive_test=True), 0)
        forward = test_ip.drive_packet(50, 50)
        stop = test_ip.drive_packet(0, 0)
        self.assertEqual(connection.sendall.call_args_list[0], mock.call(forward))
        self.assertEqual(connection.sendall.call_args_list[-1], mock.call(stop))

    @mock.patch(
        "test_ip.socket.create_connection",
        side_effect=socket.timeout("timed out"),
    )
    def test_connection_error_is_reported_without_traceback(self, create_connection):
        error = io.StringIO()
        with mock.patch("sys.stderr", error):
            self.assertEqual(test_ip.run("192.0.2.1", 9000), 1)
        self.assertIn("connection failed", error.getvalue())


if __name__ == "__main__":
    unittest.main()

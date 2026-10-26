import pathlib
import unittest


SOURCE = pathlib.Path(__file__).resolve().parents[1] / "pico" / "pico_relay.py"


class PicoRelaySafetyTests(unittest.TestCase):
    def test_final_demo_uses_uart1_gp4_gp5_and_keeps_drive_locked(self):
        source = SOURCE.read_text()
        self.assertIn("uart = machine.UART(\n    1,", source)
        self.assertIn("tx=machine.Pin(4)", source)
        self.assertIn("rx=machine.Pin(5)", source)
        self.assertIn("ALLOW_NONZERO_DRIVE = False", source)
        self.assertNotIn("UDP", source)

    def test_network_waits_are_finite_and_reconnect(self):
        source = SOURCE.read_text()
        self.assertIn("WIFI_CONNECT_TIMEOUT_MS", source)
        self.assertIn("server.settimeout(ACCEPT_TIMEOUT_S)", source)
        self.assertIn("while wlan.isconnected()", source)
        self.assertIn("Wi-Fi lost; rebuilding listener", source)
        self.assertEqual(source.count('print("Wi-Fi OK; IP:", wlan.ifconfig()[0])'), 2)

    def test_disconnect_and_command_timeout_stop_roomba(self):
        source = SOURCE.read_text()
        self.assertIn("COMMAND_TIMEOUT_MS = 500", source)
        self.assertGreaterEqual(source.count("stop_roomba()"), 5)
        self.assertIn("finally:\n        stop_roomba()", source)
        self.assertIn("watchdog STOP: fired_at_ms=", source)
        self.assertIn("uart_written", source)
        self.assertIn(
            'except KeyboardInterrupt:\n        print("stopped by user")', source
        )

    def test_ready_is_sent_after_roomba_initialization(self):
        source = SOURCE.read_text()
        self.assertIn('READY = b"READY\\n"', source)
        receive_client = source[source.index("def receive_client"):]
        self.assertLess(
            receive_client.index("roomba_init()"),
            receive_client.index("send_all(connection, READY)"),
        )
        self.assertLess(
            receive_client.index("send_all(connection, READY)"),
            receive_client.index("connection.setblocking(False)"),
        )
        self.assertIn("disconnect STOP: at_ms=", receive_client)
        self.assertIn('"connected: {} at_ms={}"', receive_client)
        self.assertIn('"relay ready: at_ms={} bytes={}"', receive_client)
        self.assertIn("roomba_init()", receive_client)
        self.assertIn("send_all(connection, READY)", receive_client)
        self.assertIn("diagnostics remain locked", source)
        self.assertNotIn("if not roomba_init():", receive_client)

    def test_only_arm_or_generation_drive_extends_watchdog(self):
        source = SOURCE.read_text()
        self.assertIn("def forward_valid_frames(received, connection):", source)
        self.assertIn("abs(right) > OI_MAX_WHEEL_SPEED_MM_S", source)
        self.assertIn(
            "received, forwarded = forward_valid_frames(received, connection)", source
        )
        self.assertIn("return bytearray(received[offset:]), forwarded", source)
        self.assertIn("Only a CRC-checked,", source)
        self.assertNotIn("del received", source)
        self.assertNotIn("if data:\n                    uart.write(data)", source)
        self.assertIn("normal STOP forwarded: at_ms=", source)
        self.assertIn("drive forwarded: at_ms=", source)


if __name__ == "__main__":
    unittest.main()

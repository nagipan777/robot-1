import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RoombaBridgeTcpIntegrationTests(unittest.TestCase):
    def test_bridge_uses_dedicated_topic_and_is_stop_only_by_default(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        self.assertIn(
            'declare_parameter("command_topic", "/roomba/cmd_vel_unstamped")',
            source,
        )
        self.assertIn('declare_parameter("drive_enabled", False)', source)
        self.assertIn('declare_parameter("watchdog_test_mode", False)', source)
        self.assertIn(
            "watchdog_test_mode=true requires drive_enabled=true",
            source,
        )
        self.assertIn('declare_parameter("max_speed_mm_s", 50)', source)
        self.assertIn("self.command_topic, self.on_cmd", source)
        self.assertIn('declare_parameter("arm_topic", "/roomba/arm_generation")', source)
        self.assertIn("self.arm_topic, self.on_arm", source)
        self.assertIn("if rclpy.ok():", source)
        self.assertIn("COMMAND_QOS", source)
        self.assertIn("topic=%s type=geometry_msgs/msg/Twist", source)

    def test_command_qos_keeps_only_latest_reliable_sample(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        qos = source[source.index("COMMAND_QOS =") : source.index("class RoombaBridgeTcp")]
        self.assertIn("history=HistoryPolicy.KEEP_LAST", qos)
        self.assertIn("depth=1", qos)
        self.assertIn("reliability=ReliabilityPolicy.RELIABLE", qos)
        self.assertIn("durability=DurabilityPolicy.VOLATILE", qos)

    def test_watchdog_test_keeps_initial_stop_and_safe_diagnostics(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        connect_start = source.index("    def connect(self):")
        connect_end = source.index("    def on_cmd", connect_start)
        connect = source[connect_start:connect_end]
        self.assertIn("self.transport.verify_stop()", connect)
        self.assertIn("self.transport.query_oi_mode()", connect)
        self.assertNotIn("watchdog_test_mode", connect)

    def test_watchdog_test_sends_each_fresh_command_once_and_has_fallback(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        method = source[source.index("    def tick_watchdog_test") : source.index("    def send_stop")]
        self.assertIn("self.watchdog_test_state.poll()", method)
        self.assertEqual(method.count("self.transport.send_twist("), 1)
        self.assertIn("bridge will send no TCP/OI", method)
        self.assertIn('"traffic until %.2fs fallback', method)
        self.assertIn("fallback STOP", method)
        self.assertIn("RoombaDiagnosticError", method)

    def test_watchdog_trial_does_not_issue_live_diagnostics(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        start = source.index("    def tick_watchdog_test")
        method = source[start : source.index("    def send_stop", start)]
        self.assertNotIn("query_oi_mode", method)
        self.assertIn('"are suspended during the bounded watchdog trial"', method)

    def test_reconnect_clears_old_command(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        connect = source[source.index("    def connect(self):"):source.index("    def on_cmd", source.index("    def connect(self):"))]
        self.assertIn("self.clear_command()", connect)
        self.assertIn("self.clear_motion_authorization()", connect)
        self.assertIn("self.transport.connect()", connect)

    def test_nonzero_commands_require_stop_ack_and_safe_mode_gate(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        connect_start = source.index("    def connect(self):")
        on_cmd_start = source.index("    def on_cmd", connect_start)
        connect = source[connect_start:on_cmd_start]
        tick_start = source.index("    def tick(self):", on_cmd_start)
        tick = source[tick_start:source.index("    def send_stop", tick_start)]
        on_cmd = source[on_cmd_start:tick_start]

        self.assertIn("status, right, left = self.transport.verify_stop()", connect)
        self.assertIn("mode = self.transport.query_oi_mode()", connect)
        self.assertIn("self.safe_gate.establish(status, right, left, mode)", connect)
        self.assertIn("self.safe_gate.permits", on_cmd)
        self.assertIn("Rejected non-zero command: drive_enabled=false", on_cmd)
        self.assertIn("query_oi_mode(live=True)", tick)
        self.assertIn("self.safe_gate.observe_mode(mode)", tick)
        self.assertIn("self.handle_safety_failure(error)", tick)

    def test_drive_ack_timeout_uses_existing_fail_safe_handler(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        tick_start = source.index("    def tick(self):")
        tick = source[tick_start : source.index("    def tick_watchdog_test", tick_start)]
        send_start = tick.index("self.transport.send_twist(")
        send_block = tick[send_start:]
        self.assertIn("RoombaDiagnosticError", send_block)
        self.assertIn("self.handle_send_failure(error)", send_block)

    def test_stop_is_not_blocked_by_safe_gate(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        on_cmd_start = source.index("    def on_cmd")
        on_cmd = source[on_cmd_start:source.index("    def tick", on_cmd_start)]
        zero_branch = on_cmd.index(
            "if msg.linear.x == 0.0 and msg.angular.z == 0.0:"
        )
        gate_check = on_cmd.index("self.safe_gate.permits")
        self.assertLess(zero_branch, gate_check)
        self.assertIn("self.send_stop()", on_cmd[zero_branch:gate_check])

    def test_arm_requires_existing_host_and_safe_gates(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        start = source.index("    def on_arm")
        method = source[start : source.index("    def tick", start)]
        self.assertIn("if not self.drive_enabled", method)
        self.assertIn("not self.safe_gate.authorized", method)
        self.assertIn("self.transport.arm(generation)", method)
        self.assertIn("self.arm_deadline", method)
        self.assertNotIn("self.connect()", method)

    def test_both_nonzero_drive_locks_remain_disabled(self):
        bridge = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        relay = (ROOT / "pico" / "pico_relay.py").read_text()
        self.assertIn('declare_parameter("drive_enabled", False)', bridge)
        self.assertIn("ALLOW_NONZERO_DRIVE = False", relay)

    def test_container_installs_bridge_without_autostart(self):
        dockerfile = (ROOT / "simulation" / "Dockerfile").read_text()
        supervisor = (ROOT / "simulation" / "supervisord.conf").read_text()
        sim_script = (ROOT / "simulation" / "sim.sh").read_text()
        self.assertIn("COPY roomba_bridge_tcp.py /usr/local/bin/roomba-bridge-tcp", dockerfile)
        self.assertIn("COPY roomba_tcp_transport.py", dockerfile)
        self.assertIn("COPY differential_drive.py", dockerfile)
        self.assertNotIn("roomba-bridge-tcp", supervisor)
        self.assertIn("roomba-tcp PICO_IP [PORT]", sim_script)

    def test_roomba_tcp_command_explicitly_forces_stop_only_dedicated_topic(self):
        sim_script = (ROOT / "simulation" / "sim.sh").read_text()
        start = sim_script.index("  roomba-tcp)")
        case = sim_script[start : sim_script.index("  roomba-tcp-diagnostic)", start)]
        self.assertIn("-p drive_enabled:=false", case)
        self.assertIn("-p watchdog_test_mode:=false", case)
        self.assertIn("-p command_topic:=/roomba/cmd_vel_unstamped", case)
        self.assertNotIn("drive_enabled:=true", case)

    def test_diagnostic_command_remains_stop_only(self):
        sim_script = (ROOT / "simulation" / "sim.sh").read_text()
        start = sim_script.index("  roomba-tcp-diagnostic)")
        case = sim_script[start : sim_script.index("  roomba-short-e-bridge)", start)]
        self.assertIn("-p drive_enabled:=false", case)
        self.assertIn("-p diagnostic_on_connect:=true", case)
        self.assertIn("-p watchdog_test_mode:=false", case)
        self.assertIn("-p command_topic:=/roomba/cmd_vel_unstamped", case)
        self.assertNotIn("drive_enabled:=true", case)

    def test_watchdog_launcher_requires_both_locks_and_bounds_speed(self):
        sim_script = (ROOT / "simulation" / "sim.sh").read_text()
        start = sim_script.index("  roomba-tcp-watchdog)")
        case = sim_script[start : sim_script.index("  roomba-wheel-lift-test)", start)]
        self.assertIn('"$3" != "WHEELS_LIFTED"', case)
        self.assertIn("-p drive_enabled:=true", case)
        self.assertIn("-p diagnostic_on_connect:=true", case)
        self.assertIn("-p watchdog_test_mode:=true", case)
        self.assertIn("-p watchdog_test_silence:=0.75", case)
        self.assertIn("-p watchdog_test_confirmation:=WHEELS_LIFTED", case)
        self.assertIn("-p max_speed_mm_s:=20", case)

    def test_short_e_launcher_requires_wheel_lift_and_hard_bounds(self):
        sim_script = (ROOT / "simulation" / "sim.sh").read_text()
        start = sim_script.index("  roomba-short-e-bridge)")
        case = sim_script[start : sim_script.index("  roomba-tcp-watchdog)", start)]
        self.assertIn('"$3" != "WHEELS_LIFTED"', case)
        self.assertIn("-p drive_enabled:=true", case)
        self.assertIn("-p diagnostic_on_connect:=true", case)
        self.assertIn("-p watchdog_test_mode:=false", case)
        self.assertIn("-p max_speed_mm_s:=20", case)
        self.assertIn("-p command_timeout:=0.5", case)
        self.assertIn("-p max_drive_session_duration:=0.4", case)
        self.assertIn("-p command_topic:=/roomba/cmd_vel_unstamped", case)

    def test_bounded_drive_gate_defaults_off_and_revokes_arm_on_deadline(self):
        source = (ROOT / "simulation" / "roomba_bridge_tcp.py").read_text()
        self.assertIn('declare_parameter("max_drive_session_duration", 0.0)', source)
        self.assertIn("self.drive_session_limiter.poll()", source)
        self.assertIn("self.expire_bounded_drive()", source)
        self.assertIn("bounded drive deadline reached: STOP sent, ARM revoked", source)
        self.assertIn("bounded Drive ACK: status=0x03", source)


if __name__ == "__main__":
    unittest.main()

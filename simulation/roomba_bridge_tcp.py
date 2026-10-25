#!/usr/bin/env python3
"""ROS 2 bridge: Twist -> Pico W (TCP) -> Roomba 680.

実行例:
    # Safe default: connect, verify READY, and send STOP only.
    python3 roomba_bridge_tcp.py --ros-args -p host:=192.168.0.32

Non-zero commands require the explicit ``drive_enabled:=true`` parameter.

安全:
    - 既定では非ゼロ速度を送らない
    - 速度は max_speed_mm_s で比例制限
    - command_timeout 秒間、指令が来なければ停止指令を送る
    - Pico W 側も、500ms 指令が途絶えると自動停止する
    - 再接続後は新しいTwistを受信するまで停止を維持する
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from differential_drive import twist_to_wheel_speeds
from roomba_tcp_transport import (
    CommandState,
    DriveSessionLimiter,
    RoombaDiagnosticError,
    RoombaTcpTransport,
    SafeModeGate,
    WatchdogTestCommandState,
)


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    # Velocity is state, not an event. Retaining old samples can replay stale
    # motion after synchronous TCP/UART work blocks the executor. Keep only
    # the newest command (including STOP), while retaining reliable delivery.
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class RoombaBridgeTcp(Node):
    def __init__(self):
        super().__init__("roomba_bridge_tcp")
        self.declare_parameter("host", "192.168.0.32")
        self.declare_parameter("port", 9000)
        self.declare_parameter("command_topic", "/roomba/cmd_vel_unstamped")
        self.declare_parameter("arm_topic", "/roomba/arm_generation")
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("diagnostic_on_connect", False)
        self.declare_parameter("wheel_base", 0.235)  # 車輪間距離 [m]
        self.declare_parameter("max_speed_mm_s", 50)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("safe_check_interval", 0.25)
        self.declare_parameter("arm_hold_timeout", 1.0)
        # Disabled by default.  The short-E wheel-lift launcher sets this to a
        # conservative sub-second value.  Values above one second are refused.
        self.declare_parameter("max_drive_session_duration", 0.0)
        # Compatibility guard only. The normal bridge never disables its
        # host-side timeout; Pico-only watchdog tests require a separate path.
        self.declare_parameter("watchdog_test_mode", False)
        self.declare_parameter("watchdog_test_silence", 0.75)
        self.declare_parameter("watchdog_test_confirmation", "")

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.command_topic = self.get_parameter("command_topic").value
        self.arm_topic = self.get_parameter("arm_topic").value
        self.drive_enabled = self.get_parameter("drive_enabled").value
        self.diagnostic_on_connect = self.get_parameter("diagnostic_on_connect").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.max_speed_mm_s = self.get_parameter("max_speed_mm_s").value
        self.command_timeout = self.get_parameter("command_timeout").value
        self.safe_check_interval = self.get_parameter("safe_check_interval").value
        self.arm_hold_timeout = self.get_parameter("arm_hold_timeout").value
        self.max_drive_session_duration = self.get_parameter(
            "max_drive_session_duration"
        ).value
        self.watchdog_test_mode = self.get_parameter("watchdog_test_mode").value
        self.watchdog_test_silence = self.get_parameter(
            "watchdog_test_silence"
        ).value
        self.watchdog_test_confirmation = self.get_parameter(
            "watchdog_test_confirmation"
        ).value
        if self.wheel_base <= 0.0:
            raise ValueError("wheel_base must be positive")
        if not 1 <= self.max_speed_mm_s <= 500:
            raise ValueError("max_speed_mm_s must be from 1 to 500")
        if self.command_timeout <= 0.0:
            raise ValueError("command_timeout must be positive")
        if self.safe_check_interval <= 0.0:
            raise ValueError("safe_check_interval must be positive")
        if not 0.2 <= self.arm_hold_timeout <= 2.0:
            raise ValueError("arm_hold_timeout must be from 0.2 to 2.0 seconds")
        if self.watchdog_test_mode and not (
            self.drive_enabled
            and self.watchdog_test_confirmation == "WHEELS_LIFTED"
        ):
            raise ValueError(
                "watchdog_test_mode=true requires drive_enabled=true and "
                "watchdog_test_confirmation=WHEELS_LIFTED"
            )
        if not 0.5 < self.watchdog_test_silence <= 1.0:
            raise ValueError("watchdog_test_silence must be > 0.5 and <= 1.0")

        self.transport = RoombaTcpTransport(self.host, self.port)
        self.command_state = CommandState(
            self.drive_enabled, self.command_timeout, time.monotonic
        )
        self.drive_session_limiter = DriveSessionLimiter(
            self.max_drive_session_duration, time.monotonic
        )
        self.watchdog_test_state = WatchdogTestCommandState(
            self.watchdog_test_silence, time.monotonic
        )
        self.safe_gate = SafeModeGate()
        self.disabled_rejection_logged = False
        self.next_safe_check = 0.0
        self.next_retry = 0.0
        self.watchdog_preflight_logged = False
        self.arm_deadline = 0.0
        self.bounded_drive_rejection_logged = False
        self.create_subscription(Twist, self.command_topic, self.on_cmd, COMMAND_QOS)
        self.create_subscription(UInt32, self.arm_topic, self.on_arm, COMMAND_QOS)
        self.create_timer(0.05, self.tick)  # 20Hz
        self.connect()

        if not self.drive_enabled:
            self.get_logger().warn(
                "STOP-only mode: drive_enabled=false; non-zero commands are disabled"
            )

    def clear_command(self):
        self.command_state.clear()
        self.watchdog_test_state.clear()
        self.watchdog_preflight_logged = False

    def clear_motion_authorization(self):
        self.safe_gate.reset()
        self.next_safe_check = 0.0
        self.arm_deadline = 0.0

    def connect(self):
        # Invalidate commands received before or during a reconnect.  Only a
        # new callback after READY may authorize movement.
        self.clear_command()
        self.clear_motion_authorization()
        try:
            self.transport.connect()
            self.get_logger().info(
                f"Connected to Pico W {self.host}:{self.port}; relay READY; STOP sent"
            )
            if self.drive_enabled or self.diagnostic_on_connect:
                status, right, left = self.transport.verify_stop()
                self.get_logger().info(
                    "STOP confirmed: host_sent=true, pico_validated=true, "
                    "uart_written=true, roomba_requested_right=%d, "
                    "roomba_requested_left=%d, ack=0x%02x"
                    % (right, left, status)
                )
                mode = self.transport.query_oi_mode()
                mode_name = ("Off", "Passive", "Safe", "Full")[mode]
                if mode == 2:
                    self.get_logger().info("Roomba OI mode: 2 (Safe)")
                else:
                    raise RoombaDiagnosticError(
                        f"Roomba OI mode is not Safe: {mode} ({mode_name})"
                    )
                self.safe_gate.establish(status, right, left, mode)
                self.next_safe_check = time.monotonic() + self.safe_check_interval
        except (OSError, ConnectionError, RoombaDiagnosticError) as error:
            self.clear_motion_authorization()
            self.transport.close(send_stop=False)
            self.next_retry = time.monotonic() + 2.0
            self.get_logger().warn(f"Pico W に接続できません: {error}")

    def on_cmd(self, msg: Twist):
        if not self.drive_enabled:
            self.clear_command()
            is_nonzero = msg.linear.x != 0.0 or msg.angular.z != 0.0
            if is_nonzero and not self.disabled_rejection_logged:
                self.disabled_rejection_logged = True
                self.get_logger().warn(
                    "Rejected non-zero command: drive_enabled=false; "
                    "topic=%s type=geometry_msgs/msg/Twist "
                    "linear.x=%.6f angular.z=%.6f"
                    % (self.command_topic, msg.linear.x, msg.angular.z)
                )
            elif not is_nonzero:
                # Permit one confirmation log for each separately bounded test.
                self.disabled_rejection_logged = False
            return
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.clear_command()
            self.drive_session_limiter.latch_stop()
            if self.transport.connected:
                self.get_logger().info(
                    "normal STOP received and sent; monotonic=%.6f"
                    % time.monotonic()
                )
                self.send_stop()
            return
        if not self.transport.connected:
            # A command received before READY must not become valid later.
            self.clear_command()
            return
        if not self.safe_gate.permits(msg.linear.x, msg.angular.z):
            self.clear_command()
            self.get_logger().warn(
                "Rejected non-zero command: STOP ACK and OI Safe are not confirmed"
            )
            return
        if self.transport.armed_generation is None:
            self.clear_command()
            self.get_logger().warn(
                "Rejected non-zero command: explicit ARM generation is required"
            )
            return
        session_event = self.drive_session_limiter.observe_nonzero()
        if session_event in ("expired", "latched"):
            self.clear_command()
            if session_event == "expired":
                self.expire_bounded_drive()
            elif not self.bounded_drive_rejection_logged:
                self.bounded_drive_rejection_logged = True
                self.get_logger().warn(
                    "Rejected non-zero command: bounded drive STOP is latched; "
                    "a new explicit ARM is required"
                )
            return
        if session_event == "started":
            self.get_logger().warn(
                "bounded drive session started: limit=%.3fs monotonic=%.6f"
                % (self.max_drive_session_duration, time.monotonic())
            )
        if self.watchdog_test_mode:
            self.watchdog_test_state.update(msg.linear.x, msg.angular.z)
        else:
            self.command_state.update(msg.linear.x, msg.angular.z)
        self.arm_deadline = 0.0

    def on_arm(self, msg: UInt32):
        generation = int(msg.data)
        if not self.drive_enabled:
            self.get_logger().warn("ARM rejected: drive_enabled=false")
            return
        if not self.transport.connected or not self.safe_gate.authorized:
            self.get_logger().warn(
                "ARM rejected: TCP connection and SafeModeGate are required"
            )
            return
        try:
            self.transport.arm(generation)
            self.drive_session_limiter.arm()
            self.bounded_drive_rejection_logged = False
            self.clear_command()
            self.arm_deadline = time.monotonic() + self.arm_hold_timeout
            self.get_logger().warn(
                "ARM accepted for this TCP session: generation=%d" % generation
            )
        except (OSError, ConnectionError, RoombaDiagnosticError, TypeError, ValueError) as error:
            self.handle_safety_failure(error)

    def tick(self):
        if not self.transport.connected:
            if time.monotonic() >= self.next_retry:
                self.connect()
            return

        if self.watchdog_test_mode:
            self.tick_watchdog_test()
            return

        if self.drive_session_limiter.poll():
            self.expire_bounded_drive()
            return

        command = self.command_state.current()
        if command is None:
            if (
                self.transport.armed_generation is not None
                and time.monotonic() < self.arm_deadline
            ):
                return
            self.send_stop()
            return

        if command != (0.0, 0.0) and time.monotonic() >= self.next_safe_check:
            try:
                mode = self.transport.query_oi_mode(live=True)
                self.safe_gate.observe_mode(mode)
                self.next_safe_check = time.monotonic() + self.safe_check_interval
            except (OSError, ConnectionError, RoombaDiagnosticError) as error:
                self.handle_safety_failure(error)
                return
            command = self.command_state.current()
            if command is None:
                self.send_stop()
                return

        try:
            sequence = self.transport.send_twist(
                command[0],
                command[1],
                track_width_m=self.wheel_base,
                max_speed_mm_s=self.max_speed_mm_s,
            )
            if self.drive_session_limiter.enabled:
                left, right = twist_to_wheel_speeds(
                    command[0],
                    command[1],
                    track_width_m=self.wheel_base,
                    max_wheel_speed_mm_s=self.max_speed_mm_s,
                )
                self.get_logger().info(
                    "bounded Drive ACK: status=0x03 sequence=%d right=%d left=%d "
                    "monotonic=%.6f"
                    % (sequence, right, left, time.monotonic())
                )
        except (
            OSError,
            ConnectionError,
            RoombaDiagnosticError,
            TypeError,
            ValueError,
        ) as error:
            self.handle_send_failure(error)

    def tick_watchdog_test(self):
        event, command = self.watchdog_test_state.poll()
        if event == "command":
            # This dedicated test must leave Pico's receive loop free to fire
            # its 500 ms watchdog.  The connection preflight immediately above
            # already required STOP ACK=0x07, requested speeds=0, and OI Safe=2.
            # A synchronous live UART diagnostic here can occupy Pico while the
            # intentional no-traffic interval is being measured.
            if not self.watchdog_preflight_logged:
                self.watchdog_preflight_logged = True
                self.get_logger().info(
                    "watchdog test preflight gate retained: STOP ACK and OI Safe "
                    "were confirmed on this TCP connection; live diagnostics "
                    "are suspended during the bounded watchdog trial"
                )
            try:
                self.transport.send_twist(
                    command[0],
                    command[1],
                    track_width_m=self.wheel_base,
                    max_speed_mm_s=self.max_speed_mm_s,
                )
            except (
                OSError,
                ConnectionError,
                RoombaDiagnosticError,
                TypeError,
                ValueError,
            ) as error:
                self.handle_send_failure(error)
            return
        if event == "silence_start":
            self.get_logger().warn(
                "watchdog test silence observed; bridge will send no TCP/OI "
                "traffic until %.2fs fallback; monotonic=%.6f"
                % (self.watchdog_test_silence, time.monotonic())
            )
            return
        if event == "fallback_stop":
            self.get_logger().warn(
                "watchdog test host fallback STOP sent after %.2fs silence; "
                "monotonic=%.6f"
                % (self.watchdog_test_silence, time.monotonic())
            )
            self.send_stop()

    def send_stop(self):
        self.drive_session_limiter.latch_stop()
        try:
            self.transport.send_stop()
        except (OSError, ConnectionError) as error:
            self.handle_send_failure(error)

    def expire_bounded_drive(self):
        self.clear_command()
        self.bounded_drive_rejection_logged = False
        self.get_logger().warn(
            "bounded drive deadline reached: STOP sent, ARM revoked; "
            "limit=%.3fs monotonic=%.6f"
            % (self.max_drive_session_duration, time.monotonic())
        )
        self.send_stop()

    def handle_send_failure(self, error):
        self.get_logger().warn(f"送信失敗、再接続します: {error}")
        self.clear_command()
        self.clear_motion_authorization()
        # A missing, corrupt, or negative per-Drive ACK is a safety failure.
        # Attempt one raw STOP before closing; close() suppresses socket errors.
        self.transport.close(send_stop=True)
        self.next_retry = time.monotonic() + 1.0

    def handle_safety_failure(self, error):
        self.get_logger().error(f"走行許可を取り消しました: {error}")
        self.clear_command()
        self.clear_motion_authorization()
        try:
            self.transport.send_stop()
        except (OSError, ConnectionError):
            pass
        self.transport.close(send_stop=False)
        self.next_retry = time.monotonic() + 1.0

    def shutdown(self):
        self.clear_command()
        self.clear_motion_authorization()
        self.transport.close(send_stop=True)


def main():
    rclpy.init()
    node = RoombaBridgeTcp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

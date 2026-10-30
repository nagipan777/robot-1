"""ROS-independent TCP transport for a Pico W Roomba OI relay."""

import math
import re
import socket
import struct
import time

from differential_drive import twist_to_wheel_speeds


OI_DRIVE_DIRECT = 145
RELAY_READY = b"READY\n"
STOP_PACKET = bytes([OI_DRIVE_DIRECT, 0, 0, 0, 0])
DIAGNOSTIC_MAGIC = 0xD7
DIAGNOSTIC_STOP = 0x01
DIAGNOSTIC_OI_MODE = 0x02
DIAGNOSTIC_OI_MODE_LIVE = 0x03
CONTROL_ARM = 0x04
GENERATION_DRIVE = 0x05
SEQUENCED_DRIVE = 0x06
DIAGNOSTIC_ENCODERS = 0x07
DIAGNOSTIC_STOP_RESPONSE = 0x81
DIAGNOSTIC_OI_MODE_RESPONSE = 0x82
CONTROL_ARM_RESPONSE = 0x84
SEQUENCED_DRIVE_RESPONSE = 0x86
DIAGNOSTIC_ENCODERS_RESPONSE = 0x87
DIAGNOSTIC_ERROR = 0xE0
ARM_ACCEPTED = 0x01
DRIVE_ACK_VALIDATED = 0x01
DRIVE_ACK_UART_WRITTEN = 0x02
DRIVE_ACK_COMPLETE = DRIVE_ACK_VALIDATED | DRIVE_ACK_UART_WRITTEN
STOP_ACK_VALIDATED = 0x01
STOP_ACK_UART_WRITTEN = 0x02
STOP_ACK_ROBOT_ZERO = 0x04
STOP_ACK_COMPLETE = (
    STOP_ACK_VALIDATED | STOP_ACK_UART_WRITTEN | STOP_ACK_ROBOT_ZERO
)
ENCODER_ACK_STOP_WRITTEN = 0x01
ENCODER_ACK_QUERY_COMPLETE = 0x02
ENCODER_ACK_COMPLETE = ENCODER_ACK_STOP_WRITTEN | ENCODER_ACK_QUERY_COMPLETE
STOP_DIAGNOSTIC_RESPONSE_SIZE = 7
OI_MODE_RESPONSE_SIZE = 3
ARM_RESPONSE_SIZE = 7
DRIVE_RESPONSE_SIZE = 13
ENCODER_RESPONSE_SIZE = 7


class RoombaDiagnosticError(Exception):
    """Raised when a fixed diagnostic request is rejected or incomplete."""


class DriveDeadlineExpired(RoombaDiagnosticError):
    """Raised when a bounded-drive deadline prevents or interrupts TCP I/O."""


class SafeModeGate:
    """Latch motion permission only after STOP and OI Safe are confirmed."""

    def __init__(self):
        self.authorized = False

    def reset(self):
        self.authorized = False

    def establish(self, stop_status, requested_right, requested_left, oi_mode):
        self.authorized = (
            stop_status == STOP_ACK_COMPLETE
            and requested_right == 0
            and requested_left == 0
            and oi_mode == 2
        )
        if not self.authorized:
            raise RoombaDiagnosticError(
                "motion safety gate failed: status=0x{:02x}, right={}, "
                "left={}, oi_mode={}".format(
                    stop_status, requested_right, requested_left, oi_mode
                )
            )

    def observe_mode(self, oi_mode):
        if oi_mode != 2:
            self.reset()
            raise RoombaDiagnosticError(
                "Roomba left OI Safe mode: mode={}".format(oi_mode)
            )

    def permits(self, linear_x, angular_z):
        return (linear_x == 0.0 and angular_z == 0.0) or self.authorized


def drive_direct_packet(left_mm_s, right_mm_s):
    """Return OI Drive Direct bytes; the protocol order is right, then left."""
    return struct.pack(">Bhh", OI_DRIVE_DIRECT, right_mm_s, left_mm_s)


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def with_crc(payload):
    return payload + struct.pack(">H", crc16_ccitt(payload))


def _received_count_from_error(error):
    """Extract fixed-response progress from existing diagnostic messages."""
    message = str(error)
    match = re.search(r"received (\d+)/(\d+) bytes", message)
    if match is None:
        match = re.search(r"\((\d+)/(\d+) bytes", message)
    return int(match.group(1)) if match is not None else 0


class CommandState:
    """Gate commands by explicit enablement, freshness, and reconnect resets."""

    def __init__(self, drive_enabled=False, command_timeout=0.5, clock=None):
        if command_timeout <= 0.0:
            raise ValueError("command_timeout must be positive")
        if clock is None:
            import time

            clock = time.monotonic
        self.drive_enabled = bool(drive_enabled)
        self.command_timeout = command_timeout
        self.clock = clock
        self.revision = 0
        self.clear()

    def clear(self):
        self.command = None
        self.received_at = None

    def update(self, linear_x, angular_z):
        if not self.drive_enabled:
            self.clear()
            return
        self.command = (linear_x, angular_z)
        self.received_at = self.clock()
        self.revision += 1

    def current(self):
        if not self.drive_enabled or self.received_at is None:
            self.clear()
            return None
        if self.clock() - self.received_at > self.command_timeout:
            self.clear()
            return None
        return self.command

    def current_snapshot(self):
        """Return the latest fresh command with its receipt time and revision."""
        command = self.current()
        if command is None:
            return None
        return command, self.received_at, self.revision


class DriveAckState:
    """Track one in-flight Drive separately from ARM and command freshness."""

    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.reset("initial")

    def reset(self, reason):
        self.state = "idle"
        self.generation = None
        self.sequence = None
        self.started_at = None
        self.completed_at = None
        self.reason = reason

    def begin(self, generation, sequence):
        if self.state == "pending":
            raise RoombaDiagnosticError(
                "Drive ACK still pending: generation={}, sequence={}".format(
                    self.generation, self.sequence
                )
            )
        self.state = "pending"
        self.generation = generation
        self.sequence = sequence
        self.started_at = self.clock()
        self.completed_at = None
        self.reason = "waiting"

    def confirm(self, generation, sequence):
        if (
            self.state != "pending"
            or generation != self.generation
            or sequence != self.sequence
        ):
            raise RoombaDiagnosticError(
                "Drive ACK state mismatch: pending={}/{}, received={}/{}".format(
                    self.generation, self.sequence, generation, sequence
                )
            )
        self.state = "confirmed"
        self.completed_at = self.clock()
        self.reason = "validated"

    def fail(self, reason):
        self.state = "failed"
        self.completed_at = self.clock()
        self.reason = str(reason)

    @property
    def elapsed_s(self):
        if self.started_at is None:
            return None
        end = self.completed_at if self.completed_at is not None else self.clock()
        return end - self.started_at


class DriveSessionLimiter:
    """One-shot monotonic deadline for explicitly bounded hardware tests.

    A zero limit disables the limiter for existing launch paths.  Once an
    enabled session expires or is stopped, another non-zero command is
    rejected until a new explicit ARM resets the limiter.
    """

    MAX_DURATION_S = 1.0

    def __init__(self, duration_s=0.0, clock=None, *, max_duration_s=None):
        if max_duration_s is None:
            max_duration_s = self.MAX_DURATION_S
        if (
            not isinstance(duration_s, (int, float))
            or isinstance(duration_s, bool)
            or not math.isfinite(duration_s)
            or duration_s < 0.0
            or not isinstance(max_duration_s, (int, float))
            or isinstance(max_duration_s, bool)
            or not math.isfinite(max_duration_s)
            or max_duration_s <= 0.0
            or duration_s > max_duration_s
        ):
            raise ValueError(
                "drive session duration must be 0 or <= {:.1f} seconds".format(
                    max_duration_s
                )
            )
        self.duration_s = float(duration_s)
        self.max_duration_s = float(max_duration_s)
        self.clock = clock or time.monotonic
        self.started_at = None
        self.latched = False

    @property
    def enabled(self):
        return self.duration_s > 0.0

    def arm(self):
        self.started_at = None
        self.latched = False

    def observe_nonzero(self):
        if not self.enabled:
            return "allowed"
        if self.latched:
            return "latched"
        now = self.clock()
        if self.started_at is None:
            self.started_at = now
            return "started"
        if now - self.started_at >= self.duration_s:
            self.latched = True
            return "expired"
        return "allowed"

    def poll(self):
        if (
            not self.enabled
            or self.latched
            or self.started_at is None
            or self.clock() - self.started_at < self.duration_s
        ):
            return False
        self.latched = True
        return True

    @property
    def deadline(self):
        if not self.enabled or self.started_at is None:
            return None
        return self.started_at + self.duration_s

    def permits_dispatch(self):
        if not self.enabled:
            return True
        if self.latched or self.started_at is None:
            return False
        if self.clock() >= self.deadline:
            self.latched = True
            return False
        return True

    def latch_stop(self):
        if self.enabled and self.started_at is not None:
            self.latched = True


class WatchdogTestCommandState:
    """Forward each fresh ROS command once, then preserve an intentional gap."""

    def __init__(self, silence_duration=0.75, clock=None, gap_notice_delay=0.1):
        if not 0.5 < silence_duration <= 1.0:
            raise ValueError("silence_duration must be > 0.5 and <= 1.0")
        if not 0.0 <= gap_notice_delay < silence_duration:
            raise ValueError("gap_notice_delay must be within the silence duration")
        if clock is None:
            import time

            clock = time.monotonic
        self.silence_duration = silence_duration
        self.gap_notice_delay = gap_notice_delay
        self.clock = clock
        self.clear()

    def clear(self):
        self.pending = None
        self.received_at = None
        self.silence_announced = False
        self.fallback_stop_sent = False

    def update(self, linear_x, angular_z):
        self.pending = (linear_x, angular_z)
        self.received_at = self.clock()
        self.silence_announced = False
        self.fallback_stop_sent = False

    def poll(self):
        if self.pending is not None:
            command = self.pending
            self.pending = None
            return "command", command
        if self.received_at is None:
            return "idle", None
        elapsed = self.clock() - self.received_at
        if elapsed + 1e-9 >= self.silence_duration and not self.fallback_stop_sent:
            self.fallback_stop_sent = True
            return "fallback_stop", None
        if elapsed + 1e-9 >= self.gap_notice_delay and not self.silence_announced:
            self.silence_announced = True
            return "silence_start", None
        return "silence", None


class RoombaTcpTransport:
    def __init__(
        self,
        host,
        port,
        *,
        connect_timeout=2.0,
        socket_factory=socket.create_connection,
        trace=None,
    ):
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.socket_factory = socket_factory
        self.trace = trace
        self.socket = None
        self.armed_generation = None
        self.drive_sequence = 0
        self.drive_ack_state = DriveAckState()

    def _trace(self, stage, **fields):
        if self.trace is not None:
            self.trace(stage, **fields)

    def _fail_drive_ack(self, reason, *, received_bytes=0):
        self.drive_ack_state.fail(reason)
        self._trace(
            "tcp_drive_ack_failed",
            generation=self.drive_ack_state.generation,
            sequence=self.drive_ack_state.sequence,
            received_bytes=received_bytes,
            expected_bytes=DRIVE_RESPONSE_SIZE,
            elapsed_s=self.drive_ack_state.elapsed_s,
            reason=self.drive_ack_state.reason,
        )

    @property
    def connected(self):
        return self.socket is not None

    def connect(self):
        self.close(send_stop=False)
        self.armed_generation = None
        self.drive_sequence = 0
        self.drive_ack_state.reset("connect")
        connection = None
        started = time.monotonic()
        self._trace("tcp_connect_start", host=self.host, port=self.port)
        try:
            connection = self.socket_factory(
                (self.host, self.port), timeout=self.connect_timeout
            )
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            ready = self._receive_exact(
                connection, len(RELAY_READY), operation="relay READY"
            )
            if ready != RELAY_READY:
                raise ConnectionError("unexpected Pico W response: {!r}".format(ready))
            connection.settimeout(2.0)
            self.socket = connection
            self.send_stop()
            self._trace(
                "tcp_ready_received",
                bytes=len(ready),
                elapsed_s=time.monotonic() - started,
            )
        except Exception as error:
            self._trace(
                "tcp_connect_failed",
                error=str(error),
                elapsed_s=time.monotonic() - started,
            )
            if connection is not None:
                try:
                    connection.close()
                except OSError:
                    pass
            self.socket = None
            raise

    def send_stop(self):
        self.send_packet(STOP_PACKET)
        self.armed_generation = None
        self.drive_sequence = 0
        self.drive_ack_state.reset("stop")

    def arm(self, generation):
        if not isinstance(generation, int) or isinstance(generation, bool):
            raise TypeError("ARM generation must be an integer")
        if not 1 <= generation <= 0xFFFFFFFF:
            raise ValueError("ARM generation must be from 1 to 4294967295")
        # A new ARM attempt invalidates any prior session authorization before
        # the request is put on the wire.  Only a fully validated ACK below may
        # publish the new generation to Drive.
        self.armed_generation = None
        self.drive_sequence = 0
        self.drive_ack_state.reset("arm_attempt")
        packet = bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM]) + struct.pack(">I", generation)
        started = time.monotonic()
        self._trace("tcp_arm_send_start", generation=generation, bytes=len(packet))
        self.send_packet(packet)
        self._trace("tcp_arm_sent", generation=generation, bytes=len(packet))
        try:
            response = self._receive_exact(
                self.socket, ARM_RESPONSE_SIZE, operation="ARM response"
            )
        except (OSError, ConnectionError, RoombaDiagnosticError) as error:
            self.armed_generation = None
            self.drive_sequence = 0
            self._trace(
                "tcp_arm_response_failed",
                generation=generation,
                error=str(error),
                elapsed_s=time.monotonic() - started,
            )
            raise
        self._trace(
            "tcp_arm_response_received",
            generation=generation,
            bytes=len(response),
            raw=response.hex(),
            elapsed_s=time.monotonic() - started,
        )
        expected = bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM_RESPONSE])
        if response[:2] != expected:
            raise RoombaDiagnosticError(
                "unexpected ARM response: raw={}".format(response.hex())
            )
        status = response[2]
        highest_generation = struct.unpack(">I", response[3:])[0]
        if status != ARM_ACCEPTED or highest_generation != generation:
            raise RoombaDiagnosticError(
                "ARM rejected: status=0x{:02x}, requested_generation={}, "
                "highest_generation={}".format(
                    status, generation, highest_generation
                )
            )
        self.armed_generation = generation
        self.drive_sequence = 0
        return generation

    def verify_stop(self):
        """Return ``(status, requested_right, requested_left)`` from Pico W."""
        self.send_packet(bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_STOP]))
        response = self._receive_exact(
            self.socket,
            STOP_DIAGNOSTIC_RESPONSE_SIZE,
            operation="STOP diagnostic",
        )
        if response[:2] != bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_STOP_RESPONSE]):
            raise RoombaDiagnosticError(
                "unexpected STOP diagnostic response: {!r}".format(response)
            )
        status = response[2]
        right, left = struct.unpack(">hh", response[3:])
        if status != STOP_ACK_COMPLETE or right != 0 or left != 0:
            raise RoombaDiagnosticError(
                "STOP was not confirmed: status=0x{:02x}, right={}, left={}, "
                "raw={}".format(
                    status, right, left, response.hex()
                )
            )
        return status, right, left

    def query_oi_mode(self, *, live=False, deadline_monotonic=None):
        """Return Roomba sensor packet 35 (0=Off, 1=Passive, 2=Safe, 3=Full)."""
        request = DIAGNOSTIC_OI_MODE_LIVE if live else DIAGNOSTIC_OI_MODE
        started = time.monotonic()
        self._trace(
            "tcp_safe_query_start",
            live=live,
            deadline_monotonic=deadline_monotonic,
        )
        self.send_packet(bytes([DIAGNOSTIC_MAGIC, request]))
        sent_at = time.monotonic()
        self._trace(
            "tcp_safe_query_sent", live=live, send_elapsed_s=sent_at - started
        )
        operation = "live OI mode diagnostic" if live else "OI mode diagnostic"
        response = self._receive_exact(
            self.socket,
            OI_MODE_RESPONSE_SIZE,
            operation=operation,
            deadline_monotonic=deadline_monotonic,
        )
        self._trace(
            "tcp_safe_query_received",
            live=live,
            response_wait_elapsed_s=time.monotonic() - sent_at,
            raw=response.hex(),
        )
        if response[:2] != bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_RESPONSE]):
            raise RoombaDiagnosticError(
                "unexpected OI mode response: {!r}".format(response)
            )
        mode = response[2]
        if mode not in (0, 1, 2, 3):
            raise RoombaDiagnosticError("invalid OI mode: {}".format(mode))
        return mode

    def query_encoders(self):
        """Return unsigned packet 43/44 counts from a STOP-first diagnostic."""
        self.send_packet(bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_ENCODERS]))
        response = self._receive_exact(
            self.socket,
            ENCODER_RESPONSE_SIZE,
            operation="encoder diagnostic",
        )
        if response[:2] != bytes(
            [DIAGNOSTIC_MAGIC, DIAGNOSTIC_ENCODERS_RESPONSE]
        ):
            raise RoombaDiagnosticError(
                "unexpected encoder response: raw={}".format(response.hex())
            )
        status = response[2]
        if status != ENCODER_ACK_COMPLETE:
            raise RoombaDiagnosticError(
                "encoder query was not confirmed: status=0x{:02x}, raw={}".format(
                    status, response.hex()
                )
            )
        self.armed_generation = None
        self.drive_sequence = 0
        return struct.unpack(">HH", response[3:])

    def send_twist(
        self,
        linear_x,
        angular_z,
        *,
        track_width_m,
        max_speed_mm_s,
        deadline_monotonic=None,
    ):
        self._trace(
            "transport_send_start",
            linear_x=linear_x,
            angular_z=angular_z,
            generation=self.armed_generation,
        )
        left, right = twist_to_wheel_speeds(
            linear_x,
            angular_z,
            track_width_m=track_width_m,
            max_wheel_speed_mm_s=max_speed_mm_s,
        )
        packet = drive_direct_packet(left, right)
        if left == 0 and right == 0:
            self.send_stop()
            return
        if self.armed_generation is None:
            raise RoombaDiagnosticError("non-zero drive rejected: ARM is required")
        sequence = self.drive_sequence + 1
        payload = (
            bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE])
            + struct.pack(">II", self.armed_generation, sequence)
            + packet
        )
        # This is deliberately adjacent to send_packet: a delayed ROS callback,
        # live Safe query, or trace write must never dispatch a late Drive.
        dispatch_checked_at = time.monotonic()
        if (
            deadline_monotonic is not None
            and dispatch_checked_at >= deadline_monotonic
        ):
            self._trace(
                "tcp_drive_deadline_rejected",
                generation=self.armed_generation,
                sequence=sequence,
                deadline_monotonic=deadline_monotonic,
                checked_at=dispatch_checked_at,
                overdue_s=dispatch_checked_at - deadline_monotonic,
            )
            raise DriveDeadlineExpired(
                "non-zero drive rejected before TCP send: deadline exceeded by "
                "{:.6f}s".format(dispatch_checked_at - deadline_monotonic)
            )
        send_started = time.monotonic()
        self.drive_ack_state.begin(self.armed_generation, sequence)
        try:
            self.send_packet(with_crc(payload))
        except (OSError, ConnectionError) as error:
            self._fail_drive_ack(
                "send_error: {}".format(error), received_bytes=0
            )
            raise
        send_returned = time.monotonic()
        self._trace(
            "tcp_drive_sent",
            generation=self.armed_generation,
            sequence=sequence,
            right_mm_s=right,
            left_mm_s=left,
            send_elapsed_s=send_returned - send_started,
        )
        self._trace(
            "tcp_drive_ack_wait_start",
            generation=self.armed_generation,
            sequence=sequence,
            deadline_monotonic=deadline_monotonic,
        )
        try:
            response = self._receive_exact(
                self.socket,
                DRIVE_RESPONSE_SIZE,
                operation="Drive ACK",
                deadline_monotonic=deadline_monotonic,
            )
        except (OSError, ConnectionError, RoombaDiagnosticError) as error:
            self._fail_drive_ack(
                error,
                received_bytes=_received_count_from_error(error),
            )
            raise
        self._trace(
            "tcp_drive_ack_received",
            generation=self.armed_generation,
            sequence=sequence,
            raw=response.hex(),
            ack_wait_elapsed_s=time.monotonic() - send_returned,
        )
        if len(response) != DRIVE_RESPONSE_SIZE:
            self._fail_drive_ack("invalid_size", received_bytes=len(response))
            raise RoombaDiagnosticError("invalid Drive ACK size")
        body, supplied_crc = response[:-2], struct.unpack(">H", response[-2:])[0]
        if crc16_ccitt(body) != supplied_crc:
            self._fail_drive_ack(
                "checksum_mismatch", received_bytes=len(response)
            )
            raise RoombaDiagnosticError(
                "Drive ACK checksum mismatch: raw={}".format(response.hex())
            )
        if body[:2] != bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE_RESPONSE]):
            self._fail_drive_ack(
                "unexpected_response", received_bytes=len(response)
            )
            raise RoombaDiagnosticError(
                "unexpected Drive ACK: raw={}".format(response.hex())
            )
        status = body[2]
        ack_generation, ack_sequence = struct.unpack(">II", body[3:11])
        if (
            status != DRIVE_ACK_COMPLETE
            or ack_generation != self.armed_generation
            or ack_sequence != sequence
        ):
            self._fail_drive_ack(
                "status_generation_or_sequence_mismatch",
                received_bytes=len(response),
            )
            raise RoombaDiagnosticError(
                "Drive rejected: status=0x{:02x}, generation={}, sequence={}".format(
                    status, ack_generation, ack_sequence
                )
            )
        self.drive_sequence = sequence
        self.drive_ack_state.confirm(ack_generation, ack_sequence)
        self._trace(
            "tcp_drive_ack_validated",
            generation=self.armed_generation,
            sequence=sequence,
            status=status,
            ack_elapsed_s=self.drive_ack_state.elapsed_s,
        )
        return sequence

    def send_packet(self, packet):
        if self.socket is None:
            raise ConnectionError("TCP transport is not connected")
        self.socket.sendall(packet)

    def close(self, *, send_stop=True):
        connection = self.socket
        self.socket = None
        self.armed_generation = None
        self.drive_sequence = 0
        self.drive_ack_state.reset("close")
        if connection is None:
            return
        if send_stop:
            try:
                connection.sendall(STOP_PACKET)
            except OSError:
                pass
        try:
            connection.close()
        except OSError:
            pass

    def _receive_exact(
        self,
        connection,
        size,
        *,
        operation="TCP response",
        deadline_monotonic=None,
    ):
        """Receive one fixed response without letting it hide a Drive deadline.

        The same thread remains the sole socket owner.  With an active Drive
        deadline, short receive timeouts provide cancellation points.  At the
        absolute deadline it sends STOP on the current session, then closes the
        socket so a late/partial response can never be consumed as a response
        belonging to a later session.
        """
        received = bytearray()
        started = time.monotonic()
        original_timeout = None
        deadline_poll_s = 0.01
        if deadline_monotonic is not None:
            try:
                original_timeout = connection.gettimeout()
            except AttributeError:
                original_timeout = None
        while len(received) < size:
            if deadline_monotonic is not None:
                remaining = deadline_monotonic - time.monotonic()
                if remaining <= 0.0:
                    stop_started = time.monotonic()
                    self._trace(
                        "tcp_deadline_stop_begin",
                        operation=operation,
                        received_bytes=len(received),
                        expected_bytes=size,
                        deadline_monotonic=deadline_monotonic,
                        stop_started=stop_started,
                        overdue_s=max(0.0, stop_started - deadline_monotonic),
                    )
                    stop_error = None
                    try:
                        connection.sendall(STOP_PACKET)
                    except OSError as error:
                        stop_error = error
                    self.armed_generation = None
                    self.drive_sequence = 0
                    self.drive_ack_state.fail("bounded_deadline")
                    stop_returned = time.monotonic()
                    self._trace(
                        "tcp_deadline_stop_return",
                        operation=operation,
                        stop_returned=stop_returned,
                        send_elapsed_s=stop_returned - stop_started,
                    )
                    # Poisoned response state must not survive into another
                    # request. Closing also causes Pico's disconnect STOP.
                    if self.socket is connection:
                        self.socket = None
                    try:
                        connection.close()
                    except OSError:
                        pass
                    deadline_error = DriveDeadlineExpired(
                        "{} interrupted at bounded Drive deadline; STOP sent; "
                        "received {}/{} bytes; deadline={:.6f}; "
                        "stop_started={:.6f}; overdue_s={:.6f}; "
                        "stop_send_elapsed_s={:.6f}{}".format(
                            operation,
                            len(received),
                            size,
                            deadline_monotonic,
                            stop_started,
                            max(0.0, stop_started - deadline_monotonic),
                            stop_returned - stop_started,
                            " (STOP write failed: {})".format(stop_error)
                            if stop_error is not None
                            else "",
                        )
                    )
                    if stop_error is not None:
                        raise deadline_error from stop_error
                    raise deadline_error
                try:
                    connection.settimeout(min(deadline_poll_s, remaining))
                except AttributeError:
                    pass
            try:
                chunk = connection.recv(size - len(received))
            except socket.timeout as error:
                if deadline_monotonic is not None:
                    continue
                raise RoombaDiagnosticError(
                    "{} timed out: received {}/{} bytes; raw={}; "
                    "elapsed={:.3f}s".format(
                        operation,
                        len(received),
                        size,
                        bytes(received).hex(),
                        time.monotonic() - started,
                    )
                ) from error
            if not chunk:
                raise ConnectionError(
                    "Pico W closed during {} ({}/{} bytes; raw={})".format(
                        operation, len(received), size, bytes(received).hex()
                    )
                )
            received.extend(chunk)
        if deadline_monotonic is not None and self.socket is connection:
            try:
                connection.settimeout(original_timeout)
            except AttributeError:
                pass
        return bytes(received)

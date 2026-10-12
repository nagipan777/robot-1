"""Safe, testable serial driver for the Roomba 680 Open Interface."""

from __future__ import annotations

import argparse
import struct
import threading
import time
from typing import Callable, Optional, Tuple

from odometry import Odometry, Pose2D


DEFAULT_PORT = "/dev/cu.usbserial-A10QJ1R4"
BAUDRATE = 19200

OPCODE_START = 128
OPCODE_SAFE = 131
OPCODE_SENSORS = 142
OPCODE_DRIVE_DIRECT = 145
OPCODE_QUERY_LIST = 149

SENSOR_OI_MODE = 35
SENSOR_LEFT_ENCODER = 43
SENSOR_RIGHT_ENCODER = 44
OI_MODE_SAFE = 2


class RoombaError(Exception):
    """Base class for driver errors."""


class RoombaConnectionError(RoombaError):
    """Raised when the driver cannot enter and confirm OI Safe mode."""


class RoombaCommunicationError(RoombaError):
    """Raised when serial communication fails or a response is incomplete."""


class RoombaSafetyError(RoombaError):
    """Raised when a motion command would violate a safety condition."""


class RoombaDriver:
    """Roomba 680 OI driver with Safe-mode gating and a command watchdog.

    ``serial_instance`` makes the transport injectable for unit tests. When it
    is omitted, ``pyserial`` is imported only when :meth:`connect` is called.
    The watchdog is armed by a non-zero drive command and sends a zero-speed
    command if no subsequent drive command arrives before ``command_timeout``.
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        *,
        max_speed_mm_s: int = 50,
        read_timeout: float = 1.0,
        write_timeout: float = 1.0,
        command_timeout: Optional[float] = 1.0,
        serial_instance=None,
        serial_factory: Optional[Callable[..., object]] = None,
        odometry: Optional[Odometry] = None,
    ) -> None:
        if not isinstance(max_speed_mm_s, int) or not 1 <= max_speed_mm_s <= 500:
            raise ValueError("max_speed_mm_s must be an integer from 1 to 500")
        if read_timeout <= 0 or write_timeout <= 0:
            raise ValueError("serial timeouts must be positive")
        if command_timeout is not None and command_timeout <= 0:
            raise ValueError("command_timeout must be positive or None")

        self.port = port
        self.max_speed_mm_s = max_speed_mm_s
        self.read_timeout = float(read_timeout)
        self.write_timeout = float(write_timeout)
        self.command_timeout = command_timeout
        self._serial = serial_instance
        self._serial_factory = serial_factory
        self.odometry = odometry if odometry is not None else Odometry()
        self._connected = False
        self._safe_confirmed = False
        self._drive_deadline: Optional[float] = None
        self._lock = threading.RLock()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None
        self._watchdog_error: Optional[BaseException] = None

    @property
    def safe_confirmed(self) -> bool:
        return self._safe_confirmed

    @property
    def watchdog_error(self) -> Optional[BaseException]:
        return self._watchdog_error

    def connect(self) -> None:
        """Open the port, enter OI Safe mode, and verify sensor packet 35."""
        if self._connected:
            return

        try:
            if self._serial is None:
                factory = self._serial_factory
                if factory is None:
                    import serial

                    factory = serial.Serial
                self._serial = factory(
                    port=self.port,
                    baudrate=BAUDRATE,
                    bytesize=8,
                    parity="N",
                    stopbits=1,
                    timeout=min(self.read_timeout, 0.05),
                    write_timeout=self.write_timeout,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                )
            elif hasattr(self._serial, "is_open") and not self._serial.is_open:
                self._serial.open()

            # A finite transport timeout is required so _read_exact() can
            # enforce its overall deadline even for an injected pyserial port.
            if hasattr(self._serial, "timeout"):
                self._serial.timeout = min(self.read_timeout, 0.05)

            self._connected = True
            # FT232RL and a sleeping Roomba need more settling time than an
            # in-memory test transport. This mirrors the proven probe script.
            time.sleep(0.5)
            self._reset_input_buffer()
            for _ in range(3):
                self._write_raw(bytes([OPCODE_START]))
                time.sleep(0.3)
            self._write_raw(bytes([OPCODE_SAFE]))
            time.sleep(0.3)
            self._reset_input_buffer()
            self._write_raw(bytes([OPCODE_SENSORS, SENSOR_OI_MODE]))
            mode = self._read_exact(1)
            if mode[0] != OI_MODE_SAFE:
                raise RoombaConnectionError(
                    f"OI Safe mode was not confirmed (mode={mode[0]})"
                )
            self._safe_confirmed = True
            self._start_watchdog()
        except Exception as exc:
            self._safe_confirmed = False
            self._emergency_stop()
            self._close_serial_quietly()
            self._connected = False
            if isinstance(exc, RoombaConnectionError):
                raise
            raise RoombaConnectionError(
                f"failed to confirm OI Safe mode on {self.port}: {exc}. "
                "Press CLEAN to wake the Roomba, then check 19200 bps, "
                "TX/RX crossover, and common GND."
            ) from exc

    def drive_direct(self, left_mm_s: int, right_mm_s: int) -> None:
        """Set wheel speeds; OI payload order is right wheel, then left wheel."""
        self._validate_speed(left_mm_s, "left_mm_s")
        self._validate_speed(right_mm_s, "right_mm_s")
        if not self._connected or not self._safe_confirmed:
            raise RoombaSafetyError("drive is disabled until OI Safe mode is confirmed")

        command = bytes([OPCODE_DRIVE_DIRECT]) + struct.pack(
            ">hh", right_mm_s, left_mm_s
        )
        try:
            with self._lock:
                self._write_raw(command)
                self._drive_deadline = (
                    time.monotonic() + self.command_timeout
                    if self.command_timeout is not None and (left_mm_s or right_mm_s)
                    else None
                )
        except Exception as exc:
            self._handle_communication_failure(exc, "drive command failed")

    def read_encoders(self) -> Tuple[int, int]:
        """Return unsigned 16-bit ``(left, right)`` encoder counts."""
        if not self._connected:
            raise RoombaConnectionError("driver is not connected")
        try:
            with self._lock:
                self._reset_input_buffer()
                self._write_raw(
                    bytes(
                        [
                            OPCODE_QUERY_LIST,
                            2,
                            SENSOR_LEFT_ENCODER,
                            SENSOR_RIGHT_ENCODER,
                        ]
                    )
                )
                data = self._read_exact(4)
            return struct.unpack(">HH", data)
        except Exception as exc:
            self._handle_communication_failure(exc, "encoder read failed")

    def read_odometry(self) -> Pose2D:
        """Read both encoders and update the configured odometry estimator."""
        left, right = self.read_encoders()
        return self.odometry.update(left, right)

    def stop(self) -> None:
        """Send zero speed to both wheels."""
        if self._serial is None or not self._connected:
            return
        try:
            with self._lock:
                self._write_raw(bytes([OPCODE_DRIVE_DIRECT, 0, 0, 0, 0]))
                self._drive_deadline = None
        except Exception as exc:
            self._safe_confirmed = False
            self._emergency_stop()
            raise RoombaCommunicationError(f"stop command failed: {exc}") from exc

    def close(self) -> None:
        """Stop the wheels and close the serial port."""
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.5)

        stop_error = None
        if self._serial is not None and self._connected:
            try:
                self.stop()
            except RoombaCommunicationError as exc:
                stop_error = exc
        self._close_serial_quietly()
        self._connected = False
        self._safe_confirmed = False
        self._drive_deadline = None
        if stop_error is not None:
            raise stop_error

    def __enter__(self) -> "RoombaDriver":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        try:
            self.close()
        except RoombaCommunicationError:
            if exc_type is None:
                raise
        return False

    def _validate_speed(self, value: int, name: str) -> None:
        if not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if abs(value) > self.max_speed_mm_s:
            raise RoombaSafetyError(
                f"{name}={value} exceeds the configured limit "
                f"of {self.max_speed_mm_s} mm/s"
            )

    def _write_raw(self, data: bytes) -> None:
        written = self._serial.write(data)
        if written is not None and written != len(data):
            raise IOError(f"partial serial write: {written}/{len(data)} bytes")
        self._serial.flush()

    def _read_exact(self, size: int) -> bytes:
        deadline = time.monotonic() + self.read_timeout
        received = bytearray()
        while len(received) < size:
            if time.monotonic() >= deadline:
                raise RoombaCommunicationError(
                    f"incomplete response: received {len(received)}/{size} bytes"
                )
            chunk = self._serial.read(size - len(received))
            if chunk:
                received.extend(chunk)
            else:
                time.sleep(0.001)
        return bytes(received)

    def _reset_input_buffer(self) -> None:
        reset = getattr(self._serial, "reset_input_buffer", None)
        if reset is not None:
            reset()

    def _handle_communication_failure(self, exc: Exception, message: str) -> None:
        self._safe_confirmed = False
        self._drive_deadline = None
        self._emergency_stop()
        if isinstance(exc, RoombaCommunicationError):
            raise exc
        raise RoombaCommunicationError(f"{message}: {exc}") from exc

    def _emergency_stop(self) -> None:
        if self._serial is None:
            return
        try:
            with self._lock:
                self._write_raw(bytes([OPCODE_DRIVE_DIRECT, 0, 0, 0, 0]))
        except Exception:
            pass

    def _start_watchdog(self) -> None:
        if self.command_timeout is None:
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="roomba-command-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def _watchdog_loop(self) -> None:
        interval = min(self.command_timeout / 4.0, 0.05)
        while not self._watchdog_stop.wait(interval):
            with self._lock:
                deadline = self._drive_deadline
                if deadline is None or time.monotonic() < deadline:
                    continue
                try:
                    self._write_raw(bytes([OPCODE_DRIVE_DIRECT, 0, 0, 0, 0]))
                except Exception as exc:
                    self._watchdog_error = exc
                    self._safe_confirmed = False
                finally:
                    self._drive_deadline = None

    def _close_serial_quietly(self) -> None:
        if self._serial is None:
            return
        try:
            self._serial.close()
        except Exception:
            pass


def _run_odometry_test(
    driver: RoombaDriver,
    *,
    samples: int = 10,
    interval: float = 0.5,
    output=print,
) -> int:
    """Run the read-only hardware odometry check."""
    try:
        driver.read_odometry()
        previous_left = driver.odometry.prev_left
        previous_right = driver.odometry.prev_right
        output(f"Baseline: L={previous_left} R={previous_right}")

        for index in range(1, samples + 1):
            time.sleep(interval)
            pose = driver.read_odometry()
            left = driver.odometry.prev_left
            right = driver.odometry.prev_right
            left_delta = Odometry.tick_diff(left, previous_left)
            right_delta = Odometry.tick_diff(right, previous_right)
            output(
                f"[{index:02d}/{samples:02d}] "
                f"L={left} R={right} "
                f"dL={left_delta:+d} dR={right_delta:+d} "
                f"x={pose.x:.6f} y={pose.y:.6f} theta={pose.theta:.6f}"
            )
            previous_left = left
            previous_right = right
    except RoombaCommunicationError as exc:
        output(f"Odometry test aborted: {exc}")
        return 1
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Roomba 680 UART hardware check")
    parser.add_argument("--port", default=DEFAULT_PORT)
    test_mode = parser.add_mutually_exclusive_group()
    test_mode.add_argument(
        "--drive-test",
        action="store_true",
        help="explicitly allow a 30 mm/s straight drive for 0.5 seconds",
    )
    test_mode.add_argument(
        "--odom-test",
        action="store_true",
        help="read and print odometry 10 times without driving the wheels",
    )
    args = parser.parse_args(argv)

    print(f"Connecting read-only test to {args.port} at {BAUDRATE} bps")
    with RoombaDriver(port=args.port) as driver:
        if args.odom_test:
            return _run_odometry_test(driver)
        left, right = driver.read_encoders()
        print(f"Encoders: left={left}, right={right}")
        if args.drive_test:
            print("Drive test: 30 mm/s for 0.5 seconds")
            driver.drive_direct(30, 30)
            time.sleep(0.5)
            driver.stop()
            print("Drive test stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

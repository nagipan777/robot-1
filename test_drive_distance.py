
import math
import time
import serial

from odometry import Odometry

PORT = "/dev/cu.usbserial-A10QJ1R4"
BAUD = 19200

# Motion settings
TARGET_DISTANCE = 0.50  # meters
SPEED = 50              # mm/s
SLOW_SPEED = 25         # mm/s
SAMPLE_INTERVAL = 0.1
MAX_DURATION = 25.0

# Heading correction
KP = 100.0              # mm/s per radian
MAX_CORRECTION = 15     # mm/s

# Safety
STALL_TIMEOUT = 2.0
MIN_PROGRESS = 0.002    # meters


def send_drive(ser, right, left):
    """Roomba OI Drive Direct (145)."""
    cmd = (
        bytes([145])
        + int(right).to_bytes(2, "big", signed=True)
        + int(left).to_bytes(2, "big", signed=True)
    )
    ser.write(cmd)
    ser.flush()


def read_encoders(ser):
    """OI Query List: encoder packets 43 and 44."""
    ser.reset_input_buffer()
    ser.write(bytes([149, 2, 43, 44]))
    ser.flush()

    data = ser.read(4)

    if len(data) != 4:
        raise RuntimeError(
            f"Encoder RX error: {len(data)}/4 "
            f"HEX={data.hex()}"
        )

    left = int.from_bytes(data[:2], "big")
    right = int.from_bytes(data[2:], "big")

    return left, right


def main():
    odom = Odometry()

    with serial.Serial(
        PORT,
        BAUD,
        timeout=0.5,
        write_timeout=1.0,
    ) as ser:

        try:
            # OI Start -> Safe
            ser.write(bytes([128]))
            ser.flush()
            time.sleep(0.3)

            ser.write(bytes([131]))
            ser.flush()
            time.sleep(0.3)

            # Initialize odometry
            left, right = read_encoders(ser)
            odom.update(left, right)

            print("DRIVE START")
            print(
                "TIME   DIST(m)   X(m)   Y(m)   "
                "THETA(deg)"
            )

            start = time.monotonic()
            last_progress = start
            progress_reference = 0.0

            while True:
                elapsed = time.monotonic() - start

                if elapsed >= MAX_DURATION:
                    print("TIMEOUT - STOP")
                    break

                left, right = read_encoders(ser)
                pose = odom.update(left, right)

                # Forward displacement along starting axis
                distance = pose.x
                heading = pose.theta

                print(
                    f"{elapsed:5.2f}  "
                    f"{distance:+.4f}  "
                    f"{pose.x:+.4f}  "
                    f"{pose.y:+.4f}  "
                    f"{math.degrees(heading):+8.2f}"
                )

                if distance >= TARGET_DISTANCE:
                    print("TARGET REACHED")
                    break

                if distance >= progress_reference + MIN_PROGRESS:
                    progress_reference = distance
                    last_progress = time.monotonic()

                if time.monotonic() - last_progress > STALL_TIMEOUT:
                    print("NO PROGRESS - STOP")
                    break

                remaining = TARGET_DISTANCE - distance

                base_speed = (
                    SLOW_SPEED if remaining < 0.08
                    else SPEED
                )

                # Differential heading correction
                correction = max(
                    -MAX_CORRECTION,
                    min(MAX_CORRECTION, KP * heading)
                )

                right_speed = int(base_speed + correction)
                left_speed = int(base_speed - correction)

                send_drive(
                    ser,
                    right=right_speed,
                    left=left_speed
                )

                time.sleep(SAMPLE_INTERVAL)

            send_drive(ser, 0, 0)
            print("WHEELS STOP")
            time.sleep(0.2)

            # Final encoder reading
            left, right = read_encoders(ser)
            pose = odom.update(left, right)

            print("\n--- FINAL RESULT ---")
            print(f"x = {pose.x:.4f} m")
            print(f"y = {pose.y:.4f} m")
            print(
                f"theta = "
                f"{math.degrees(pose.theta):.2f} deg"
            )

            error = TARGET_DISTANCE - pose.x
            print(f"distance error = {error:+.4f} m")

        finally:
            try:
                send_drive(ser, 0, 0)
            except Exception as exc:
                print("STOP ERROR:", exc)


if __name__ == "__main__":
    main()

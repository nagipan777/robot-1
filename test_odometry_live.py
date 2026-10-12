
import csv
import math
import time
import serial

from odometry import Odometry

# Serial settings
PORT = "/dev/cu.usbserial-A10QJ1R4"
BAUD = 19200

# Test settings
DRIVE_SPEED = 50       # mm/s
DRIVE_DURATION = 1.0   # seconds
SAMPLE_INTERVAL = 0.2  # seconds

CSV_FILE = "odometry_log.csv"


def send_drive(ser, right, left):
    """Roomba OI Drive Direct (145)"""
    cmd = (
        bytes([145])
        + int(right).to_bytes(2, "big", signed=True)
        + int(left).to_bytes(2, "big", signed=True)
    )
    ser.write(cmd)
    ser.flush()


def read_encoders(ser):
    """Read OI encoder packets 43 and 44."""
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
    records = []

    with serial.Serial(
        PORT,
        BAUD,
        timeout=0.5,
        write_timeout=1.0,
    ) as ser:

        try:
            # Start -> Safe
            ser.write(bytes([128]))
            ser.flush()
            time.sleep(0.3)

            ser.write(bytes([131]))
            ser.flush()
            time.sleep(0.3)

            # Initial encoder values
            left, right = read_encoders(ser)
            odom.update(left, right)

            print(f"START L={left}, R={right}")
            print("TIME    X(m)    Y(m)    THETA(deg)")

            records.append([0.0, 0.0, 0.0, 0.0])

            # Clockwise rotation
            send_drive(
                ser,
                right=-DRIVE_SPEED,
                left=DRIVE_SPEED,
            )

            start = time.monotonic()
            next_sample = start + SAMPLE_INTERVAL

            while True:
                now = time.monotonic()

                if now >= start + DRIVE_DURATION:
                    break

                sleep_time = next_sample - now
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if time.monotonic() >= start + DRIVE_DURATION:
                    break

                left, right = read_encoders(ser)
                pose = odom.update(left, right)

                elapsed = time.monotonic() - start
                theta_deg = math.degrees(pose.theta)

                print(
                    f"{elapsed:5.2f}  "
                    f"{pose.x:+.4f}  "
                    f"{pose.y:+.4f}  "
                    f"{theta_deg:+8.2f}"
                )

                records.append([
                    elapsed,
                    pose.x,
                    pose.y,
                    theta_deg,
                ])

                next_sample += SAMPLE_INTERVAL

            # Stop before final reading
            send_drive(ser, 0, 0)
            print("WHEELS STOP")
            time.sleep(0.2)

            left, right = read_encoders(ser)
            pose = odom.update(left, right)

            elapsed = time.monotonic() - start
            theta_deg = math.degrees(pose.theta)

            records.append([
                elapsed,
                pose.x,
                pose.y,
                theta_deg,
            ])

            print("\n--- FINAL ODOMETRY ---")
            print(f"x     = {pose.x:.4f} m")
            print(f"y     = {pose.y:.4f} m")
            print(f"theta = {theta_deg:.2f} deg")

        finally:
            # Emergency stop on error or Ctrl+C
            try:
                send_drive(ser, 0, 0)
            except Exception as exc:
                print("STOP ERROR:", exc)

    # Save CSV after serial port is closed
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s", "x_m", "y_m", "theta_deg"
        ])
        writer.writerows(records)

    print(f"CSV saved: {CSV_FILE}")


if __name__ == "__main__":
    main()

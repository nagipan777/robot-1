
import serial
import time
import math

from odometry import Odometry

PORT = "/dev/cu.usbserial-A10QJ1R4"
BAUD = 19200

DRIVE_SPEED = 50
DRIVE_DURATION = 1.0


def send_drive(ser, right, left):
    """OI Drive Direct (145)"""
    cmd = (
        bytes([145])
        + int(right).to_bytes(2, "big", signed=True)
        + int(left).to_bytes(2, "big", signed=True)
    )
    ser.write(cmd)
    ser.flush()


def read_encoders(ser):
    """OI Query List: Packet 43, 44"""
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
        timeout=1.0,
        write_timeout=1.0
    ) as ser:

        try:
            # OI Start -> Safe
            ser.write(bytes([128]))
            ser.flush()
            time.sleep(0.3)

            ser.write(bytes([131]))
            ser.flush()
            time.sleep(0.3)

            # Initial encoder readings
            left0, right0 = read_encoders(ser)
            odom.update(left0, right0)

            print(f"START L={left0}, R={right0}")

            # Rotation test
            print("WHEELS START")
            send_drive(
                ser,
                right=-DRIVE_SPEED,
                left=DRIVE_SPEED
            )

            time.sleep(DRIVE_DURATION)

            # Stop
            send_drive(ser, 0, 0)
            print("WHEELS STOP")
            time.sleep(0.2)

            # Final encoder readings
            left1, right1 = read_encoders(ser)

            print(f"END   L={left1}, R={right1}")

            dl = Odometry.tick_diff(left1, left0)
            dr = Odometry.tick_diff(right1, right0)

            print(f"DELTA L={dl:+d}, R={dr:+d}")

            # Pose estimation
            pose = odom.update(left1, right1)

            print("\n--- ODOMETRY ---")
            print(f"x     = {pose.x:.4f} m")
            print(f"y     = {pose.y:.4f} m")
            print(
                f"theta = "
                f"{math.degrees(pose.theta):.2f} deg"
            )

        finally:
            try:
                send_drive(ser, 0, 0)
            except Exception as exc:
                print("STOP ERROR:", exc)


if __name__ == "__main__":
    main()


import math
import time
import serial

from odometry import Odometry

PORT = "/dev/cu.usbserial-A10QJ1R4"
BAUD = 19200

TARGET_DEG = -90.0
SPEED = 50
SLOW_SPEED = 25
SAMPLE_INTERVAL = 0.1
MAX_DURATION = 8.0


def send_drive(ser, right, left):
    cmd = (
        bytes([145])
        + int(right).to_bytes(2, "big", signed=True)
        + int(left).to_bytes(2, "big", signed=True)
    )
    ser.write(cmd)
    ser.flush()


def read_encoders(ser):
    ser.reset_input_buffer()
    ser.write(bytes([149, 2, 43, 44]))
    ser.flush()

    data = ser.read(4)

    if len(data) != 4:
        raise RuntimeError(
            f"Encoder RX error: {len(data)}/4"
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
            ser.write(bytes([128]))
            ser.flush()
            time.sleep(0.3)

            ser.write(bytes([131]))
            ser.flush()
            time.sleep(0.3)

            left, right = read_encoders(ser)
            odom.update(left, right)

            print("TURN START")
            start = time.monotonic()
            last_progress = start
            previous_angle = 0.0

            while True:
                elapsed = time.monotonic() - start

                if elapsed > MAX_DURATION:
                    print("TIMEOUT")
                    break

                left, right = read_encoders(ser)
                pose = odom.update(left, right)
                angle = math.degrees(pose.theta)

                print(
                    f"t={elapsed:.2f}s "
                    f"theta={angle:+.2f} deg"
                )

                if angle <= TARGET_DEG:
                    print("TARGET REACHED")
                    break

                if abs(angle - previous_angle) >= 0.5:
                    last_progress = time.monotonic()
                    previous_angle = angle

                if time.monotonic() - last_progress > 1.0:
                    print("NO PROGRESS - STOP")
                    break

                remaining = TARGET_DEG - angle

                if abs(remaining) < 15:
                    speed = SLOW_SPEED
                else:
                    speed = SPEED

                send_drive(
                    ser,
                    right=-speed,
                    left=speed,
                )

                time.sleep(SAMPLE_INTERVAL)

            send_drive(ser, 0, 0)
            print("WHEELS STOP")

            time.sleep(0.2)
            left, right = read_encoders(ser)
            pose = odom.update(left, right)

            print("\nFINAL RESULT")
            print(
                f"theta = "
                f"{math.degrees(pose.theta):.2f} deg"
            )
            print(f"x = {pose.x:.4f} m")
            print(f"y = {pose.y:.4f} m")

        finally:
            try:
                send_drive(ser, 0, 0)
            except Exception as exc:
                print("STOP ERROR:", exc)


if __name__ == "__main__":
    main()

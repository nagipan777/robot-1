
import serial
import time

PORT = "/dev/cu.usbserial-A10QJ1R4"
BAUD = 19200
TEST_COUNT = 100
INTERVAL = 0.5

QUERY = bytes([149, 2, 43, 44])


def encoder_delta(current, previous):
    """16bitエンコーダーの折り返しを処理"""
    return (current - previous + 32768) % 65536 - 32768


def read_encoders(ser):
    """左右エンコーダーを読み取る"""
    ser.write(QUERY)
    ser.flush()

    data = ser.read(4)

    if len(data) != 4:
        return None, data.hex()

    left = int.from_bytes(data[:2], "big")
    right = int.from_bytes(data[2:], "big")

    return (left, right), data.hex()


def main():
    previous = None
    success = 0
    failed = 0
    rejected = 0

    with serial.Serial(
        PORT,
        BAUD,
        timeout=1.0
    ) as ser:

        # Roomba OI Start
        ser.write(bytes([128]))
        ser.flush()
        time.sleep(0.5)

        # 初回だけ受信バッファをクリア
        ser.reset_input_buffer()

        print("Encoder diagnostic started")

        for i in range(TEST_COUNT):
            result, raw = read_encoders(ser)

            if result is None:
                failed += 1
                print(
                    f"[{i+1:03d}] RX ERROR "
                    f"HEX={raw}"
                )
                # 不完全応答後は同期が保証できないため終了
                print("RX synchronization lost")
                break

            left, right = result

            if previous is None:
                print(
                    f"[{i+1:03d}] "
                    f"L={left}, R={right} INITIAL"
                )
                previous = result
                success += 1
                time.sleep(INTERVAL)
                continue

            dl = encoder_delta(left, previous[0])
            dr = encoder_delta(right, previous[1])

            # 停止状態専用フィルター
            if abs(dl) > 100 or abs(dr) > 100:
                rejected += 1
                print(
                    f"[{i+1:03d}] REJECT "
                    f"L={left}, R={right} "
                    f"dL={dl}, dR={dr} "
                    f"HEX={raw}"
                )
                # 異常値を基準値に採用しない
                # フレーム同期も疑わしいため終了
                break

            success += 1
            previous = result

            print(
                f"[{i+1:03d}] "
                f"L={left:5d}, R={right:5d} "
                f"dL={dl:+5d}, dR={dr:+5d}"
            )

            time.sleep(INTERVAL)

    print("\n===== RESULT =====")
    print(f"SUCCESS : {success}")
    print(f"RX ERROR: {failed}")
    print(f"REJECT  : {rejected}")
    print(f"SUCCESS RATE: {success / TEST_COUNT * 100:.1f}%")


if __name__ == "__main__":
    main()

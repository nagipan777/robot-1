"""Roomba 680 OI 通信テスト (CP2102 経由)

使い方:
    pip install pyserial
    python roomba_test.py COM3          # Windows
    python roomba_test.py /dev/ttyUSB0  # ラズパイ / Linux

事前に:
    - CLEANボタンを押してルンバを起こしておく(ライトが点いた状態)
    - 配線: ルンバTXD→CP2102 RXD / ルンバRXD→CP2102 TXD / GND共通
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-A10QJ1R4"
BAUD = 19200  # 680は19200 8N1

START = 128          # 0x80 パッシブモードへ
SENSORS = 142        # 0x8E センサー取得
GROUP0_LEN = 26      # パケットグループ0 (ID 7-26) のバイト数


def main():
    print(f"open {PORT} @ {BAUD}bps 8N1")
    s = serial.Serial(
        PORT,
        BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    time.sleep(0.5)
    s.reset_input_buffer()

    # 起きていない場合に備えて、Startを数回送る
    for i in range(3):
        s.write(bytes([START]))
        s.flush()
        print(f"[{i + 1}] Start (0x80) 送信")
        time.sleep(0.3)

    s.reset_input_buffer()

    # Sensors グループ0 を要求
    s.write(bytes([SENSORS, 0]))
    s.flush()
    print("Sensors (0x8E 0x00) 送信")
    time.sleep(0.2)

    data = s.read(GROUP0_LEN)
    print(f"受信 {len(data)} バイト (期待値 {GROUP0_LEN})")

    if len(data) == 0:
        print("→ 無反応。ルンバが寝ている/配線(TX・RX交差, GND)/ボーレートを確認")
    elif len(data) < GROUP0_LEN:
        print("→ 一部のみ受信。ノイズやレベル不足の可能性")
        print("  raw:", data.hex(" "))
    else:
        print("→ 通信成功!")
        print("  raw:", data.hex(" "))
        # グループ0の先頭: ID7 バンプ&ウィールドロップ, ID8 崖(左) ...
        print(f"  Bump/WheelDrop(ID7) = 0b{data[0]:08b}")
        print(f"  Wall(ID8)           = {data[1]}")

    s.close()


if __name__ == "__main__":
    main()
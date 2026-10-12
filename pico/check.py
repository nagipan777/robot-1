import machine, time
uart = machine.UART(0, baudrate=19200, tx=machine.Pin(0), rx=machine.Pin(1))
for i in range(40):
    uart.write(bytes([128])); time.sleep_ms(100)              # Start
    uart.write(bytes([140, 0, 1, 72, 32])); time.sleep_ms(100)  # 曲0を登録
    uart.write(bytes([141, 0]))                               # 曲0を再生
    time.sleep_ms(300)
    n = uart.any()
    print(i, "rx:", uart.read(n) if n else None)
    time.sleep_ms(500)
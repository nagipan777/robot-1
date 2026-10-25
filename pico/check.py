import machine, time
uart = machine.UART(0, baudrate=19200, tx=machine.Pin(0), rx=machine.Pin(1))
uart.write(bytes([128])); time.sleep_ms(200)                 # Start
uart.write(bytes([131])); time.sleep_ms(200)                 # Safe
uart.write(bytes([140, 0, 1, 72, 32])); time.sleep_ms(100)   # 曲0を登録
uart.write(bytes([141, 0]))# 再生
uart.write(bytes([145, 0, 100, 0, 100]))  # 右100mm/s、左100mm/s
time.sleep_ms(1000)
uart.write(bytes([145, 0, 0, 0, 0]))  
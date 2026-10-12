import machine
import time


led = machine.Pin("LED", machine.Pin.OUT)

try:
    while True:
        led.on()
        time.sleep(0.5)
        led.off()
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    led.off()

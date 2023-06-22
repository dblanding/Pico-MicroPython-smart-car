from bno08x_rvc import BNO08x_RVC
from machine import Pin, UART
import time

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
# uart.init(rxbuf=2048)
print(uart)
rvc = BNO08x_RVC(uart)
while True:
    yaw, pitch, roll, x_accel, y_accel, z_accel = rvc.heading
    # print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
    # print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
    print(yaw)
    time.sleep(0.01)

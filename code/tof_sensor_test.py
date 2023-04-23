"""
from:
http://www.coderdojotc.org/micropython/robots/02-base-bot/
"""

import time
from machine import Pin, I2C
import VL53L0X

sda=machine.Pin(12)
scl=machine.Pin(13)
i2c=machine.I2C(0, sda=sda, scl=scl)

print(i2c.scan())

# Create a VL53L0X object
tof = VL53L0X.VL53L0X(i2c)
tof.start() # startup the sensor
while True:
# Start ranging
    dist = tof.read()
    print(dist)
    time.sleep(.1)

import encoder_rp2 as encoder
from machine import Pin
from time import sleep

e_right = encoder.Encoder(0, Pin(14))
e_left = encoder.Encoder(1, Pin(12))

while True:
    sleep(0.5)
    print(e_left.value(), '\t', e_right.value())

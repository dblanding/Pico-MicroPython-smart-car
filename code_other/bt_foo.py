"""
Receive 4 comma separated values sent from joystick BT module
like this:

-150,150,150,-150\r\n
150,-151,-150,150\r\n

BT module sends one character (one byte) at a time.
Each received byte needs to be acknowledged to get the next one.

The 4 values are drive speeds for 4 omni-car motors.
The differential-drive car only needs the first 2.
"""

from machine import UART,Pin, PWM
from time import sleep

MIN_PWM = 15_000  # min pwm value motors can handle

uart = UART(0, 9600)
print(dir(uart))
byte_str = b''

# pins connected to L298N Motor Drive Controller Board
ena = PWM(Pin(21))
in1 = Pin(20, Pin.OUT, value=0)
in2 = Pin(19, Pin.OUT, value=0)
in3 = Pin(18, Pin.OUT, value=0)
in4 = Pin(17, Pin.OUT, value=0)
enb = PWM(Pin(16))

ena.freq(1_000)
ena.freq(1_000)

def drive_motors(spd_a, spd_b):
    if spd_a < 0:
        # drive forward (rotate CCW)
        in1.value(0)
        in2.value(1)
    else:
        # rotate CW
        in1.value(1)
        in2.value(0)
    
    if spd_b > 0:
        # drive forward (rotate CW)
        in3.value(0)
        in4.value(1)
    else:
        # rotate CCW
        in3.value(1)
        in4.value(0)
    
    pwm_a = abs(spd_a)
    pwm_b = abs(spd_b)
    if pwm_a < MIN_PWM:
        pwm_a = 0
    if pwm_b < MIN_PWM:
        pwm_b = 0

    ena.duty_u16(pwm_a)
    enb.duty_u16(pwm_b)

while True:
    if uart.any():
        value = uart.readline()
        if value == b'\r':
            four_val_str = byte_str.decode('utf-8')
            v1_str, v2_str, v3_str, v4_str = four_val_str.split(',')
            v1, v2 = int(v1_str), int(v2_str)
            # scale speed to +/- 65,535
            MULT = 350
            drive_motors(v1 * MULT, v2 * MULT)

        elif value == b'\n':
            byte_str = b''
        else:
            byte_str += value
        # send a character to ask for more data
        uart.write("A")
        uart.flush()
        

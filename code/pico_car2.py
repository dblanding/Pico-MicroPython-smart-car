"""
MicroPython code for Pico car project using:
* Raspberry Pi Pico mounted on differential drive car
* 56:1 gear motors with encoders
* Asynchrounous webserver enabling remote control
* Encoder feedback used in fwd and back modes to
    * drive motors at TARGET_TICK_RATE for 0.9 meter
    * PID (+C for R & L) keeps wheels at target spd and
      in lock step despite R & L motor differences
"""

import encoder_rp2 as encoder
import gc
import math
import network
import uasyncio as asyncio
import _thread
from machine import Pin, PWM
import time
from secrets import secrets
from motors import Motors
from odometer import Odometer
from parameters import TICKS_PER_METER, TARGET_TICK_RATE

ssid = secrets['ssid']
password = secrets['wifi_password']

# Set drive_mode to one of: 'F', 'B', 'R', 'L', 'S'
drive_mode = 'S'  # stop

# Set PWM value for motors during in-place turns
turn_spd = 20_000

html = """<!DOCTYPE html>
<html>
    <head>
        <title>Robot Control</title>
    </head>

    <body>
        <center><b>

        <form action="./forward">
            <input type="submit" value="Forward" style="height:120px; width:120px" />
        </form>

        <table>
            <tr>
                <td><form action="./left">
                <input type="submit" value="Left" style="height:120px; width:120px" />
                </form></td>

                <td><form action="./stop">
                <input type="submit" value="Stop" style="height:120px; width:120px" />
                </form></td>

                <td><form action="./right">
                <input type="submit" value="Right" style="height:120px; width:120px" />
                </form></td>
            </tr>
        </table>

        <form action="./back">
        <input type="submit" value="Back" style="height:120px; width:120px" />
        </form>

    </body>
</html>
"""

# setup encoders
enc_b = encoder.Encoder(0, Pin(14))
enc_a = encoder.Encoder(1, Pin(12))

# setup onboard LED
led = Pin("LED", Pin.OUT, value=0)

# setup pins connected to L298N Motor Drive Controller Board
ena = PWM(Pin(21))
in1 = Pin(20, Pin.OUT, value=0)
in2 = Pin(19, Pin.OUT, value=0)
in3 = Pin(18, Pin.OUT, value=0)
in4 = Pin(17, Pin.OUT, value=0)
enb = PWM(Pin(16))

ena.freq(1_000)
enb.freq(1_000)

def set_mtr_dirs(a_mode, b_mode):
    """Set motor direction pins for both motors.
    options are: 'FWD', 'REV', 'OFF'."""

    if a_mode == 'FWD':
        in1.value(0)
        in2.value(1)
    elif a_mode == 'REV':
        in1.value(1)
        in2.value(0)
    else:  # Parked
        in1.value(0)
        in2.value(0)

    if b_mode == 'FWD':
        in3.value(0)
        in4.value(1)
    elif b_mode == 'REV':
        in3.value(1)
        in4.value(0)
    else:  # Parked
        in3.value(0)
        in4.value(0)

def set_mtr_spds(a_PWM_val, b_PWM_val):
    """set speeds for both a and b motors
    allowable values are u16 integers (< 65_536)"""

    ena.duty_u16(int(a_PWM_val))
    enb.duty_u16(int(b_PWM_val))

def move_forward():
    # print('move forward')
    set_mtr_dirs('FWD', 'FWD')

def move_backward():
    # print('move backward')
    set_mtr_dirs('REV', 'REV')

def move_stop():
    # print('STOP')
    set_mtr_dirs('OFF', 'OFF')
    set_mtr_spds(0, 0)

def turn_left():
    # print('turn left')
    set_mtr_dirs('REV', 'FWD')
    # set_mtr_spds(turn_spd, turn_spd)

def turn_right():
    # print('turn right')
    set_mtr_dirs('FWD', 'REV')
    set_mtr_spds(turn_spd, turn_spd)

#Stop the robot ASAP
move_stop()

wlan = network.WLAN(network.STA_IF)

def connect():
    wlan.active(True)
    wlan.config(pm = 0xa11140) # Disable power-save mode
    wlan.connect(ssid, password)

    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        print('waiting for connection...')
        time.sleep(1)

    if wlan.status() != 3:
        raise RuntimeError('network connection failed')
    else:
        print('connected')
        status = wlan.ifconfig()
        ip = status[0]
        print('ip = ' + status[0])
    return ip

async def serve_client(reader, writer):
    global drive_mode
    # print("Client connected")
    request_line = await reader.readline()
    request_line = str(request_line)
    # print("Request:", request_line)
    # We are not interested in HTTP request headers, skip them
    while await reader.readline() != b"\r\n":
        pass

    try:
        command = request_line.split()[1]
    except IndexError:
        pass
    print("command = ", command)
    if command == '/forward?':
        drive_mode  = 'F'
    elif command =='/left?':
        drive_mode = 'L'
    elif command =='/stop?':
        drive_mode = 'S'
    elif command =='/right?':
        drive_mode = 'R'
    elif command =='/back?':
        drive_mode = 'B'

    response = html
    writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    writer.write(response)

    await writer.drain()
    await writer.wait_closed()
    # print("Client disconnected")

async def main():
    global drive_mode
    print('Connecting to Network...')
    connect()

    print('Setting up webserver...')
    asyncio.create_task(asyncio.start_server(serve_client, "0.0.0.0", 80))
    start_time = time.ticks_ms()
    prev_time = time.ticks_ms()
    prev_mode = 'S'  # stop
    odom = Odometer()
    while True:

        # Flash LED
        led.toggle()

        # update odometer
        enc_a_val = enc_a.value()
        enc_b_val = enc_b.value()
        pose = odom.update(enc_a_val, enc_b_val)
        
        # Check to see if drive_mode has changed
        if drive_mode != prev_mode:
            prev_mode = drive_mode
            mtrs = None
            gc.collect()
            if drive_mode == 'F':
                # Instantiate Motors object
                mtrs = Motors(TARGET_TICK_RATE)
                
                # Set direction pins
                move_forward()
                
                # save starting value of encoders
                enc_a_start = enc_a.value()
                enc_b_start = enc_b.value()

            elif drive_mode == 'B':
                # Instantiate Motors object
                mtrs = Motors(TARGET_TICK_RATE, fwd=False)
                
                # Set direction pins
                move_backward()
                
                # save starting value of encoders
                enc_a_start = enc_a.value()
                enc_b_start = enc_b.value()

            elif drive_mode == 'R':
                # Set direction pins
                turn_right()

            elif drive_mode == 'L':
                # Set direction pins
                turn_left()
                
            elif drive_mode == 'S':
                # Stop motors
                move_stop()
                del(mtrs)

        # Drive forward to distance
        if drive_mode == 'F':
            goal_distance = 0.9  # meters
            goal_a = enc_a_start + (goal_distance * TICKS_PER_METER)
            if enc_a.value() < goal_a:
                pwm_a, pwm_b = mtrs.update(enc_a.value(), enc_b.value())
                set_mtr_spds(pwm_a, pwm_b)
                # print(pose)
            else:
                drive_mode = 'S'
                move_stop()
                del(mtrs)

        # Drive backward to distance
        if drive_mode == 'B':
            goal_distance = 0.9  # meters
            goal_a = enc_a_start - (goal_distance * TICKS_PER_METER)
            if enc_a.value() > goal_a:
                pwm_a, pwm_b = mtrs.update(enc_a.value(), enc_b.value())
                set_mtr_spds(pwm_a, pwm_b)
                # print(pose)
            else:
                drive_mode = 'S'
                move_stop()
                del(mtrs)

        # Turn left to angle 90 deg
        if drive_mode == 'L':
            goal_angle = math.pi / 2
            _, _, curr_angle = pose
            if curr_angle < goal_angle:
                set_mtr_spds(turn_spd, turn_spd)
                print(pose)
            else:
                drive_mode = 'S'
                move_stop()
             
        # Turn right to angle 0 deg
        if drive_mode == 'R':
            goal_angle = 0
            _, _, curr_angle = pose
            if curr_angle > goal_angle:
                set_mtr_spds(turn_spd, turn_spd)
                print(pose)
            else:
                drive_mode = 'S'
                move_stop()

        await asyncio.sleep(0.1)

try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()

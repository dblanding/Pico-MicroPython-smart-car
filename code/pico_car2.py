"""
MicroPython code for Pico car project using:
* Raspberry Pi Pico mounted on differential drive car
* 56:1 gear motors with encoders
* Asynchrounous webserver enables remote control
* In fwd or back modes, use encoder feedback to drive motors at target speed
* BNO08x IMU
"""

import encoder_rp2 as encoder
import gc
import network
import uasyncio as asyncio
import _thread
from machine import Pin, PWM, UART
import time
from secrets import secrets
from motors import Motors
from bno08x_rvc import BNO08x_RVC, RVCReadTimeoutError

# setup IMU in RVC mode on UART
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
# print(dir(uart))
rvc = BNO08x_RVC(uart)
# print(dir(rvc))
yaw, r, p, x, y, z = rvc.heading

ssid = secrets['ssid']
password = secrets['wifi_password']

# Set drive_mode to one of: 'F', 'B', 'R', 'L', 'S'
drive_mode = 'S'  # stop

# Motor parameters
target_tick_rate = 4000  # ticks per sec
TICKS_PER_METER = 11_514

# Set motor speed during in-place turns
turn_spd = target_tick_rate * 6


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
ena.freq(1_000)

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
    # set_mtr_spds(mtr_spd_a, mtr_spd_b)

def move_backward():
    # print('move backward')
    set_mtr_dirs('REV', 'REV')
    # set_mtr_spds(mtr_spd_a, int(mtr_spd_b))

def move_stop():
    # print('STOP')
    set_mtr_dirs('OFF', 'OFF')
    set_mtr_spds(0, 0)

def turn_left():
    # print('turn left')
    set_mtr_dirs('REV', 'FWD')
    set_mtr_spds(turn_spd, turn_spd)

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
    prev_time = start_time
    prev_mode = 'S'  # stop
    prev_yaw = 0
    loop_count = 0
    while True:
        loop_count += 1
        
        # Get data from IMU
        try:
            yaw, *_ = rvc.heading
            prev_yaw = yaw
        except RVCReadTimeoutError:
            yaw = prev_yaw
        # print(yaw)

        if loop_count == 10:
            loop_count = 0

            # Flash LED
            led.toggle()

            # Check to see if drive_mode has changed
            if drive_mode != prev_mode:
                prev_mode = drive_mode
                mtrs = None
                gc.collect()
                if drive_mode == 'F':
                    # Instantiate Motor objects
                    # mtr_a = Motor(target_tick_rate, 11)
                    # mtr_b = Motor(target_tick_rate, 13.5)
                    mtrs = Motors(target_tick_rate)
                    
                    # Set direction pins
                    move_forward()
                    
                    # save starting value of encoders
                    enc_a_start = enc_a.value()
                    enc_b_start = enc_b.value()

                elif drive_mode == 'B':
                    # Instantiate Motor objects
                    # mtr_a = Motor(target_tick_rate, 13)
                    # mtr_b = Motor(target_tick_rate, 14)
                    mtrs = Motors(target_tick_rate, fwd=False)
                    
                    # Set direction pins
                    move_backward()
                    
                    # save starting value of encoders
                    enc_a_start = enc_a.value()
                    enc_b_start = enc_b.value()

                elif drive_mode == 'R':
                    # Execute right turn
                    turn_right()
                    del(mtrs)

                elif drive_mode == 'L':
                    # Execute left turn
                    turn_left()
                    del(mtrs)

                elif drive_mode == 'S':
                    # Stop motors
                    move_stop()
                    del(mtrs)

            # Drive forward to distance (m)
            if drive_mode == 'F':
                goal_distance = 0.9  # meter
                goal_a = enc_a_start + (goal_distance * TICKS_PER_METER)
                if enc_a.value() < goal_a:
                    # pwm_a = mtr_a.update(enc_a.value())
                    # pwm_b = mtr_b.update(enc_b.value())
                    pwm_a, pwm_b = mtrs.update(enc_a.value(), enc_b.value(), yaw)
                    set_mtr_spds(pwm_a, pwm_b)
                else:
                    drive_mode = 'S'
                    move_stop()
                    del(mtrs)

            # Drive backward to distance (m)
            if drive_mode == 'B':
                goal_distance = 0.9  # meter
                goal_a = enc_a_start - (goal_distance * TICKS_PER_METER)
                if enc_a.value() > goal_a:
                    # pwm_a = mtr_a.update(enc_a.value())
                    # pwm_b = mtr_b.update(enc_b.value())
                    pwm_a, pwm_b = mtrs.update(enc_a.value(), enc_b.value(), yaw)
                    set_mtr_spds(pwm_a, pwm_b)
                else:
                    drive_mode = 'S'
                    move_stop()
                    del(mtrs)

            # Calculate pose (odometrically)
            # To do...

        # short delay time keeps the IMU data fresh
        # everything else goes every tenth time thru loop 
        await asyncio.sleep(0.01)

try:
    asyncio.run(main())
finally:
    uart.deinit()
    asyncio.new_event_loop()

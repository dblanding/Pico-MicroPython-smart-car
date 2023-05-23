"""
MicroPython code for Pico car project using:
* Raspberry Pi Pico mounted on differential drive car
* Motor encoders used for closed-loop speed control
* Asynchrounous webserver enables remote control
"""

import encoder_rp2 as encoder
import network
import uasyncio as asyncio
from machine import Pin, PWM
import time
from secrets import secrets

ssid = secrets['ssid']
password = secrets['wifi_password']

# Operate motors at a moderate speed
target_tick_rate = 5000  # ticks per sec

# Account for intrinsic differences between motors a and b
mult_a = 5.4
mult_b = 6.7

# Nominal PWM signal for each motor resulting in straight travel
mtr_spd_a = int(target_tick_rate * mult_a)
mtr_spd_b = int(target_tick_rate * mult_b)

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

enc_b = encoder.Encoder(0, Pin(2))
enc_a = encoder.Encoder(1, Pin(0))


# setup onboard LED
led = Pin("LED", Pin.OUT, value=0)

# pins connected to L298N Motor Drive Controller Board
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

    ena.duty_u16(a_PWM_val)
    enb.duty_u16(b_PWM_val)

def move_forward():
    mtr_spd = 30_000
    print('move forward')
    set_mtr_dirs('FWD', 'FWD')
    set_mtr_spds(mtr_spd, mtr_spd)
    start_time = time.ticks_ms()
    prev_time = start_time
    time.sleep(0.05)
    prev_enc_a = enc_a.value()
    prev_enc_b = enc_b.value()
    deadline = time.ticks_add(start_time, 2000)  # 2 seconds
    accum_err_a = []
    accum_err_b = []
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        curr_time = time.ticks_ms()
        curr_enc_a = enc_a.value()
        curr_enc_b = enc_b.value()
        delta_enc_a = curr_enc_a - prev_enc_a
        delta_enc_b = curr_enc_b - prev_enc_b
        delta_time = curr_time - prev_time
        prev_time = curr_time
        prev_enc_a = curr_enc_a
        prev_enc_b = curr_enc_b
        tick_rate_a = (delta_enc_a / delta_time) * 1000
        tick_rate_b = (delta_enc_b / delta_time) * 1000
        
        print(tick_rate_a, tick_rate_b)
        time.sleep(0.1)

def move_forward():
    print('move forward')
    set_mtr_dirs('FWD', 'FWD')
    set_mtr_spds(mtr_spd_a, mtr_spd_b)

def move_backward():
    print('move backward')
    set_mtr_dirs('REV', 'REV')
    set_mtr_spds(mtr_spd_a, int(mtr_spd_b))

def move_stop():
    print('STOP')
    set_mtr_dirs('OFF', 'OFF')
    set_mtr_spds(0, 0)

def move_left():
    print('turn left')
    set_mtr_dirs('REV', 'FWD')
    set_mtr_spds(int(mtr_spd_a * 0.7),
                 int(mtr_spd_b * 0.7))

def move_right():
    print('turn right')
    set_mtr_dirs('FWD', 'REV')
    set_mtr_spds(int(mtr_spd_a * 0.7),
                 int(mtr_spd_b * 0.7))

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
    print("Client connected")
    request_line = await reader.readline()
    request_line = str(request_line)
    print("Request:", request_line)
    # We are not interested in HTTP request headers, skip them
    while await reader.readline() != b"\r\n":
        pass

    try:
        command = request_line.split()[1]
    except IndexError:
        pass
    print("command = ", command)
    if command == '/forward?':
        move_forward()
    elif command =='/left?':
        move_left()
    elif command =='/stop?':
        move_stop()
    elif command =='/right?':
        move_right()
    elif command =='/back?':
        move_backward()

    response = html
    writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    writer.write(response)

    await writer.drain()
    await writer.wait_closed()
    print("Client disconnected")

async def main():
    print('Connecting to Network...')
    connect()

    print('Setting up webserver...')
    asyncio.create_task(asyncio.start_server(serve_client, "0.0.0.0", 80))
    while True:
        # Flash LED
        led.on()
        await asyncio.sleep(0.25)
        led.off()
        await asyncio.sleep(2)

try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
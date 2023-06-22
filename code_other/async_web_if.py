"""
Zumo robot web server revised to run asynchronously
"""

import network
import uasyncio as asyncio
from time import sleep
from machine import Pin
from secrets import secrets

ssid = secrets['ssid']
password = secrets['wifi_password']

onboard = Pin("LED", Pin.OUT, value=0)

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

def move_forward():
    print('move forward')
    
def move_backward():
    print('move backward')

def move_stop():
    print('STOP')

def move_left():
    print('move left')

def move_right():
    print('move right')

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
        sleep(1)

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
        for _ in range(3):
            onboard.on()
            await asyncio.sleep(0.1)
            onboard.off()
            await asyncio.sleep(0.1)

        onboard.on()
        await asyncio.sleep(0.25)
        onboard.off()
        await asyncio.sleep(10)

try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()

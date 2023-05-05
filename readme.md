# Pico-&mu;Python Smart Car Project

![pico car](imgs/pico-car.jpg)

### This Medium article by Dan McCreary [Raspberry Pi Pico Robot in Micropython](https://dmccreary.medium.com/raspberry-pi-pico-robot-in-micropython-51f956486270) inspired me to buiild a similar smart car project
* Further details are presented at [base robot kit project](http://www.coderdojotc.org/micropython/robots/02-base-bot/)
    * Discovered they use [MkDocs](https://www.mkdocs.org/getting-started/). May want to check this out sometime...

### My project will use **at least 2** of these time-of-flight VCSEL sensors

* Googled for info about Using the Raspberry Pi Pico with 2 x VL53L0x in Micropython
    * Found this [Orionrobots video](https://www.youtube.com/watch?v=XQrxPcq2tZ8) which shows:
        * Using two devices, one on each I2C bus
        * How to modify device address in order to have multiple devices on the same bus
        * in both cases, it is important to hold the XSHUT pin high.
    * The library used by Orionrobots is based on Kevin McAleer's [Raspberry Pi Pico & VL53L0X for MicroPython](https://www.youtube.com/watch?v=YBu6GKnN4lk)
    * Unfortunately, The library used (by both Kevin McAleeer and the derivative Orionrobots) only produces values up to 500 mm.
        * Whereas the datasheet shows the VL53L0X should produce data up to 1200 mm.
        * The VL53L0X library (used by McCreary) produces values all the way to 1200 mm. 
            * A diff shows that the libraries used by McAleer (and Orionrobots) is actually a mofified version of the one used by McCreary.
* I thought I would try using the **disparity** of two forward-looking but obliquely oriented (pigeon-toed) sensors as a way to enable the cheap *smart-car* to steer toward a wall along a path that is perpendicular to the wall. This proved to be quite difficult.
    * Without motor encoders, it is very frustrating trying to control steering on differential drive configuration.
    * The motors have a large and varialble amount of static friction, causing the actual motor speed to be unknowable. 
    * Sometimes, the friction is so large, that a motor won't start to turn even with a PWM duty cycle of (40_000 / 65_536)
    * Decided to not pursue this approach any further.
* Instead, I decided to use the Bosch BNO08x IMU sensor in RVC mode to keep track of the car's yaw angle.
* I will use the VCSEL sensors just for distance measurement

### Use the Bosch BNO08x IMU sensor to keep track of the car's **yaw** angle

* I found this [MicroPython library](https://github.com/rdagger/micropython-bno08x-rvc/blob/master/bno08x_rvc.py) (based on this [CircuitPython library](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x_RVC)) which enables access to the BNO08x IMU module (in RVC mode) over the UART serial bus.
* (Truth be known, I was having a bit of difficulty figuring out how to attach more than one device to the I2C bus, so I decided to just sidestep this issue and connect over UART.)
* This primer on [Raspberry Pi Pico Serial Communication Example(MicroPython)](https://electrocredible.com/raspberry-pi-pico-serial-uart-micropython/) provided the detailed understanding I needed to get it all working.

### Use the [L298N motor driver module](https://www.etechnophiles.com/l298n-motor-driver-pin-diagram/)
* Used on my ROS robot and it worked well, so I will use that again.

### Power to the pico will come from 2 different sources:
    1. The USB connection (5V)
    2. The 5V power output from the motor driver board
    
* The Pico datasheet shows a recommended way to accomodate power from 2 sources:

![pico power](imgs/pico-power.png)

* I ordered the P-FET but until it comes, I have hooked up the 5V output from the motor driver board directly to the VSYS pin of the pico, as shown in this wiring diagram. It seems to work OK both with / without the USB connection to the Pico.

![Pico car wiring](imgs/pico-car_bb.png)

* This video [Power for the Raspberry Pi Pico - Guide to using VBUS, VSYS and 3V3 for external power circuits](https://www.youtube.com/watch?v=3PH9jzRsb5E) may provide some good info...

### Add WiFi control

Bob Grant ([Bytes N Bits](https://bytesnbits.co.uk/S)) has produced this series of 3 video tutorials which teach how to build a web interface for controlling a project on the Pi Pico W.
* WiFi Control Your Micropython Project
    * [Youtube video](https://www.youtube.com/watch?v=eym8NpHr9Xw)
    * [Bytes N Bits website page](https://bytesnbits.co.uk/simple-micropython-wifi-connection/)
    * [Github Repo](https://github.com/getis/pi-pico-w-simple-wifi-setup)

* Web Control Panel Part 1: The web server
    * [Youtube video](https://www.youtube.com/watch?v=h18LMskRNMA)
    * [Bytes N Bits website page](https://bytesnbits.co.uk/web-control-panel-web-server/)
    * [Github Repo](https://github.com/getis/micropython-web-control-panel)

* Web Control Panel Part 2: Async & Dual Core web server
    * [Youtube video](https://www.youtube.com/watch?v=PY732g2ZN4g)
    * [Bytes N Bits website page](https://bytesnbits.co.uk/web-control-panel-2-asyncio-dual-core/)
    * [Github Repo](https://github.com/getis/micropython-web-control-panel)

Christopher Barnatt shows a very simple [Pi Pico W WiFi Controlled Robot](https://www.explainingcomputers.com/pi_pico_w_robot.html)
* The robot runs a basic webserver displaying a very simple form with 5 buttons.

![webpage form](imgs/5-button_form.png)

This looks just right for this project. Simple and straightforward. I just need the webserver to run asynchronously so it doesn't block the robot's navigation code.
* `pico_car+webserver.py` is the final code for the pico car with the navigation code and web interface bundled together.


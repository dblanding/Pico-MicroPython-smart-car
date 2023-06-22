"""
MicroPython code for Pico car project using:
* Raspberry Pi Pico mounted on differential drive 'smart car'
* 2 VL53L0x time-of-flight distance sensors
    (keep XSHUT pin on VL53L0X devices pulled high)
* BNO08X IMU reports yaw angle
* Yaw angle feedback enables:
    - car to drive straight
    - make measured turns in place
"""

from machine import Pin, I2C, PWM, UART
import time
import VL53L0X
from bno08x_rvc import BNO08x_RVC

# setup IMU in RVC mode on UART
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
print(uart)
rvc = BNO08x_RVC(uart)
yaw, r, p, x, y, z = rvc.heading

# setup onboard LED and turn it on
led = Pin("LED", Pin.OUT, value=1)

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


def setup_tof_sensor(bus_id, sda_pin, scl_pin):
    """Setup a Vcsel sensor on an I2C bus.
    There are two available busses: 0 & 1.
    Return VL53L0X object."""
    sda = Pin(sda_pin)
    scl = Pin(scl_pin)

    print("setting up i2c%s" % bus_id)
    i2c = I2C(id=bus_id, sda=sda, scl=scl)
    print("Set up device %s on i2c%s" % (i2c.scan(), bus_id))

    return VL53L0X.VL53L0X(i2c)


tof0 = setup_tof_sensor(0, 12, 13)  # right sensor
tof1 = setup_tof_sensor(1, 14, 15)  # rear sensor
tof0.start()
tof1.start()

def go_back_to_wall(space):
    """Go until back sensor distance == space (mm)"""
    # Motor control parameters
    nom_spd = 45_000
    kp = 1500  # Proportional PID coefficient
    kd = 2500  # Derivative PID coefficient (dissipative)
    p_trim = 0
    d_trim = 0
    p_trim_limit = 10_000
    yaw_prev = 0

    try:
        d1 = tof1.read()
        set_mtr_dirs('REV', 'REV')
        set_mtr_spds(nom_spd, nom_spd)
        while  d1 > space:
            d1 = tof1.read()
            yaw, r, p, x, y, z = rvc.heading
            p_trim = int(yaw * kp)
            if p_trim > p_trim_limit:
                p_trim = p_trim_limit
            elif p_trim < -p_trim_limit:
                p_trim = -p_trim_limit
            d_trim = int( (yaw - yaw_prev) * kd)
            yaw_prev = yaw
            a_spd = nom_spd - p_trim - d_trim
            b_spd = nom_spd + p_trim + d_trim
            print(yaw, d1, a_spd, b_spd, p_trim, d_trim)
            set_mtr_spds(a_spd, b_spd)
            time.sleep(0.01)
        set_mtr_dirs('OFF', 'OFF')
        print('stop')
        
        # Print some values as car coasts to a complete stop
        for cycle in range(15):
            d1 = tof1.read()
            yaw, p, r, x, y, z = rvc.heading
            print(yaw, d1)
            time.sleep(0.01)
    finally:
        led.off()
        tof0.stop()
        tof1.stop()
        set_mtr_dirs('OFF', 'OFF')
        set_mtr_spds(0, 0)


if __name__ == "__main__":
    go_back_to_wall(250)
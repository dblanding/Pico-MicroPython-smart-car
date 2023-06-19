# car geometry
TRACK_WIDTH = 0.1778  # meters (7 inches)
WHEEL_CIRC = 0.214  # meters

# encoder / gearbox
TICKS_PER_REV = 2464
METERS_PER_TICK = WHEEL_CIRC / TICKS_PER_REV
TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRC

# nominal motor speed for driving straight (ticks per sec)
TARGET_TICK_RATE = 4000

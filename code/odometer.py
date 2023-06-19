import math
from parameters import TRACK_WIDTH, METERS_PER_TICK


class Odometer():
    """
    Calculate and return current pose based on incremental changes in
    encoder values (a & b)
    length dimensions are in meters
    angle dimensions are in radians (+) CCW from X axis.
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.ang = 0.0
        self.prev_enc_a_val = 0
        self.prev_enc_b_val = 0

    def update(self, enc_a_val, enc_b_val):
        """
        Update current pose by incrementing from previous pose
        by the change in encoder values from previous values.
        """

        # find change in encoder values
        delta_enc_a = enc_a_val - self.prev_enc_a_val
        delta_enc_b = enc_b_val - self.prev_enc_b_val
        self.prev_enc_a_val = enc_a_val
        self.prev_enc_b_val = enc_b_val

        # incremental distance fwd of car
        delta_dist_fwd = ((delta_enc_a + delta_enc_b) / 2) * METERS_PER_TICK

        # incremental angle change of car
        delta_ang = (delta_enc_b - delta_enc_a) * METERS_PER_TICK / TRACK_WIDTH

        # convert polar coords of incremental car motion to rect coords
        delta_x, delta_y = self.p2r(delta_dist_fwd, self.ang)

        # update x, y coords of pose
        self.x += delta_x
        self.y += delta_y

        # update pose angle
        self.ang += delta_ang

        return (self.x, self.y, self.ang)

    def p2r(self, r, theta):
        """Convert polar coords to rectangular"""
        x = math.cos(theta) * r
        y = math.sin(theta) * r
        return (x, y)
        

        
